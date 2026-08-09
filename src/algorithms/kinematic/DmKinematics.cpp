#include "DmKinematics.h"
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <iostream>
#include <cmath>
#include <Eigen/SVD>

// ---------- 辅助函数：旋转矩阵正交化 ----------
static Eigen::Matrix3d orthogonalizeRotation(const Eigen::Matrix3d& R) {
    Eigen::JacobiSVD<Eigen::Matrix3d> svd(R, Eigen::ComputeFullU | Eigen::ComputeFullV);
    return svd.matrixU() * svd.matrixV().transpose();
}

// ---------- 归一化角度到 [-π, π] ----------
static double normalizeAngle(double theta) {
    theta = std::fmod(theta + M_PI, 2 * M_PI);
    if (theta < 0) theta += 2 * M_PI;
    return theta - M_PI;
}

DmKinematics::DmKinematics(const std::string& urdf_path, const std::string& ee_frame_name) {
    try {
        pinocchio::urdf::buildModel(urdf_path, model);
        data = pinocchio::Data(model);

        if (model.existFrame(ee_frame_name)) {
            ee_frame_id = model.getFrameId(ee_frame_name);
        } else {
            std::cerr << "[DmKinematics] 错误: 找不到 frame '" << ee_frame_name 
                      << "'，使用最后一个 link 作为末端。" << std::endl;
            ee_frame_id = model.nframes - 1;
        }

        std::cout << "[DmKinematics] 模型加载成功，自由度: " << model.nq << std::endl;
        std::cout << "[DmKinematics] 末端 frame: " << model.frames[ee_frame_id].name << std::endl;
        std::cout << "[DmKinematics] 关节限位:" << std::endl;
        for (int i = 0; i < model.nq; ++i) {
            std::cout << "  J" << i << ": [" << model.lowerPositionLimit[i] 
                      << ", " << model.upperPositionLimit[i] << "] rad" << std::endl;
        }
    } catch (const std::exception& e) {
        std::cerr << "[DmKinematics] 加载模型失败: " << e.what() << std::endl;
        throw;
    }
}

Eigen::VectorXd DmKinematics::forward(const Eigen::VectorXd& q) {
    pinocchio::forwardKinematics(model, data, q);
    pinocchio::updateFramePlacements(model, data);
    const pinocchio::SE3& pos_ee = data.oMf[ee_frame_id];
    Eigen::Matrix3d R = orthogonalizeRotation(pos_ee.rotation());
    Eigen::Vector3d rpy = pinocchio::rpy::matrixToRpy(R);
    Eigen::VectorXd result(6);
    result.head<3>() = pos_ee.translation();
    result.tail<3>() = rpy;
    return result;
}

bool DmKinematics::inverse(const Eigen::Vector3d& target_pos,
                           const Eigen::Vector3d& target_rpy,
                           const Eigen::VectorXd& q_init,
                           Eigen::VectorXd& q_out,
                           bool enable_orientation,
                           double pos_weight,
                           double ori_weight) {
    const double EPS = 1e-3;          // 1 mm
    const double EPS_ORI = 1.745e-3;  // 0.1° (弧度)
    const int IT_MAX = 400;
    const double DT = 0.5;
    const double DAMP_MIN = 1e-6;
    const double DAMP_MAX = 1e-2;
    const int IT_POS_ONLY = enable_orientation ? 20 : 0;

    q_out = q_init;
    for (int j = 0; j < q_out.size(); ++j)
        q_out[j] = normalizeAngle(q_out[j]);

    Eigen::Matrix3d target_rot = pinocchio::rpy::rpyToMatrix(target_rpy);
    pinocchio::SE3 oMdes(target_rot, target_pos);

    typedef Eigen::Matrix<double, 6, 1> Vector6d;
    Vector6d err_raw;

    for (int i = 0; i < IT_MAX; ++i) {
        pinocchio::forwardKinematics(model, data, q_out);
        pinocchio::updateFramePlacements(model, data);
        const pinocchio::SE3& dMi = data.oMf[ee_frame_id];

        err_raw = pinocchio::log6(dMi.inverse() * oMdes).toVector();
        double pos_err = err_raw.head<3>().norm();
        double ori_err = err_raw.tail<3>().norm();

        if (pos_err < EPS && (!enable_orientation || ori_err < EPS_ORI)) {
            std::cout << "[DmKinematics] 逆解收敛 | 迭代: " << i 
                      << " | 位置误差: " << pos_err * 1000 << " mm"
                      << " | 姿态误差: " << ori_err * 180 / M_PI << " deg\n";
            return true;
        }

        Vector6d err_weighted;
        if (enable_orientation && i >= IT_POS_ONLY) {
            err_weighted.head<3>() = err_raw.head<3>() * pos_weight;
            err_weighted.tail<3>() = err_raw.tail<3>() * ori_weight;
        } else {
            err_weighted.head<3>() = err_raw.head<3>() * pos_weight;
            err_weighted.tail<3>().setZero();
        }

        pinocchio::Data::Matrix6x J(6, model.nv);
        J.setZero();
        pinocchio::computeFrameJacobian(model, data, q_out, ee_frame_id, pinocchio::LOCAL, J);

        int row_used = (enable_orientation && i >= IT_POS_ONLY) ? 6 : 3;
        Eigen::MatrixXd J_used = J.topRows(row_used);
        Eigen::VectorXd err_used = err_weighted.head(row_used);

        Eigen::JacobiSVD<Eigen::MatrixXd> svd(J_used, Eigen::ComputeThinU | Eigen::ComputeThinV);
        double sigma_max = svd.singularValues()(0);
        double sigma_min = svd.singularValues()(svd.singularValues().size() - 1);
        double damp = DAMP_MIN + (DAMP_MAX - DAMP_MIN) * std::exp(-sigma_min / sigma_max);

        Eigen::VectorXd dq(model.nv);
        dq.setZero();
        for (int k = 0; k < svd.singularValues().size(); ++k) {
            double sigma = svd.singularValues()(k);
            if (sigma > 1e-6) {
                double scale = sigma / (sigma * sigma + damp * damp);
                dq += scale * svd.matrixV().col(k) * svd.matrixU().col(k).dot(err_used);
            }
        }

        q_out = pinocchio::integrate(model, q_out, dq * DT);

        for (int j = 0; j < q_out.size(); ++j) {
            if (model.lowerPositionLimit[j] > -1e9) {
                q_out[j] = std::max(model.lowerPositionLimit[j], 
                                    std::min(model.upperPositionLimit[j], q_out[j]));
            } else {
                q_out[j] = normalizeAngle(q_out[j]);
            }
        }
    }

    pinocchio::forwardKinematics(model, data, q_out);
    pinocchio::updateFramePlacements(model, data);
    const pinocchio::SE3& final_pose = data.oMf[ee_frame_id];
    Vector6d final_err = pinocchio::log6(final_pose.inverse() * oMdes).toVector();
    double pos_err = final_err.head<3>().norm();
    double ori_err = final_err.tail<3>().norm();

    std::cerr << "[DmKinematics] 逆解失败！" << std::endl;
    std::cerr << "  目标位置: (" << target_pos.transpose() << ")" << std::endl;
    std::cerr << "  实际位置误差: " << pos_err * 1000 << " mm" << std::endl;
    std::cerr << "  实际姿态误差: " << ori_err * 180 / M_PI << " deg" << std::endl;
    return false;
}
Eigen::VectorXd DmKinematics::computeGravityTorque(const Eigen::VectorXd& q) {
    // 1. 零速度、零加速度
    Eigen::VectorXd dq = Eigen::VectorXd::Zero(model.nv);
    Eigen::VectorXd ddq = Eigen::VectorXd::Zero(model.nv);

    // 2. 调用递归牛顿欧拉算法，重力向量默认 (0,0,-9.81)
    //    Pinocchio 会自动从 model.gravity 读取，默认 -9.81 沿 Z 轴
    pinocchio::rnea(model, data, q, dq, ddq);

    // 3. 返回广义重力力矩
    return data.tau;   // data.tau 是 rnea 的结果，仅包含重力和科氏力，此处 dq=0，只有重力项
}