#include "BerkeleyKinematics.h"
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/math/rpy.hpp>
#include <iostream>

BerkeleyKinematics::BerkeleyKinematics(const std::string& urdf_path) {
    try {
        pinocchio::urdf::buildModel(urdf_path, model);
        data = pinocchio::Data(model);
        ee_frame_id = model.getFrameId("arm_right_hand_link");
        
        if (ee_frame_id >= (int)model.nframes) {
            std::cerr << "[Kinematics] 警告: 找不到 Frame 'arm_right_hand_link'" << std::endl;
        }
        std::cout << "[Kinematics] 模型加载成功，自由度: " << model.nq << std::endl;
        
        // 打印关节限位，确认单位是弧度
        std::cout << "[Kinematics] 关节限位:" << std::endl;
        for(int i=0; i<model.nq; ++i) {
            std::cout << "  J" << i << ": [" << model.lowerPositionLimit[i] 
                      << ", " << model.upperPositionLimit[i] << "] rad" << std::endl;
        }
    } catch (const std::exception& e) {
        std::cerr << "[Kinematics] 加载模型失败: " << e.what() << std::endl;
        throw;
    }
}

Eigen::VectorXd BerkeleyKinematics::forward(const Eigen::VectorXd& q) {
    pinocchio::forwardKinematics(model, data, q);
    pinocchio::updateFramePlacements(model, data);

    const pinocchio::SE3 &pos_ee = data.oMf[ee_frame_id];
    Eigen::Vector3d rpy = pinocchio::rpy::matrixToRpy(pos_ee.rotation());

    Eigen::VectorXd result(6);
    result.head<3>() = pos_ee.translation();
    result.tail<3>() = rpy;
    return result;
}

bool BerkeleyKinematics::inverse(const Eigen::Vector3d& target_pos, 
                                const Eigen::Vector3d& target_rpy, 
                                const Eigen::VectorXd& q_init,
                                Eigen::VectorXd& q_out) {
    // --- 核心控制参数 ---
    const double EPS = 5e-2;      // 精度设为 1mm
    const int IT_MAX = 100;       // 进一步缩减迭代次数，防止阻塞 CAN 通信
    const double DT = 0.1;        // 迭代步长
    const double DAMP_MAX = 1e-2; // 最大阻尼
    const double DAMP_MIN = 1e-4; // 基础阻尼

    q_out = q_init;
    
    // 目标位姿转为 SE3 矩阵
    Eigen::Matrix3d target_rot = pinocchio::rpy::rpyToMatrix(target_rpy(0), target_rpy(1), target_rpy(2));
    pinocchio::SE3 oMdes(target_rot, target_pos);

    typedef Eigen::Matrix<double, 6, 1> Vector6d;
    
    // --- 5轴臂优先级权重配置 ---
    // [X, Y, Z, Roll, Pitch, Yaw]
    // 将姿态权重设为极低 (0.01)，确保位置解算不被姿态误差干扰
    Vector6d weight;
    weight << 1.0, 1.0, 1.0, 0.00, 0.00, 0.0; 

    for (int i = 0; i < IT_MAX; i++) {
        pinocchio::forwardKinematics(model, data, q_out);
        pinocchio::updateFramePlacements(model, data);
        const pinocchio::SE3 &dMi = data.oMf[ee_frame_id];
        
        // 计算空间误差向量
        Vector6d err = pinocchio::log6(dMi.inverse() * oMdes).toVector();
        
        // 应用权重：位置是硬指标，姿态是软指标
        err = err.cwiseProduct(weight);

        // 判定准则：只要加权误差（主要是位置）达标即视为成功
        if (err.norm() < EPS) {
            std::cout << "[IK] 成功收敛 | 迭代: " << i << " | 最终加权误差: " << err.norm() << std::endl;
            return true;
        }

        // 计算当前关节状态下的雅可比矩阵 (LOCAL 坐标系)
        pinocchio::Data::Matrix6x J(6, model.nv);
        J.setZero();
        pinocchio::computeFrameJacobian(model, data, q_out, ee_frame_id, pinocchio::LOCAL, J);

        // 阻尼最小二乘求解：随迭代增加阻尼以应对奇异位姿
        Eigen::MatrixXd JJt = J * J.transpose();
        double damp = DAMP_MIN + (DAMP_MAX - DAMP_MIN) * (static_cast<double>(i) / IT_MAX);
        JJt.diagonal().array() += damp;
        
        Eigen::VectorXd dq = J.transpose() * JJt.ldlt().solve(err);

        // 更新关节角
        q_out = pinocchio::integrate(model, q_out, dq * DT);

        // 强制关节限位约束 (URDF Limit)
        for (int j = 0; j < q_out.size(); ++j) {
            q_out[j] = std::max(model.lowerPositionLimit[j], 
                               std::min(model.upperPositionLimit[j], q_out[j]));
        }
    }
    
    // --- 失败诊断 ---
    pinocchio::forwardKinematics(model, data, q_out);
    pinocchio::updateFramePlacements(model, data);
    const pinocchio::SE3 &final_pose = data.oMf[ee_frame_id];
    Vector6d final_err = pinocchio::log6(final_pose.inverse() * oMdes).toVector();
    
    std::cerr << "[IK] 失败：点可能不可达或姿态冲突" << std::endl;
    std::cerr << "  请求坐标: (" << target_pos.transpose() << ")" << std::endl;
    std::cerr << "  实际位置误差: " << final_err.head<3>().norm() << " m" << std::endl;
    std::cerr << "  实际旋转误差: " << final_err.tail<3>().norm() << " rad" << std::endl;
    
    return false;
}