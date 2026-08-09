#pragma once

#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/math/rpy.hpp>
#include <Eigen/Dense>
#include <string>
#include <pinocchio/algorithm/rnea.hpp>   // 逆动力学
#include <pinocchio/algorithm/crba.hpp>   // 质量矩阵（可选）

class DmKinematics {
public:
    /**
     * @brief 构造函数，加载 URDF 模型并初始化
     * @param urdf_path URDF 文件路径
     * @param ee_frame_name 末端执行器所在的 frame 名称（默认为 "link_6"）
     */
    DmKinematics(const std::string& urdf_path, 
                 const std::string& ee_frame_name = "tool0_tcp");

    /**
     * @brief 正向运动学
     * @param q 关节角度（弧度），维度 nq
     * @return Eigen::VectorXd 末端位姿 [x, y, z, roll, pitch, yaw] （单位：米，弧度）
     */
    Eigen::VectorXd forward(const Eigen::VectorXd& q);

    /**
     * @brief 逆运动学
     * @param target_pos     目标位置 (x,y,z) 米
     * @param target_rpy     目标姿态 (roll,pitch,yaw) 弧度
     * @param q_init         初始关节角度猜测（弧度）
     * @param q_out          求解结果（弧度）
     * @param enable_orientation 是否启用姿态约束（若机械臂自由度>=6可设为true）
     * @param pos_weight     位置误差权重（默认1.0）
     * @param ori_weight     姿态误差权重（默认1.0，仅在 enable_orientation=true 时生效）
     * @return true 收敛成功，否则失败
     */
    bool inverse(const Eigen::Vector3d& target_pos,
                 const Eigen::Vector3d& target_rpy,
                 const Eigen::VectorXd& q_init,
                 Eigen::VectorXd& q_out,
                 bool enable_orientation = false,
                 double pos_weight = 1.0,
                 double ori_weight = 1.0);

    /// 获取关节数（自由度）
    int getJointNum() const { return model.nq; }

    /// 获取关节限位（若无限位，返回 ±inf）
    void getJointLimits(Eigen::VectorXd& lower, Eigen::VectorXd& upper) const {
        lower = model.lowerPositionLimit;
        upper = model.upperPositionLimit;
    }
    /**
     * @brief 计算当前关节角度下的重力补偿力矩（广义力）
     * @param q 关节角度（弧度），维度 nq
     * @return Eigen::VectorXd 重力矩（N·m），维度 nv（通常 nv == nq）
     */
    Eigen::VectorXd computeGravityTorque(const Eigen::VectorXd& q);

private:
    pinocchio::Model model;
    pinocchio::Data data;
    int ee_frame_id;               // 末端 frame 的索引
};