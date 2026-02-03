#pragma once
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <Eigen/Dense>
class BerkeleyKinematics {
public:
    BerkeleyKinematics(const std::string& urdf_path);

    // 正向运动学：输入关节角(弧度)，输出 [x,y,z(米), r,p,y(弧度)]
    Eigen::VectorXd forward(const Eigen::VectorXd& q);

    // 逆运动学：返回是否收敛，结果写入 q_out
    bool inverse(const Eigen::Vector3d& target_pos,      // 米
                const Eigen::Vector3d& target_rpy,       // 弧度  
                const Eigen::VectorXd& q_init,           // 弧度
                Eigen::VectorXd& q_out);                 // 输出弧度
                
    // 获取模型维度（5 或 6）
    int getJointNum() const { return model.nq; }
    
    // 获取关节限位（用于外部检查）
    void getJointLimits(Eigen::VectorXd& lower, Eigen::VectorXd& upper) const {
        lower = model.lowerPositionLimit;
        upper = model.upperPositionLimit;
    }

private:
    pinocchio::Model model;
    pinocchio::Data data;
    int ee_frame_id;
};