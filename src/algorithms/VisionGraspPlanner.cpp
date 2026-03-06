#include "VisionGraspPlanner.h"
#include <iostream>

VisionGraspPlanner::VisionGraspPlanner(const std::string& yamlPath) {
    T_ee_cam = Eigen::Matrix4d::Identity();
    // 默认手动偏移设为0，你可以在此根据实测结果修改
    // 例如：发现固定偏右 2cm，则 manual_offset = Eigen::Vector3d(0.0, -0.02, 0.0);
    manual_offset = Eigen::Vector3d(0.0, 0.0, 0.0); 

    if (!loadCalibration(yamlPath)) {
        std::cerr << "[VisionGraspPlanner] 严重错误：无法加载标定文件，使用单位阵！" << std::endl;
    }
}

bool VisionGraspPlanner::loadCalibration(const std::string& path) {
    try {
        YAML::Node config = YAML::LoadFile(path);
        
        // 1. 读取 3x3 旋转矩阵 (YAML中是 9 个元素的列表)
        if (config["rotation"] && config["rotation"].IsSequence()) {
            std::vector<double> rot = config["rotation"].as<std::vector<double>>();
            if (rot.size() == 9) {
                // 按行主序填入 Eigen 矩阵
                T_ee_cam(0,0) = rot[0]; T_ee_cam(0,1) = rot[1]; T_ee_cam(0,2) = rot[2];
                T_ee_cam(1,0) = rot[3]; T_ee_cam(1,1) = rot[4]; T_ee_cam(1,2) = rot[5];
                T_ee_cam(2,0) = rot[6]; T_ee_cam(2,1) = rot[7]; T_ee_cam(2,2) = rot[8];
            }
        }

        // 2. 读取平移向量
        if (config["translation"] && config["translation"].IsSequence()) {
            std::vector<double> trans = config["translation"].as<std::vector<double>>();
            if (trans.size() == 3) {
                T_ee_cam(0,3) = trans[0];
                T_ee_cam(1,3) = trans[1];
                T_ee_cam(2,3) = trans[2];
            }
        }

        std::cout << "[VisionGraspPlanner] 成功加载标定文件: " << path << std::endl;
        std::cout << "变换矩阵 T_ee_cam:\n" << T_ee_cam << std::endl;
        return true;

    } catch (const std::exception& e) {
        std::cerr << "[VisionGraspPlanner] 解析 YAML 出错: " << e.what() << std::endl;
        return false;
    }
}

Eigen::Matrix3d VisionGraspPlanner::rpyToMatrix(float r, float p, float y) {
    Eigen::AngleAxisd rollAngle(r, Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd pitchAngle(p, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd yawAngle(y, Eigen::Vector3d::UnitZ());
    return (yawAngle * pitchAngle * rollAngle).toRotationMatrix();
}

Eigen::Vector3d VisionGraspPlanner::getTargetInBase(const float current_ee_pose[6], const float obj_in_cam[3]) {
    // 1. 构建 T_base_ee
    Eigen::Matrix4d T_base_ee = Eigen::Matrix4d::Identity();
    T_base_ee.block<3,3>(0,0) = rpyToMatrix(current_ee_pose[3], current_ee_pose[4], current_ee_pose[5]);
    T_base_ee(0,3) = current_ee_pose[0];
    T_base_ee(1,3) = current_ee_pose[1];
    T_base_ee(2,3) = current_ee_pose[2];

    // 2. 变换计算
    Eigen::Vector4d P_cam(obj_in_cam[0], obj_in_cam[1], obj_in_cam[2], 1.0);
    Eigen::Vector4d P_base = T_base_ee * (T_ee_cam * P_cam);

    // 3. 返回结果 + 手动补偿
    return P_base.head<3>() + manual_offset;
}