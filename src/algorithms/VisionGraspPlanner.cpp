#include "VisionGraspPlanner.h"
#include <iostream>

VisionGraspPlanner::VisionGraspPlanner(const std::string& yamlPath) {
    T_ee_cam = Eigen::Matrix4d::Identity();
    // 默认手动偏移设为0，你可以在此根据实测结果修改
    // 例如：发现固定偏右 2cm，则 manual_offset = Eigen::Vector3d(0.0, -0.02, 0.0);
    manual_offset = Eigen::Vector3d(0.0236, -0.0168, -0.0527); 
    // manual_offset = Eigen::Vector3d::Zero(); 

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

// // VisionGraspPlanner.cpp 核心逻辑修改
// Eigen::Vector3d VisionGraspPlanner::getTargetInBase(const float current_ee_pose[6], const float obj_in_cam[3]) {
//     // 1. 构建 T_base_ee (当前法兰盘/Link 6 相对于基座的位姿)
//     Eigen::Matrix4d T_base_ee = Eigen::Matrix4d::Identity();
//     Eigen::Matrix3d R_base_ee = rpyToMatrix(current_ee_pose[3], current_ee_pose[4], current_ee_pose[5]);
//     T_base_ee.block<3,3>(0,0) = R_base_ee;
//     T_base_ee(0,3) = current_ee_pose[0];
//     T_base_ee(1,3) = current_ee_pose[1];
//     T_base_ee(2,3) = current_ee_pose[2];

//     // 2. 物体在相机系下的坐标 (Vector4d)
//     // 如果你发现识别结果固定偏左或偏右，应该在这里对 obj_in_cam 进行加减
//     Eigen::Vector4d P_cam(obj_in_cam[0], obj_in_cam[1], obj_in_cam[2], 1.0);

//     // 3. 计算物体在机器人基座系下的【绝对空间坐标】
//     // 公式：P_base_obj = T_base_ee * T_ee_cam * P_cam
//     Eigen::Vector4d P_base_obj = T_base_ee * (T_ee_cam * P_cam);

//     // 4. 【动态补偿夹爪长度】
//     // 目标：我们要计算 Link 6 的位置，使得夹爪指尖刚好碰到物体。
//     // 夹爪是安装在 Link 6 上的，延伸方向就是 Link 6 的 Z 轴。
    
//     // 获取 Link 6 的 Z 轴在基座坐标系下的方向矢量（即旋转矩阵的第三列）
//     Eigen::Vector3d ee_z_axis = R_base_ee.col(2); 

//     // 核心计算：法兰盘目标位置 = 物体空间位置 - (法兰盘Z轴指向 * 夹爪长度)
//     // 这样无论机械臂是斜着抓、横着抓还是竖着抓，补偿方向永远是正确的
//     Eigen::Vector3d final_target = P_base_obj.head<3>();
//     // Eigen::Vector3d final_target = P_base_obj.head<3>() - ee_z_axis * 0.165;

//     // (可选) 如果仍有微小系统误差，仅在 P_base_obj 阶段进行 manual_offset 补偿
//     final_target += manual_offset; 

//     return final_target;
// }
Eigen::Vector3d VisionGraspPlanner::getTargetInBase(const float current_ee_pose[6], const float obj_in_cam[3]) {
    // 1. 构建 T_base_ee (当前法兰盘/Link 6 相对于基座的位姿)
    Eigen::Matrix4d T_base_ee = Eigen::Matrix4d::Identity();
    Eigen::Matrix3d R_base_ee = rpyToMatrix(current_ee_pose[3], current_ee_pose[4], current_ee_pose[5]);
    T_base_ee.block<3,3>(0,0) = R_base_ee;
    T_base_ee(0,3) = current_ee_pose[0];
    T_base_ee(1,3) = current_ee_pose[1];
    T_base_ee(2,3) = current_ee_pose[2];

    // ==========================================
    // 2. 【新增】：YOLO 透视畸变补偿 (Perspective Compensation)
    // ==========================================
    // 设定目标物体的平均物理高度(厚度)，例如魔方或物块高约 4 厘米 (0.04m)
    // 你需要根据实际抓取的物品修改这个值
    const double OBJ_HEIGHT = 0.055; 
    
    // 计算补偿系数 (相似三角形原理：物体顶面中心到光心的连线，映射到桌面时的缩放率)
    double scale = 1.0;
    if (obj_in_cam[2] > OBJ_HEIGHT) {
        // 深度 Z 越大，侧视畸变越小；深度越小，侧视畸变越大
        scale = 1.0 - (OBJ_HEIGHT / obj_in_cam[2]); 
    }

    // 重新构建修正后的相机系坐标
    // X 和 Y 会按照比例向相机光心(0,0)方向收缩，自动抵消侧偏
    Eigen::Vector4d P_cam(obj_in_cam[0] * scale, 
                          obj_in_cam[1] * scale, 
                          obj_in_cam[2], 
                          1.0);

    // 3. 计算物体在机器人基座系下的【绝对空间坐标】
    Eigen::Vector4d P_base_obj = T_base_ee * (T_ee_cam * P_cam);

    // 4. 获取 Link 6 的 Z 轴并输出最终位置
    Eigen::Vector3d ee_z_axis = R_base_ee.col(2); 
    Eigen::Vector3d final_target = P_base_obj.head<3>();

    // 叠加我们上一轮测试出的固定基座系统误差补偿
    final_target += manual_offset; 

    return final_target;
}