#include "VisionGraspPlanner.h"
#include <iostream>

VisionGraspPlanner::VisionGraspPlanner(const std::string &yamlPath)
{
    T_ee_cam = Eigen::Matrix4d::Identity();
    // 沿用实测的固定偏移（GRCNN 改造后若出现固定偏差需重新实测，见 docs）
    // manual_offset = Eigen::Vector3d(0.0236, -0.0168, -0.05);
    manual_offset = Eigen::Vector3d(0.0, -0.0168, -0.02);
    // manual_offset = Eigen::Vector3d::Zero();

    if (!loadCalibration(yamlPath))
    {
        std::cerr << "[VisionGraspPlanner] 严重错误：无法加载标定文件，使用单位阵！" << std::endl;
    }
}

bool VisionGraspPlanner::loadCalibration(const std::string &path)
{
    try
    {
        YAML::Node config = YAML::LoadFile(path);

        // 1. 读取 3x3 旋转矩阵 (YAML中是 9 个元素的列表)
        if (config["rotation"] && config["rotation"].IsSequence())
        {
            std::vector<double> rot = config["rotation"].as<std::vector<double>>();
            if (rot.size() == 9)
            {
                // 按行主序填入 Eigen 矩阵
                T_ee_cam(0, 0) = rot[0];
                T_ee_cam(0, 1) = rot[1];
                T_ee_cam(0, 2) = rot[2];
                T_ee_cam(1, 0) = rot[3];
                T_ee_cam(1, 1) = rot[4];
                T_ee_cam(1, 2) = rot[5];
                T_ee_cam(2, 0) = rot[6];
                T_ee_cam(2, 1) = rot[7];
                T_ee_cam(2, 2) = rot[8];
            }
        }

        // 2. 读取平移向量
        if (config["translation"] && config["translation"].IsSequence())
        {
            std::vector<double> trans = config["translation"].as<std::vector<double>>();
            if (trans.size() == 3)
            {
                T_ee_cam(0, 3) = trans[0];
                T_ee_cam(1, 3) = trans[1];
                T_ee_cam(2, 3) = trans[2];
            }
        }

        std::cout << "[VisionGraspPlanner] 成功加载标定文件: " << path << std::endl;
        std::cout << "变换矩阵 T_ee_cam:\n"
                  << T_ee_cam << std::endl;
        return true;
    }
    catch (const std::exception &e)
    {
        std::cerr << "[VisionGraspPlanner] 解析 YAML 出错: " << e.what() << std::endl;
        return false;
    }
}

Eigen::Matrix3d VisionGraspPlanner::rpyToMatrix(float r, float p, float y)
{
    Eigen::AngleAxisd rollAngle(r, Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd pitchAngle(p, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd yawAngle(y, Eigen::Vector3d::UnitZ());
    return (yawAngle * pitchAngle * rollAngle).toRotationMatrix();
}

Eigen::Vector3d VisionGraspPlanner::getTargetInBase(const float current_ee_pose[6],
                                                    const float obj_in_cam[3])
{
    // 1. 构建 T_base_ee (当前法兰盘/Link 6 相对于基座的位姿)
    Eigen::Matrix4d T_base_ee = Eigen::Matrix4d::Identity();
    Eigen::Matrix3d R_base_ee = rpyToMatrix(current_ee_pose[3], current_ee_pose[4],
                                            current_ee_pose[5]);
    T_base_ee.block<3, 3>(0, 0) = R_base_ee;
    T_base_ee(0, 3) = current_ee_pose[0];
    T_base_ee(1, 3) = current_ee_pose[1];
    T_base_ee(2, 3) = current_ee_pose[2];

    // 2. GRCNN 直接给出抓取点的相机系坐标（像素反投影 + 该点真实深度），
    //    无需 YOLO 时代的透视畸变补偿，直接构建齐次坐标
    Eigen::Vector4d P_cam(obj_in_cam[0],
                          obj_in_cam[1],
                          obj_in_cam[2],
                          1.0);

    // 3. 计算物体在机器人基座系下的【绝对空间坐标】
    //    公式：P_base_obj = T_base_ee * T_ee_cam * P_cam
    Eigen::Vector4d P_base_obj = T_base_ee * (T_ee_cam * P_cam);

    Eigen::Vector3d final_target = P_base_obj.head<3>();

    // 固定基座系统误差补偿（重新标定方法见 docs/GRCNN改造说明.md）
    final_target += manual_offset;

    return final_target;
}
