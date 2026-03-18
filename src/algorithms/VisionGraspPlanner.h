#ifndef VISION_GRASP_PLANNER_H
#define VISION_GRASP_PLANNER_H

#include <Eigen/Dense>
#include <string>
#include <vector>
#include <yaml-cpp/yaml.h> // 需要安装 libyaml-cpp-dev

/**
 * @brief 手眼标定坐标转换类 (动态配置版)
 */
class VisionGraspPlanner {
public:
    /**
     * @brief 构造函数
     * @param yamlPath 标定文件的绝对路径
     */
    VisionGraspPlanner(const std::string& yamlPath);

    /**
     * @brief 动态加载 YAML 标定数据
     */
    bool loadCalibration(const std::string& path);

    /**
     * @brief 计算目标在基座坐标系下的位置
     * @param current_ee_pose 当前末端位姿 {X, Y, Z, Roll, Pitch, Yaw}
     * @param obj_in_cam 视觉识别到的物体在相机系坐标 {x, y, z} (米)
     * @return Eigen::Vector3d 基座坐标系下的 {X, Y, Z}
     */
    Eigen::Vector3d getTargetInBase(const float current_ee_pose[6], const float obj_in_cam[3]);

    /**
     * @brief RPY转旋转矩阵 (ZYX顺序)
     */
    Eigen::Matrix3d rpyToMatrix(float r, float p, float y);

    // 核心矩阵 T_ee_cam
    Eigen::Matrix4d T_ee_cam;

    // 手动微调偏移量 (解决你提到的2cm偏差)
    // 建议：如果抓取偏右，修改 Y；如果抓取偏前，修改 X
    Eigen::Vector3d manual_offset;

    // 修改：建议将夹爪长度定义为常量或成员变量，方便维护
    const double GRIPPER_LENGTH = 0.165; // 夹爪长度 16.5cm

    // 建议：如果标定准确，这个 manual_offset 应该是 (0,0,0)
    // 如果相机安装有物理偏差，这个偏移应该在“相机坐标系”下定义
    Eigen::Vector3d cam_offset = Eigen::Vector3d::Zero();

private:
    std::string configPath;
};

#endif