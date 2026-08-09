#ifndef VISION_GRASP_PLANNER_H
#define VISION_GRASP_PLANNER_H

#include <Eigen/Dense>
#include <string>
#include <vector>
#include <cmath>
#include <yaml-cpp/yaml.h> // 需要安装 libyaml-cpp-dev

/**
 * @brief 手眼标定坐标转换 + GRCNN 抓取参数映射类
 *
 * 相对原版的改动：
 *  1. getTargetInBase 移除了 YOLO 透视畸变补偿（GRCNN 直接给出抓取点表面坐标，
 *     不存在 YOLO 检测框中心随物体厚度偏移的问题）
 *  2. 新增 widthToGripperAngle()：GRCNN 开口宽度(米) -> 夹爪电机角度
 *  3. 新增抓取角映射参数 angle_sign / angle_offset（配合 main.cpp 中
 *     grasp_yaw = atan2(y,x) + YAW_OFFSET + angle_sign*theta + angle_offset）
 */
class VisionGraspPlanner
{
public:
    /**
     * @brief 构造函数
     * @param yamlPath 标定文件的绝对路径
     */
    VisionGraspPlanner(const std::string &yamlPath);

    /**
     * @brief 动态加载 YAML 标定数据
     */
    bool loadCalibration(const std::string &path);

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

    /**
     * @brief GRCNN 开口宽度(米) -> 夹爪电机 SetAngle 角度
     * 线性映射 angle = width_k * width_m + width_b，钳制到 [grip_min, grip_max]。
     * 标定方法见 docs/GRCNN改造说明.md：在两个 SetAngle 下实测开口宽度，解两点式。
     */
    double widthToGripperAngle(double width_m) const
    {
        double a = width_k * width_m + width_b;
        if (a < grip_min)
            a = grip_min;
        if (a > grip_max)
            a = grip_max;
        return a;
    }

    // 核心矩阵 T_ee_cam
    Eigen::Matrix4d T_ee_cam;

    // 手动微调偏移量（基座系固定系统误差补偿）
    // 注意：GRCNN 改造后坐标链路有变化，若抓取出现固定偏差需重新实测此值
    Eigen::Vector3d manual_offset;

    const double GRIPPER_LENGTH = 0.165; // 夹爪长度 16.5cm

    // ================= GRCNN 抓取参数（标定后修改） =================

    // --- 抓取角映射（实物标定，见 docs/GRCNN改造说明.md）---
    double angle_sign = -1.0;  // 先试 -1；若夹爪闭合方向与物体长轴不垂直而是平行，改为 +1
    double angle_offset = 0.0; // 固定零位偏差 (rad)，标定后填入

    // --- 夹爪 宽度->角度 映射（实物标定）---
    double width_k = 40.0;  // rad/m，开口每米对应的电机角（估值，需实测）
    double width_b = 0.5;   // 截距（估值，需实测）
    double grip_min = 0.2;  // 最小 SetAngle（防撞）
    double grip_max = 4.0;  // 最大 SetAngle（与原代码张爪值一致）

    // 建议：如果标定准确，这个偏移应该在“相机坐标系”下定义
    Eigen::Vector3d cam_offset = Eigen::Vector3d::Zero();

private:
    std::string configPath;
};

#endif
