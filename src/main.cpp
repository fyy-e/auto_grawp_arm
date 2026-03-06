#include <iostream>
#include <thread>
#include <chrono>
#include <atomic>
#include "dummy_robot.h"
#include "src/u2can/SerialPort.h"
#include "algorithms/VisionGraspPlanner.h" // - 新增：引入视觉规划器

// 全局变量
std::atomic<bool> g_thread_running(true);
std::shared_ptr<SerialPort> CtrlStepMotor::serial = std::make_shared<SerialPort>("/dev/ttyACM0", B921600);
damiao::Motor_Control CtrlStepMotor::dm(CtrlStepMotor::serial);

/**
 * @brief 角度更新线程函数
 */
void UpdateThread(DummyRobot* robot, int update_rate_hz) {
    using namespace std::chrono;
    int update_period_ms = 1000 / update_rate_hz;
    auto next_time = steady_clock::now();
    
    while (g_thread_running) {
        robot->UpdateJointAngles();
        robot->UpdateJointAnglesCallback();
        if (robot->GetDof() > 0) {
            robot->UpdateJointPose6D();
        }
        next_time += milliseconds(update_period_ms);
        std::this_thread::sleep_until(next_time);
    }
}

/**
 * @brief 打印当前关节角度（弧度）
 */
void PrintJointAngles(const DummyRobot::Joint6D_t& joints) {
    std::cout << "当前关节角度: ";
    for (int i = 0; i < 6; i++) {
        std::cout << "J" << i+1 << "=" << joints.j[i] << "rad ";
    }
    std::cout << std::endl;
}

/**
 * @brief 打印末端位姿（米和弧度）
 */
void PrintPose(const DummyRobot::Pose6D_t& pose) {
    std::cout << "末端位姿: X=" << pose.X << "m "
              << "Y=" << pose.Y << "m "
              << "Z=" << pose.Z << "m "
              << "Roll=" << pose.roll << "rad "
              << "Pitch=" << pose.pitch << "rad "
              << "Yaw=" << pose.yaw << "rad"
              << std::endl;
}

/**
 * @brief 打印关节角度（带度数转换，用于人工阅读）
 */
void PrintJointAnglesDeg(const DummyRobot::Joint6D_t& joints) {
    const float RAD_TO_DEG = 180.0f / 3.14159265f;
    std::cout << "当前关节角度: ";
    for (int i = 0; i < 6; i++) {
        std::cout << "J" << i+1 << "=" << joints.j[i] * RAD_TO_DEG << "° ";
    }
    std::cout << std::endl;
}

int main() {
    // 1. 定义路径 (建议使用绝对路径确保加载成功)
    std::string urdf_path = "/home/fyy/桌面/arm_motionController_ws/dm_arm_v2.0/src/urdf/urdf/DM_urdf.urdf";
    std::string calib_path = "/home/fyy/桌面/arm_motionController_ws/dm_arm_v2.0/src/config/handeye_result_realsense.yaml";

    // 2. 创建机器人和视觉规划器对象
    DummyRobot robot("/dev/ttyACM0", B921600, urdf_path);
    VisionGraspPlanner grasp_planner(calib_path); // 自动从 YAML 加载标定参数

    // 初始化机器人
    robot.hand->SetEnable(true);
    // robot.CalibrateHomeOffset();
    std::cout << "初始化机械臂..." << std::endl;
    robot.Init();
    robot.SetEnable(true, damiao::POS_VEL_MODE);
    // robot.hand->CalibrateHomeOffset();
    
    std::thread update_thread(UpdateThread, &robot, 100);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    std::cout << "执行回零操作..." << std::endl;
    robot.Homing();
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));
    robot.Resting();
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));

    try {
        // --- 原有的测试 1 & 2 保持不变 ---
        
        // ================================================================
        // 测试3: 视觉引导抓取测试 (新增)
        // ================================================================
        std::cout << "\n测试3: 视觉引导抓取测试..." << std::endl;

        // A. 模拟视觉数据：假设相机在相机坐标系下检测到物体在 (x, y, z)
        // 注意：单位必须是米。例如 z=0.45m 表示物体距离相机 45 厘米
        float obj_in_cam[3] = {0.02f, -0.01f, 0.25f}; 

        // B. 获取当前机械臂末端的 6D 位姿
        auto cp = robot.GetCurrentPose();
        float current_p[6] = {cp.X, cp.Y, cp.Z, cp.roll, cp.pitch, cp.yaw};

        // C. 调用规划器：将物体坐标转换到基座坐标系
        Eigen::Vector3d target_base = grasp_planner.getTargetInBase(current_p, obj_in_cam);

        std::cout << ">>> 视觉转换结果: " 
                  << "Base_X=" << target_base.x() << " "
                  << "Base_Y=" << target_base.y() << " "
                  << "Base_Z=" << target_base.z() << std::endl;

        // D. 执行抓取动作流程
        // 1. 移动到目标点上方 10cm (预备位)
        std::cout << "正在移动到预备抓取位..." << std::endl;
        bool pre_move = robot.MoveL(target_base.x(), target_base.y(), target_base.z() + 0.20f, 
                                    3.14f, 0.0f, 1.57f); // 假设末端垂直向下姿态

        if (pre_move) {
            while (robot.IsMoving()) std::this_thread::sleep_for(std::chrono::milliseconds(100));
            
            // 2. 打开手爪
            robot.hand->SetAngle(0.78f); 
            std::this_thread::sleep_for(std::chrono::milliseconds(500));

            // 3. 下降到实际目标点位置
            std::cout << "下降执行抓取..." << std::endl;
            robot.MoveL(target_base.x(), target_base.y(), target_base.z()+ 0.15f, 3.14f, 0.0f, 1.57f);
            while (robot.IsMoving()) std::this_thread::sleep_for(std::chrono::milliseconds(100));

            // 4. 闭合手爪
            robot.hand->SetAngle(0.0f);
            std::this_thread::sleep_for(std::chrono::milliseconds(800));

            // 5. 抬升
            robot.MoveL(target_base.x(), target_base.y(), target_base.z() + 0.20f, 3.14f, 0.0f, 1.57f);
            while (robot.IsMoving()) std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        robot.Resting();
        // ================================================================

        // 持续监控
        std::cout << "\n进入监控模式..." << std::endl;
        while(g_thread_running) {
            PrintPose(robot.GetCurrentPose());
            PrintJointAnglesDeg(robot.GetCurrentJoints());
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
        }
            
    } catch (const std::exception& e) {
        std::cerr << "异常: " << e.what() << std::endl;
    }
    
    // 清理
    g_thread_running = false;
    if (update_thread.joinable()) update_thread.join();
    robot.SetEnable(false, damiao::POS_VEL_MODE);
    return 0;
}