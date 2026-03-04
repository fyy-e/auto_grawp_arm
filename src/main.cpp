#include <iostream>
#include <thread>
#include <chrono>
#include <atomic>
#include "dummy_robot.h"
#include "src/u2can/SerialPort.h"
// 全局变量
std::atomic<bool> g_thread_running(true);
// 1. 初始化静态指针为 nullptr
std::shared_ptr<SerialPort> CtrlStepMotor::serial = std::make_shared<SerialPort>("/dev/ttyACM0", B921600);
damiao::Motor_Control CtrlStepMotor::dm(CtrlStepMotor::serial);

/**
 * @brief 角度更新线程函数
 * 以固定频率更新关节角度和状态
 */
int i = 0;
void UpdateThread(DummyRobot* robot, int update_rate_hz) {
    using namespace std::chrono;
    
    int update_period_ms = 1000 / update_rate_hz;
    auto next_time = steady_clock::now();
    
    while (g_thread_running) {
        // 更新关节角度

        robot->UpdateJointAngles();
        
        // 更新角度回调（处理状态标志）
        robot->UpdateJointAnglesCallback();

        if (robot->GetDof() > 0) {
            robot->UpdateJointPose6D();
        }
        
        // 固定频率执行
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
    // 2. 创建机械臂对象（需要提供URDF路径）
    DummyRobot robot("/dev/ttyACM0", B921600, "/home/fyy/桌面/arm_motionController_ws/dm_arm_v1.0/src/urdf/urdf/DM_urdf.urdf");
    // robot.CalibrateHomeOffset();
    // 3. 初始化机械臂
    std::cout << "初始化机械臂..." << std::endl;
    robot.Init();
    robot.SetEnable(true,damiao::POS_VEL_MODE);
    robot.hand->SetEnable(true);
    // robot.hand->CalibrateHomeOffset();
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    
    // 4. 启动角度更新线程
    std::cout << "启动角度更新线程 (200Hz)..." << std::endl;
    std::thread update_thread(UpdateThread, &robot, 100);
    
    // 5. 等待机械臂使能完成
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    // 6. 回零
    std::cout << "执行回零操作..." << std::endl;
    robot.Homing();
    std::this_thread::sleep_for(std::chrono::milliseconds(3000));
    
    try {
        // 测试1: 关节空间运动（输入为弧度）
        std::cout << "\n测试1: 关节空间运动 (MoveJ)..." << std::endl;
        PrintJointAngles(robot.GetCurrentJoints());
        // PrintJointAnglesDeg(robot.GetCurrentJoints());  // 同时打印度数方便查看

        // 目标关节角（弧度）：
        // J1=-0.02rad, J2=-0.23rad, J3=0.03rad, J4=-1.57rad(约-90°), J5=0, J6=0
        bool move_success = robot.MoveJ(0.0f, 0.785f, 0.785f, 0.3f, 0.3f, 1.57f);
        
        if (move_success) {
            std::cout << "运动指令已下发（目标：弧度）" << std::endl;
            
            // 驱动关节运动（使用getter获取targetJoints）
            // robot.MoveJoints(robot.GetTargetJoints());
            // std::this_thread::sleep_for(std::chrono::milliseconds(50));
            
            // 等待运动完成
            while (robot.IsMoving()) {
                std::cout << "运动中..." << std::endl;
                PrintJointAngles(robot.GetCurrentJoints());
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }
            robot.hand->SetAngle(2.0f);  // 手爪半开
            std::cout << "运动完成!" << std::endl;
            PrintJointAngles(robot.GetCurrentJoints());
        } else {
            std::cout << "运动指令非法（可能超出限位）!" << std::endl;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(2000));
        robot.Resting();
        // 持续打印状态（用于调试）


        std::cout << "\n测试1: 笛卡尔坐标系运动 (MoveL)..." << std::endl;
        move_success = robot.MoveL(0.356810f, 0.000304f, 0.424000f, 1.570796f, -0.000000f, 1.570796f);
        
        if (move_success) {
            std::cout << "运动指令已下发（目标：弧度）" << std::endl;
            
            // 驱动关节运动（使用getter获取targetJoints）
            // robot.MoveJoints(robot.GetTargetJoints());
            // std::this_thread::sleep_for(std::chrono::milliseconds(50));
            
            // 等待运动完成
            while (robot.IsMoving()) {
                std::cout << "运动中..." << std::endl;
                PrintJointAngles(robot.GetCurrentJoints());
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }
            std::cout << "运动完成!" << std::endl;
            robot.hand->SetAngle(0.0f);  // 手爪半开
            PrintJointAngles(robot.GetCurrentJoints());
        } else {
            std::cout << "运动指令非法（可能超出限位）!" << std::endl;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(2000));
        robot.Homing();
        // 持续打印状态（用于调试）
        std::cout << "\n进入监控模式（按Ctrl+C退出）..." << std::endl;
        while(g_thread_running) {
            PrintJointAngles(robot.GetCurrentJoints());
            // PrintJointAnglesDeg(robot.GetCurrentJoints());
            if (robot.GetDof() > 0) {
                    // robot.UpdateJointPose6D();
                    PrintPose(robot.GetCurrentPose());
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(500));
            }
            
        } catch (const std::exception& e) {
            std::cerr << "测试过程中发生异常: " << e.what() << std::endl;
        }
        
        // 清理
        std::cout << "\n清理资源..." << std::endl;
        g_thread_running = false;
        if (update_thread.joinable()) {
            update_thread.join();
        }
        
        robot.SetEnable(false,damiao::POS_VEL_MODE);
        std::cout << "测试完成!" << std::endl;
        
        return 0;
}