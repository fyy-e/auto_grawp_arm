// main.cpp
#include <iostream>
#include <thread>
#include <chrono>
#include <atomic>
#include "dummy_robot.h"
#include "socketcan.h"

// 全局变量
std::atomic<bool> g_thread_running(true);

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

        if(i++ == 10){
            robot->SetEnable(true);
            i = 0;
        }

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
    std::cout << "===== 机械臂控制系统测试（SI单位：米 + 弧度） =====" << std::endl;
    // 1. 初始化 CAN 总线
    SocketCan can_bus;
    if (!can_bus.open("can0")) {
        std::cerr << "CAN 总线初始化失败！" << std::endl;
        return EXIT_FAILURE;
    } else {
        std::cout << "CAN 总线初始化完成！" << std::endl;  
    }
    
    // 2. 创建机械臂对象（需要提供URDF路径）
    DummyRobot robot(&can_bus, "/home/fyy/桌面/arm_motionController_ws/change_dummy_v4.0/src/urdf/berkeley_humanoid_lite/urdf/arm_six.urdf");
    // robot.CalibrateHomeOffset();
    // 3. 初始化机械臂
    std::cout << "初始化机械臂..." << std::endl;
    robot.Init();
    robot.SetEnable(true);
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
        // // 测试1: 关节空间运动（输入为弧度）
        // std::cout << "\n测试1: 关节空间运动 (MoveJ)..." << std::endl;
        // PrintJointAngles(robot.GetCurrentJoints());
        // // PrintJointAnglesDeg(robot.GetCurrentJoints());  // 同时打印度数方便查看

        // // 目标关节角（弧度）：
        // // J1=-0.02rad, J2=-0.23rad, J3=0.03rad, J4=-1.57rad(约-90°), J5=0, J6=0
        // bool move_success = robot.MoveJ(-0.02f, 0.785f, 0.03f, -1.57f, 0.0f, 0.0f);
        
        // if (move_success) {
        //     std::cout << "运动指令已下发（目标：弧度）" << std::endl;
            
        //     // 驱动关节运动（使用getter获取targetJoints）
        //     robot.MoveJoints(robot.GetTargetJoints());
        //     std::this_thread::sleep_for(std::chrono::milliseconds(50));
            
        //     // 等待运动完成
        //     while (robot.IsMoving()) {
        //         std::cout << "运动中..." << std::endl;
        //         PrintJointAngles(robot.GetCurrentJoints());
        //         std::this_thread::sleep_for(std::chrono::milliseconds(100));
        //     }
        //     std::cout << "运动完成!" << std::endl;
        //     PrintJointAngles(robot.GetCurrentJoints());
        // } else {
        //     std::cout << "运动指令非法（可能超出限位）!" << std::endl;
        // }

        // std::this_thread::sleep_for(std::chrono::milliseconds(2000));

        // // 测试2: 笛卡尔空间运动（输入为米和弧度）
        // std::cout << "\n测试2: 笛卡尔空间运动 (MoveL)..." << std::endl;
        // robot.UpdateJointPose6D();  // 更新正运动学
        // PrintPose(robot.GetCurrentPose());
        // PrintJointAnglesDeg(robot.GetCurrentJoints());

        // // 获取当前位姿（米和弧度）
        // auto current_pose = robot.GetCurrentPose();
        
        // // 计算目标位置：当前位置上方 0.1m（10cm），保持姿态不变
        // float target_x = current_pose.X;
        // float target_y = current_pose.Y;
        // float target_z = current_pose.Z + 0.1f;  // +0.1米 = +10厘米
        
        // std::cout << "向Z+0.1m移动..." << std::endl;
        
        // // MoveL输入：米（X,Y,Z），弧度（Roll,Pitch,Yaw）
        // move_success = robot.MoveL(target_x, target_y, target_z, 
        //                            current_pose.roll, 
        //                            current_pose.pitch, 
        //                            current_pose.yaw);
        
        // if (move_success) {
        //     std::cout << "逆运动学求解成功，开始运动" << std::endl;
        //     robot.MoveJoints(robot.GetTargetJoints());
            
        //     // 等待运动完成
        //     while (robot.IsMoving()) {
        //         std::cout << "运动中..." << std::endl;
        //         PrintPose(robot.GetCurrentPose());
        //         std::this_thread::sleep_for(std::chrono::milliseconds(100));
        //     }
        //     std::cout << "运动完成!" << std::endl;
        //     PrintPose(robot.GetCurrentPose());
        //     PrintJointAnglesDeg(robot.GetCurrentJoints());
        // } else {
        //     std::cout << "逆运动学无解或目标不可达!" << std::endl;
        // }

        // std::this_thread::sleep_for(std::chrono::milliseconds(500));

        // 测试3: 绝对位置运动（指定具体坐标）
        std::cout << "\n测试3: 移动到绝对位置 (0.2m, 0.0m, 0.2m)..." << std::endl;
        // 目标：X=0.2m, Y=0, Z=0.3m, 垂直向下（pitch = -π/2）
        bool move_success = robot.MoveL(0.2f, 0.00f, 0.3f, robot.GetCurrentPose().roll, robot.GetCurrentPose().pitch, robot.GetCurrentPose().yaw);
        std::this_thread::sleep_for(std::chrono::milliseconds(2000));
        if (move_success) {
            robot.MoveJoints(robot.GetTargetJoints());
            while (robot.IsMoving()) {
                std::this_thread::sleep_for(std::chrono::milliseconds(50));
            }
            std::cout << "到达目标位置!" << std::endl;
            PrintPose(robot.GetCurrentPose());
        }

        // 测试4: 回到休息姿态
        // std::cout << "\n测试4: 回到休息姿态..." << std::endl;
        // robot.Resting();
        // while (robot.IsMoving()) {
        //     std::this_thread::sleep_for(std::chrono::milliseconds(100));
        // }
        // std::cout << "已回到休息姿态" << std::endl;
        // PrintJointAnglesDeg(robot.GetCurrentJoints());

        // // 测试5: 速度设置（弧度/秒）
        // std::cout << "\n测试5: 调整关节速度..." << std::endl;
        // robot.SetJointSpeed(0.785f);  // 约45°/s = 0.785 rad/s
        // std::cout << "关节速度已设置为 0.785 rad/s (45°/s)" << std::endl;

        // // 测试6: 指令模式切换
        // std::cout << "\n测试6: 切换指令模式..." << std::endl;
        // robot.SetCommandMode(DummyRobot::COMMAND_CONTINUES_TRAJECTORY);
        // std::cout << "已切换到连续轨迹模式" << std::endl;

        // 持续打印状态（用于调试）
        std::cout << "\n进入监控模式（按Ctrl+C退出）..." << std::endl;
        while(g_thread_running) {
            PrintJointAngles(robot.GetCurrentJoints());
            PrintJointAnglesDeg(robot.GetCurrentJoints());
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
    
    robot.SetEnable(false);
    std::cout << "测试完成!" << std::endl;
    
    return 0;
}