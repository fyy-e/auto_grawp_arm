#include <iostream>
#include <thread>
#include <chrono>
#include <atomic>
#include <mutex>
#include <Eigen/Dense>
#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>
#include <cstring>

#include "dummy_robot.h"
#include "u2can/SerialPort.h"
#include "algorithms/VisionGraspPlanner.h"
#include "algorithms/VisionDetector.h"

// ================= 全局变量与并发控制 =================
std::atomic<bool> g_thread_running(true);
std::shared_ptr<SerialPort> CtrlStepMotor::serial = std::make_shared<SerialPort>("/dev/ttyACM0", B921600);
damiao::Motor_Control CtrlStepMotor::dm(CtrlStepMotor::serial);
std::mutex g_serial_mtx;

struct SafeTargetPos {
    double x, y, z;
};
std::mutex g_vision_mtx;
SafeTargetPos g_target_pos = {0.0, 0.0, 0.0};
std::atomic<bool> g_target_detected(false);
std::atomic<bool> g_nav_reached(false); // UDP 触发标志

// ================= UDP 监听线程 =================
void UdpListenerThread(int port) {
    int sockfd;
    struct sockaddr_in servaddr, cliaddr;
    if ((sockfd = socket(AF_INET, SOCK_DGRAM, 0)) < 0) return;

    struct timeval tv;
    tv.tv_sec = 1; tv.tv_usec = 0;
    setsockopt(sockfd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

    memset(&servaddr, 0, sizeof(servaddr));
    servaddr.sin_family = AF_INET;
    servaddr.sin_addr.s_addr = INADDR_ANY;
    servaddr.sin_port = htons(port);

    if (bind(sockfd, (const struct sockaddr *)&servaddr, sizeof(servaddr)) < 0) {
        close(sockfd);
        return;
    }

    char buffer[1024];
    socklen_t len = sizeof(cliaddr);
    while (g_thread_running) {
        int n = recvfrom(sockfd, (char *)buffer, 1024, 0, (struct sockaddr *)&cliaddr, &len);
        if (n > 0) {
            buffer[n] = '\0';
            if (std::string(buffer) == "NAV_REACHED_SUCCESS") {
                std::cout << "\033[32m[UDP] 导航到达，准备开始识别抓取！\033[0m" << std::endl;
                g_nav_reached = true;
            }
        }
    }
    close(sockfd);
}

// ================= 视觉处理线程 =================
void VisionThread(VisionDetector* detector, int target_id) {
    double tx, ty, tz;
    const double alpha = 0.4;
    while (g_thread_running) {
        if (detector->GetTargetInCam(target_id, tx, ty, tz)) {
            std::lock_guard<std::mutex> lock(g_vision_mtx);
            if (!g_target_detected) {
                g_target_pos = {tx, ty, tz};
            } else {
                g_target_pos.x = alpha * tx + (1.0 - alpha) * g_target_pos.x;
                g_target_pos.y = alpha * ty + (1.0 - alpha) * g_target_pos.y;
                g_target_pos.z = alpha * tz + (1.0 - alpha) * g_target_pos.z;
            }
            g_target_detected = true;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
}

// ================= 状态更新线程 =================
void UpdateThread(DummyRobot* robot, int update_rate_hz) {
    int update_period_ms = 1000 / update_rate_hz;
    while (g_thread_running) {
        // 给串口操作加锁，防止主线程 MoveL 发送时冲突
        std::lock_guard<std::mutex> lock(g_serial_mtx);
        robot->UpdateJointAngles();
        robot->UpdateJointAnglesCallback();
        if (robot->GetDof() > 0) robot->UpdateJointPose6D();
        std::this_thread::sleep_for(std::chrono::milliseconds(update_period_ms));
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

// ================= 主逻辑 =================
int main() {
    std::string urdf_path = "/home/ysh/dm_arm_v4.0/src/urdf/urdf/DM_urdf.urdf";
    std::string calib_path = "/home/ysh/dm_arm_v4.0/src/config/handeye_result_realsense.yaml";

    DummyRobot robot("/dev/ttyACM0", B921600, urdf_path);
    VisionGraspPlanner grasp_planner(calib_path); 
    VisionDetector detector(0.05f); 

    if (!detector.Init()) return -1;
    // robot.hand->CalibrateHomeOffset();
    // robot.CalibrateHomeOffset();
    robot.hand->SetEnable(true);
    robot.Init();
    robot.SetEnable(true, damiao::POS_VEL_MODE);
    robot.hand->SetAngle(0.0f); 

    std::thread update_thread(UpdateThread, &robot, 100);
    std::thread vision_thread(VisionThread, &detector, 1); 
    std::thread udp_thread(UdpListenerThread, 12346); 

    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    robot.Homing();
    // g_nav_reached = true;
    try {
        while (g_thread_running) {
            std::cout << ">>> [Idle] 等待狗的导航信号..." << std::endl;
            while (!g_nav_reached && g_thread_running) {
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }
            if (!g_thread_running) break;

            // 1. 到达目标点后，先移动到 Resting 位置b
            std::cout << ">>> [Task] 移动到 Resting 位置，准备识别..." << std::endl;
            // std::cout << ">>> [Task] 收到信号，执行 Resting..." << std::endl;
            
            robot.Resting(); 
            std::cout << ">>> [Task] 开始识别..." << std::endl;
            // 2. 像你原来的代码一样，开始持续识别直到成功
            bool task_done = false;
            while (!task_done && g_thread_running) {
                Eigen::Vector3d p_cam;
                bool detected = false;

                {
                    std::lock_guard<std::mutex> lock(g_vision_mtx);
                    if (g_target_detected) {
                        p_cam << g_target_pos.x, g_target_pos.y, g_target_pos.z;
                        detected = true;
                    }
                }

                if (detected) {
                    auto cp = robot.GetCurrentPose();
                    float current_p[6] = {cp.X, cp.Y, cp.Z, cp.roll, cp.pitch, cp.yaw};
                    float obj_in_cam_arr[3] = {(float)p_cam.x(), (float)p_cam.y(), (float)p_cam.z()};
                    Eigen::Vector3d target_base = grasp_planner.getTargetInBase(current_p, obj_in_cam_arr);

                    std::cout << ">>> [Action] 识别成功，执行抓取序列" << std::endl;

                    // 抓取流程
                    robot.MoveL(target_base.x(), target_base.y(), target_base.z() + 0.08f, 3.14f, 0.0f, 1.57f);
                    if (!robot.WaitMoveDone(1500)) { 
                        // 如果超时了，可以在这里做异常处理，比如停止任务或报警
                        std::cerr << ">>> 运动失败，跳过后续抓取动作" << std::endl;
                        robot.SetEnable(false, damiao::MIT_MODE); // 紧急掉电或切换模式
                    } else {
                        std::cout << ">>> 已到达目标，执行夹爪动作" << std::endl;
                        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
                    }
                    
                    robot.hand->SetAngle(4.0f); 
                    std::this_thread::sleep_for(std::chrono::seconds(1));

                    robot.MoveL(target_base.x(), target_base.y(), target_base.z(), 3.14f, 0.0f, 1.57f);
                    if (!robot.WaitMoveDone(1500)) { 
                        // 如果超时了，可以在这里做异常处理，比如停止任务或报警
                        std::cerr << ">>> 运动失败，跳过后续抓取动作" << std::endl;
                        robot.SetEnable(false, damiao::MIT_MODE); // 紧急掉电或切换模式
                    } else {
                        std::cout << ">>> 已到达目标，执行夹爪动作" << std::endl;
                        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
                    }

                    robot.hand->GraspWithTorque(-0.4f, 0.0f, 0.8f);
                    std::this_thread::sleep_for(std::chrono::seconds(1));

                    robot.MoveL(target_base.x(), target_base.y(), target_base.z() + 0.1f, 3.14f, 0.0f, 1.57f);
                    if (!robot.WaitMoveDone(1500)) { 
                        // 如果超时了，可以在这里做异常处理，比如停止任务或报警
                        std::cerr << ">>> 运动失败，跳过后续抓取动作" << std::endl;
                        robot.SetEnable(false, damiao::MIT_MODE); // 紧急掉电或切换模式
                    } else {
                        std::cout << ">>> 已到达目标，执行夹爪动作" << std::endl;
                        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
                    }
                    
                    std::cout << ">>> 抓取循环完成，返回待机位..." << std::endl;
                    robot.Resting();
                    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
                    robot.hand->SetAngle(4.2f); 
                    std::this_thread::sleep_for(std::chrono::milliseconds(2000));
                    // robot.hand->SetAngle(0.0f); 

                    task_done = true; // 抓完一次，跳出当前的持续识别循环
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(50));
            }

            // 3. 抓取完成后，回到 Home 位置再次等待
            std::cout << ">>> [Done] 任务结束，返回 Home 位置..." << std::endl;
            robot.Homing();
            // 重置标志位
            g_nav_reached = false;
            { std::lock_guard<std::mutex> lock(g_vision_mtx); g_target_detected = false; }
        }
    } catch (...) {
        std::cerr << "运行异常" << std::endl;
    }

    g_thread_running = false;
    if (udp_thread.joinable()) udp_thread.join();
    if (vision_thread.joinable()) vision_thread.join();
    if (update_thread.joinable()) update_thread.join();
    robot.SetEnable(false, damiao::POS_VEL_MODE);
    return 0;
}