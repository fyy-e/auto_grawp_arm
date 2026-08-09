// 任务主入口 - 机械臂视觉抓取主控程序（GRCNN 融合版）
// 基于 dm_arm_end(V5.0) 改造，视觉后端由 YOLO 换成 GR-ConvNet
// 改造说明见 docs/GRCNN改造说明.md
#include <iostream>
#include <thread>
#include <chrono>
#include <atomic>
#include <mutex>
#include <cmath>
#include <Eigen/Dense>
#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>
#include <cstring>

#include "dummy_robot.h"
#include "u2can/SerialPort.h"
#include "algorithms/VisionGraspPlanner.h"
#include "algorithms/VisionDetector.h"

// ================= 模式开关 =================
// 1: 等待狗的导航成功命令
// 0: 不等待，机械臂一直停在 Resting 位等待识别
#define WAIT_DOG_NAV 0

// yaw 固定补偿量：如果发现方向整体差 90° / 180°，改这里
#define YAW_OFFSET 1.57

// ================= 全局变量与并发控制 =================
std::atomic<bool> g_thread_running(true);
std::shared_ptr<SerialPort> CtrlStepMotor::serial = std::make_shared<SerialPort>("/dev/ttyACM0", B921600);
damiao::Motor_Control CtrlStepMotor::dm(CtrlStepMotor::serial);
std::mutex g_serial_mtx;

// GRCNN 抓取目标：位置 + 抓取角 + 建议开口
struct SafeTarget
{
    double x, y, z;
    double angle; // 图像平面抓取角 (rad)
    double width; // 建议开口 (m)，<=0 无效
};
std::mutex g_vision_mtx;
SafeTarget g_target = {0.0, 0.0, 0.0, 0.0, -1.0};
std::atomic<bool> g_target_detected(false);
std::atomic<bool> g_nav_reached(false); // UDP 触发标志

// ================= 抓取参数 =================
namespace grasp_cfg
{
    const double PRE_GRASP_H = 0.10; // 预抓取高度 (m)，与原代码一致
    const double LIFT_H = 0.10;      // 抓后抬起高度 (m)
    const double REACH_MAX = 0.55;   // 允许的最大水平可达半径 (m)，超出放弃
    const double REACH_MIN = 0.10;   // 过近保护
}

// ================= 工具函数 =================
double NormalizeAngle(double angle)
{
    while (angle > M_PI)
        angle -= 2.0 * M_PI;
    while (angle < -M_PI)
        angle += 2.0 * M_PI;
    return angle;
}

// ================= UDP 监听线程 =================
void UdpListenerThread(int port)
{
    int sockfd;
    struct sockaddr_in servaddr, cliaddr;
    if ((sockfd = socket(AF_INET, SOCK_DGRAM, 0)) < 0)
        return;

    struct timeval tv;
    tv.tv_sec = 1;
    tv.tv_usec = 0;
    setsockopt(sockfd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

    memset(&servaddr, 0, sizeof(servaddr));
    servaddr.sin_family = AF_INET;
    servaddr.sin_addr.s_addr = INADDR_ANY;
    servaddr.sin_port = htons(port);

    if (bind(sockfd, (const struct sockaddr *)&servaddr, sizeof(servaddr)) < 0)
    {
        close(sockfd);
        return;
    }

    char buffer[1024];
    socklen_t len = sizeof(cliaddr);
    while (g_thread_running)
    {
        int n = recvfrom(sockfd, (char *)buffer, 1024, 0, (struct sockaddr *)&cliaddr, &len);
        if (n > 0)
        {
            buffer[n] = '\0';
            if (std::string(buffer) == "NAV_REACHED_SUCCESS")
            {
                std::cout << "\033[32m[UDP] 导航到达，准备开始识别抓取！\033[0m" << std::endl;
                g_nav_reached = true;
            }
        }
    }
    close(sockfd);
}

// ================= 视觉处理线程（EMA 滤波，含角度与宽度） =================
void VisionThread(VisionDetector *detector)
{
    VisionTarget t;
    const double alpha = 0.4;
    while (g_thread_running)
    {
        if (detector->GetTargetInCam(t))
        {
            std::lock_guard<std::mutex> lock(g_vision_mtx);
            if (!g_target_detected)
            {
                g_target = {t.x, t.y, t.z, t.angle, t.width};
            }
            else
            {
                g_target.x = alpha * t.x + (1.0 - alpha) * g_target.x;
                g_target.y = alpha * t.y + (1.0 - alpha) * g_target.y;
                g_target.z = alpha * t.z + (1.0 - alpha) * g_target.z;
                g_target.angle = alpha * t.angle + (1.0 - alpha) * g_target.angle;
                if (t.width > 0)
                    g_target.width = alpha * t.width + (1.0 - alpha) * g_target.width;
            }
            g_target_detected = true;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
}

// ================= 状态更新线程 =================
void UpdateThread(DummyRobot *robot, int update_rate_hz)
{
    int update_period_ms = 1000 / update_rate_hz;
    while (g_thread_running)
    {
        std::lock_guard<std::mutex> lock(g_serial_mtx);
        robot->UpdateJointAngles();
        robot->UpdateJointAnglesCallback();
        if (robot->GetDof() > 0)
            robot->UpdateJointPose6D();
        std::this_thread::sleep_for(std::chrono::milliseconds(update_period_ms));
    }
}

/**
 * @brief 打印当前关节角度（弧度）
 */
void PrintJointAngles(const DummyRobot::Joint6D_t &joints)
{
    std::cout << "当前关节角度: ";
    for (int i = 0; i < 6; i++)
    {
        std::cout << "J" << i + 1 << "=" << joints.j[i] << "rad ";
    }
    std::cout << std::endl;
}

/**
 * @brief 打印末端位姿（米和弧度）
 */
void PrintPose(const DummyRobot::Pose6D_t &pose)
{
    std::cout << "末端位姿: X=" << pose.X << "m "
              << "Y=" << pose.Y << "m "
              << "Z=" << pose.Z << "m "
              << "Roll=" << pose.roll << "rad "
              << "Pitch=" << pose.pitch << "rad "
              << "Yaw=" << pose.yaw << "rad"
              << std::endl;
}

// ================= 带检查的运动封装（GRCNN 改造新增） =================
// 原代码不检查 MoveL 返回值，IK 失败会静默通过导致在原地闭合夹爪
bool moveLChecked(DummyRobot *robot, double x, double y, double z,
                  double roll, double pitch, double yaw, const char *stage)
{
    if (!robot->MoveL(x, y, z, roll, pitch, yaw))
    {
        std::cerr << "[MoveL] IK 求解失败 @ " << stage
                  << "，尝试备用姿态..." << std::endl;
        return false;
    }
    if (!robot->WaitMoveDone(1500))
    {
        std::cerr << "[MoveL] 运动超时 @ " << stage << std::endl;
        robot->SetEnable(false, damiao::MIT_MODE);
        // robot->Resting();
        return false;
    }
    return true;
}

// IK 降级重试：yaw +90° -> yaw -90° -> 抬高 3cm
// 大偏差目标在固定腕部姿态下常 IK 失败，旋转腕部 90° 后通常可达
bool reachWithFallback(DummyRobot *robot, double x, double y, double z, double yaw)
{
    const double R = 3.14, P = 0.0;
    if (moveLChecked(robot, x, y, z, R, P, yaw, "primary"))
        return true;
    // if (moveLChecked(robot, x, y, z, R, P, NormalizeAngle(yaw + 1.57), "yaw+90"))
    return true;
    // if (moveLChecked(robot, x, y, z, R, P, NormalizeAngle(yaw - 1.57), "yaw-90"))
    return true;
    if (moveLChecked(robot, x, y, z + 0.03, R, P, yaw, "lift3cm"))
        return true;
    return false;
}

// ================= 主逻辑 =================
int main()
{
    std::string urdf_path = "/home/ysh/dm_arm_end/src/urdf/urdf/DM_urdf.urdf";
    std::string calib_path = "/home/ysh/dm_arm_end/src/config/handeye_result_realsense.yaml";

    DummyRobot robot("/dev/ttyACM0", B921600, urdf_path);
    VisionGraspPlanner grasp_planner(calib_path);
    VisionDetector detector(0.05f);
    // robot.CalibrateHomeOffset();
    robot.hand->CalibrateHomeOffset();

    if (!detector.Init())
        return -1;

    robot.hand->SetEnable(true);
    robot.Init();
    robot.SetEnable(true, damiao::POS_VEL_MODE);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    robot.hand->SetAngle(0.0f);

    std::thread update_thread(UpdateThread, &robot, 100);
    std::thread vision_thread(VisionThread, &detector);

#if WAIT_DOG_NAV
    std::thread udp_thread(UdpListenerThread, 12346);
#endif

    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    robot.Homing();
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

#if !WAIT_DOG_NAV
    std::cout << ">>> [Init] 不等待导航，先移动到 Resting 位持续等待识别..." << std::endl;
    robot.Resting();
#endif

    try
    {
        while (g_thread_running)
        {

#if WAIT_DOG_NAV
            std::cout << ">>> [Idle] 等待狗的导航信号..." << std::endl;
            while (!g_nav_reached && g_thread_running)
            {
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }
            if (!g_thread_running)
                break;

            std::cout << ">>> [Task] 移动到 Resting 位置，准备识别..." << std::endl;
            robot.Resting();
#endif

            std::cout << ">>> [Task] 开始识别..." << std::endl;

            bool task_done = false;
            while (!task_done && g_thread_running)
            {
                SafeTarget tgt;
                bool detected = false;

                {
                    std::lock_guard<std::mutex> lock(g_vision_mtx);
                    if (g_target_detected)
                    {
                        tgt = g_target;
                        detected = true;
                    }
                }

                if (detected)
                {
                    auto cp = robot.GetCurrentPose();
                    float current_p[6] = {cp.X, cp.Y, cp.Z, cp.roll, cp.pitch, cp.yaw};
                    float obj_in_cam_arr[3] = {(float)tgt.x, (float)tgt.y, (float)tgt.z};
                    Eigen::Vector3d target_base = grasp_planner.getTargetInBase(current_p, obj_in_cam_arr);
                    if (tgt.width > 0)
                        target_base.z() -= tgt.width * 0.5;
                    if (target_base.z() <= 0)
                        target_base.z() = 0;
                    std::cout << "    tgt.width  = " << tgt.width << std::endl;
                    // 可达性检查（GRCNN 改造新增）
                    double reach = std::hypot(target_base.x(), target_base.y());
                    if (reach > grasp_cfg::REACH_MAX || reach < grasp_cfg::REACH_MIN)
                    {
                        std::cerr << ">>> 目标超出可达范围 r=" << reach << " m，跳过" << std::endl;
                        std::this_thread::sleep_for(std::chrono::milliseconds(200));
                        {
                            std::lock_guard<std::mutex> lock(g_vision_mtx);
                            g_target_detected = false;
                        }
                        continue;
                    }

                    // ===== 抓取 yaw：目标方位角 + GRCNN 物体朝向角 =====
                    // atan2 项：让手臂朝向目标方位（沿用原方案）
                    // angle 项：让夹爪对齐物体长轴（GRCNN 新增，符号/零位需标定）
                    double yaw = std::atan2(target_base.y(), target_base.x()) + YAW_OFFSET + grasp_planner.angle_sign * tgt.angle + grasp_planner.angle_offset;
                    yaw = NormalizeAngle(yaw);

                    // ===== 夹爪开口：GRCNN 宽度自适应（无宽度时用原固定值）=====
                    float open_angle = (tgt.width > 0)
                                           ? (float)grasp_planner.widthToGripperAngle(tgt.width * 1.3)
                                           : 4.0f;
                    // 力矩抓取的最小闭合角：按开口的 50% 兜底，防止空夹到底/夹碎
                    float min_close = open_angle * 0.5f;

                    std::cout << ">>> [Action] 识别成功，执行抓取序列" << std::endl;
                    std::cout << "    target_base = [" << target_base.x() << ", "
                              << target_base.y() << ", " << target_base.z() << "]" << std::endl;
                    std::cout << "    grasp_yaw   = " << yaw << " rad (angle="
                              << tgt.angle << ", width=" << tgt.width << " m)" << std::endl;
                    std::cout << "    open_angle  = " << open_angle << std::endl;

                    // 1. 先到目标上方（IK 失败自动降级重试）
                    if (!reachWithFallback(&robot, target_base.x(), target_base.y(),
                                           target_base.z() + grasp_cfg::PRE_GRASP_H, yaw))
                    {
                        std::cerr << ">>> 预抓取位不可达，放弃本次抓取" << std::endl;
                        // robot.SetEnable(false, damiao::MIT_MODE);
                        robot.Resting();
                        break;
                    }
                    std::cout << ">>> 已到达目标上方" << std::endl;
                    std::this_thread::sleep_for(std::chrono::milliseconds(500));

                    // 2. 张开夹爪（开口随物体宽度自适应）
                    robot.hand->SetAngle(open_angle);
                    std::this_thread::sleep_for(std::chrono::seconds(1));

                    // 3. 下压到抓取点（失败则先抬回预抓取位再放弃）
                    if (!reachWithFallback(&robot, target_base.x(), target_base.y(),
                                           target_base.z(), yaw))
                    {
                        std::cerr << ">>> 抓取点不可达，抬起放弃" << std::endl;
                        moveLChecked(&robot, target_base.x(), target_base.y(),
                                     target_base.z() + grasp_cfg::PRE_GRASP_H,
                                     3.14, 0.0, yaw, "retreat");
                        // robot.SetEnable(false, damiao::MIT_MODE);
                        robot.Resting();
                        break;
                    }
                    std::cout << ">>> 已到达抓取点" << std::endl;
                    std::this_thread::sleep_for(std::chrono::milliseconds(300));

                    // 4. 闭合夹爪（力矩控制，min_close 随开口联动）
                    robot.hand->GraspWithTorque(-0.4f, min_close, 0.8f);
                    std::this_thread::sleep_for(std::chrono::seconds(1));

                    // 5. 抬起
                    if (!moveLChecked(&robot, target_base.x(), target_base.y(),
                                      target_base.z() + grasp_cfg::LIFT_H,
                                      3.14, 0.0, yaw, "lift"))
                    {
                        std::cerr << ">>> 抬起运动失败" << std::endl;
                        // robot.SetEnable(false, damiao::MIT_MODE);
                        robot.Resting();
                        break;
                    }
                    std::cout << ">>> 已抬起目标" << std::endl;
                    std::this_thread::sleep_for(std::chrono::milliseconds(300));

                    std::cout << ">>> 抓取循环完成" << std::endl;

#if WAIT_DOG_NAV
                    std::cout << ">>> 返回待机位..." << std::endl;
                    robot.Resting();
                    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
                    robot.hand->SetAngle(4.0f);
                    std::this_thread::sleep_for(std::chrono::milliseconds(2000));

                    task_done = true;
#else
                    // 不等待导航模式：
                    // 抓完后回到 Resting 位，然后继续在 Resting 位等待下一次识别
                    std::cout << ">>> 返回 Resting 位，继续等待识别..." << std::endl;
                    robot.Resting();
                    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
                    robot.hand->SetAngle(4.0f);
                    std::this_thread::sleep_for(std::chrono::milliseconds(5000));

                    {
                        std::lock_guard<std::mutex> lock(g_vision_mtx);
                        g_target_detected = false;
                    }
#endif
                }

                std::this_thread::sleep_for(std::chrono::milliseconds(50));
            }

#if WAIT_DOG_NAV
            std::cout << ">>> [Done] 任务结束，返回 Home 位置..." << std::endl;
            robot.Homing();
            g_nav_reached = false;
            {
                std::lock_guard<std::mutex> lock(g_vision_mtx);
                g_target_detected = false;
            }
#endif
        }
    }
    catch (...)
    {
        std::cerr << "运行异常" << std::endl;
    }

    g_thread_running = false;

#if WAIT_DOG_NAV
    if (udp_thread.joinable())
        udp_thread.join();
#endif
    if (vision_thread.joinable())
        vision_thread.join();
    if (update_thread.joinable())
        update_thread.join();

    robot.SetEnable(false, damiao::POS_VEL_MODE);
    return 0;
}
