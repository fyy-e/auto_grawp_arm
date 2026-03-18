#include "dummy_robot.h"
#include <iostream>
#include <algorithm>
#include "ctrl_step.h"
#include <chrono>
DummyHand::DummyHand(SerialPort::SharedPtr serial, uint8_t _id) :
    nodeID(_id), serial_(serial), motorJ(nullptr)
{
    try {
        motorJ = new CtrlStepMotor(damiao::DM4310_48V, 0x07, 0x17);
    } catch (const std::exception& e) {
        std::cerr << "[DummyHand] 电机初始化失败: " << e.what() << std::endl;
        motorJ = nullptr;
    }
}

DummyHand::~DummyHand()
{
    if (motorJ != nullptr) {
        delete motorJ;
        motorJ = nullptr;
    }
}

// 3. 保护原有的纯位置控制（如开爪）不受影响
void DummyHand::SetAngle(float _angle_rad)
{
    if (motorJ == nullptr) {
        std::cerr << "[DummyHand::SetAngle] 错误：电机未初始化" << std::endl;
        return;
    }
    if (_angle_rad < minAngle) _angle_rad = minAngle;
    else if (_angle_rad > maxAngle) _angle_rad = maxAngle;  
    
    ResetImpedance(); // 每次使用位置控制前，确保切回高刚度
    motorJ->SetPositionWithVelocityLimit(_angle_rad, maxSpeed);
}

void DummyHand::SetmaxSpeed(float _val)
{
    // 参数验证
    if (_val <= 0) {
        std::cerr << "[DummyHand::SetmaxSpeed] 错误：速度值必须为正数，当前值: " << _val << std::endl;
        return;
    }
    if (_val > 1.0f) {
        std::cerr << "[DummyHand::SetmaxSpeed] 警告：速度过快 " << _val << " rad/s，已限制为1.0rad/s" << std::endl;
        _val = 1.0f;
    }
    
    maxSpeed = _val;
}

void DummyHand::SetEnable(bool _enable)
{
    if (motorJ == nullptr) {
        std::cerr << "[DummyHand::SetEnable] 错误：电机未初始化" << std::endl;
        return;
    }
    // [改动] 将默认的 POS_VEL_MODE 改为 MIT_MODE 以支持力矩控制
    motorJ->SetEnable(_enable, damiao::MIT_MODE);
    ResetImpedance(); // 确保开机时是位置环高刚度状态
    std::cerr << "[DummyHand::SetEnable] 成功：电机已上电(MIT模式)" << std::endl;
    isEnabled = _enable;
}
void DummyHand::ResetImpedance() {
    if (motorJ) {
        motorJ->mit_param.kp = 3.0f; // 恢复较高的位置刚度
        motorJ->mit_param.kd = 0.3f;
        motorJ->mit_param.tau = 0.0f;
    }
}
// 4. 新增的力矩抓取核心逻辑
bool DummyHand::GraspWithTorque(float target_torque, float min_angle, float close_speed) {
    if (motorJ == nullptr) return false;

    // 1. 设置阻抗参数 (参考你测试成功的 0.1, 0.2)
    float kp = 0.1f; 
    float kd = 0.2f;

    std::cout << "[DummyHand] 开始力矩抓取，目标位置: " << min_angle << " rad" << std::endl;

    // 2. 直接发送一次（或在小循环中发送）指向 min_angle 的指令
    // 注意：不再在循环里逐度减小目标，而是让电机自带的 PD 往 min_angle 走
    
    // 持续监控状态，直到抓到物体或到达位置
    int timeout_count = 0;
    while (timeout_count < 1000) { // 约 2 秒超时保护 (20ms * 100)
        
        // 发送 MIT 控制指令
        // 参数顺序：target_pos, target_vel, kp, kd, feedforward_torque
        motorJ->setPositionVelocityTorque(min_angle, 0.0f, target_torque, kp, kd);
        
        usleep(20000); // 20ms 周期，通讯更顺畅
        motorJ->UpdateAngle(); 

        float cur_vel = motorJ->velocity;
        float cur_tau = motorJ->tau;
        float cur_pos = motorJ->position;

        // 判定逻辑 A: 抓到了物体 (速度慢下来了，且反馈力矩上升)
        if (std::abs(cur_vel) < 0.05f && std::abs(cur_tau) >= std::abs(target_torque) * 0.8f) {
            std::cout << "[DummyHand] 捕获物体！反馈力矩: " << cur_tau << " Nm" << std::endl;
            // 抓到后切换高刚度锁死
            // motorJ->setPositionVelocityTorque(cur_pos, 0.0f, target_torque, 30.0f, 0.3f);
            return true;
        }

        // 判定逻辑 B: 到达了限位还没抓到
        if (std::abs(cur_pos - min_angle) < 0.05f) {
            break; 
        }

        timeout_count++;
    }

    std::cout << "[DummyHand] 未探测到物体，合拢至限位。" << std::endl;
    ResetImpedance();
    return false;
}
void DummyHand::CalibrateHomeOffset(){
    if (motorJ == nullptr) {
        std::cerr << "[DummyHand::CalibrateHomeOffset] 错误：电机未初始化" << std::endl;
        return;
    }
    motorJ->ApplyPositionAsHome();
}
DummyRobot::DummyRobot(std::string port_name, uint32_t baudrate, const std::string& urdf_path)
{
    // 初始化所有电机指针为 nullptr
    for (int i = 0; i < 7; i++) {
        motorJ[i] = nullptr;
    }
    
    this->serial_ = std::make_shared<SerialPort>(port_name, baudrate);
    
    // 初始化关节电机（限位值使用弧度）
    try {
        motorJ[1] = new CtrlStepMotor(damiao::DM4340_48V, 0x01, 0x11);
        motorJ[2] = new CtrlStepMotor(damiao::DM4340_48V, 0x02, 0x12);
        motorJ[3] = new CtrlStepMotor(damiao::DM4340_48V, 0x03, 0x13);
        motorJ[4] = new CtrlStepMotor(damiao::DM4310_48V, 0x04, 0x14);
        motorJ[5] = new CtrlStepMotor(damiao::DM4310_48V, 0x05, 0x15);
        motorJ[6] = new CtrlStepMotor(damiao::DM4310_48V, 0x06, 0x16);
        std::cout << "[DummyRobot] 所有关节电机初始化成功" << std::endl;
    } catch (const std::exception& e) {
        std::cerr << "[DummyRobot] 电机初始化失败: " << e.what() << std::endl;
    }
    
    hand = new DummyHand(serial_, 7);

    // 初始化Pinocchio运动学
    try {
        kinematics = new DmKinematics(urdf_path);
        dof = kinematics->getJointNum();
        std::cout << "[DummyRobot] Pinocchio 初始化成功，DOF=" << dof << std::endl;
    } catch (const std::exception& e) {
        std::cerr << "[DummyRobot] 运动学初始化失败: " << e.what() << std::endl;
        kinematics = nullptr;
        dof = 6;
    }

    currentJoints = INIT_POSE;
    targetJoints = INIT_POSE;
    initPose = INIT_POSE;
}

DummyRobot::~DummyRobot()
{
    // 删除关节电机（注意：motorJ[0] 未初始化，跳过）
    for (int j = 1; j <= 6; j++) {
        if (motorJ[j] != nullptr) {
            delete motorJ[j];
            motorJ[j] = nullptr;
        }
    }
    if (hand != nullptr) {
        delete hand;
        hand = nullptr;
    }
    if (kinematics != nullptr) {
        delete kinematics;
        kinematics = nullptr;
    }
}

void DummyRobot::Init()
{
    SetCommandMode(DEFAULT_COMMAND_MODE);
    SetJointSpeed(DEFAULT_JOINT_SPEED);
}

float DummyRobot::AbsMaxOf6(const Joint6D_t& joints, uint8_t& index) const
{
    float max_val = -1.0f;
    for (uint8_t i = 0; i < 6; i++) {
        if (std::abs(joints.j[i]) > max_val) {
            max_val = std::abs(joints.j[i]);
            index = i;
        }
    }
    return max_val;
}
//     if (_val > limit_param.Q_MAX) _val = limit_param.Q_MAX;
//     if (_val < -limit_param.Q_MAX) _val = -limit_param.Q_MAX;
bool DummyRobot::MoveJ(float j1_rad, float j2_rad, float j3_rad, 
                       float j4_rad, float j5_rad, float j6_rad)
{
    Joint6D_t target(j1_rad, j2_rad, j3_rad, j4_rad, j5_rad, j6_rad);
    bool valid = true;

    // 检查关节限位（弧度）
    for (int j = 1; j <= 6; j++){
        if (target.j[j-1] > motorJ[j]->limit_param.Q_MAX || 
            target.j[j-1] < -motorJ[j]->limit_param.Q_MAX) {
            valid = false;
            std::cerr << "[MoveJ] 关节 " << j << " 超出限位: " 
                      << target.j[j-1] << " rad" << std::endl;
        }
    }

    if (valid) {
        Joint6D_t delta;
        for (int i = 0; i < 6; i++) {
            delta.j[i] = target.j[i] - currentJoints.j[i];
        }
        
        uint8_t max_idx;
        float max_delta = AbsMaxOf6(delta, max_idx);
        
        float time = (max_delta > 0.001f) ? (max_delta / jointSpeed) : 0.1f;
        
        Joint6D_t dynamicSpeeds;
        for (int j = 1; j <= 6; j++) {
            dynamicSpeeds.j[j-1] = std::abs(delta.j[j-1]) / time;
        }

        jointsStateFlag = 0;
        targetJoints = target;
        dynamicJointSpeeds = dynamicSpeeds;
        MoveJointsWithSpeed(target, dynamicSpeeds);
        return true;
    }
    return false;
}

void DummyRobot::MoveJoints(const Joint6D_t& joints_rad)
{
    for (int j = 1; j <= 6; j++) {
        // 直接传递弧度值（减去初始偏移）
        float target_rad = joints_rad.j[j-1] - initPose.j[j-1];
        float speed_rad_per_sec = jointSpeed * jointSpeedRatio;
        
        motorJ[j]->SetAngleWithVelocityLimit(target_rad, speed_rad_per_sec);
        motorJ[j]->target_pos = target_rad;
    }
}

void DummyRobot::MoveJointsWithSpeed(const Joint6D_t& joints_rad, 
                                     const Joint6D_t& speeds_rad_per_sec)
{
    for (int j = 1; j <= 6; j++) {
        float target_rad = joints_rad.j[j-1] - initPose.j[j-1];
        motorJ[j]->SetAngleWithVelocityLimit(target_rad, speeds_rad_per_sec.j[j-1]);
    }
}

bool DummyRobot::MoveL(float x_m, float y_m, float z_m, 
                       float roll_rad, float pitch_rad, float yaw_rad)
{
    if (!kinematics || dof < 6) {   // 6轴臂必须满自由度
        std::cerr << "[MoveL] 需要6自由度运动学模型" << std::endl;
        return false;
    }

    Eigen::Vector3d target_pos(x_m, y_m, z_m);
    Eigen::Vector3d target_rpy(roll_rad, pitch_rad, yaw_rad);
    
    Eigen::VectorXd q_init(dof), q_out(dof);
    for (int i = 0; i < dof; i++) {
        q_init[i] = currentJoints.j[i];
    }
    
    // 关键修改：启用姿态约束，且姿态权重不宜过大（推荐0.5~1.0）
    bool success = kinematics->inverse(target_pos, target_rpy, q_init, q_out,
                                       true, 1.0, 0.8);
    
    if (!success) {
        std::cerr << "[MoveL] IK 求解失败" << std::endl;
        return false;
    }

    // 额外检查：逆解结果是否在电机限位内
    for (int j = 1; j <= 6; j++) {
        if (q_out[j-1] > motorJ[j]->limit_param.Q_MAX || 
            q_out[j-1] < -motorJ[j]->limit_param.Q_MAX) {
            std::cerr << "[MoveL] 逆解结果超出关节" << j << "限位" << std::endl;
            return false;
        }
    }

    // 直接调用关节运动（MoveJ内部会检查限位，但我们已经提前检查了）
    return MoveJ(q_out[0], q_out[1], q_out[2], q_out[3], q_out[4], q_out[5]);
}

void DummyRobot::UpdateJointAngles()
{
    for(int i = 1; i <= 6; i++){
        motorJ[i]->UpdateAngle();
    }
}

void DummyRobot::UpdateJointAnglesCallback()
{
    for (int i = 1; i <= 6; i++) {
        // 直接读取弧度值（假设电机返回弧度）
        currentJoints.j[i-1] = motorJ[i]->position + initPose.j[i-1];
        
        if (motorJ[i]->state == CtrlStepMotor::FINISH)
            jointsStateFlag |= (1 << i);
        else
            jointsStateFlag &= ~(1 << i);
    }
}

void DummyRobot::UpdateJointPose6D()
{
    if (!kinematics || dof == 0) return;

    Eigen::VectorXd q(dof);
    for (int i = 0; i < dof; i++) {
        q[i] = currentJoints.j[i];
    }
    
    Eigen::VectorXd pose = kinematics->forward(q);
    
    currentPose.X = pose[0];
    currentPose.Y = pose[1];
    currentPose.Z = pose[2];
    currentPose.roll = pose[3];
    currentPose.pitch = pose[4];
    currentPose.yaw = pose[5];
}

void DummyRobot::SetJointSpeed(float speed_rad_per_sec)
{
    if (speed_rad_per_sec < 0) speed_rad_per_sec = 0;
    else if (speed_rad_per_sec > 2.0f) speed_rad_per_sec = 2.0f;
    
    jointSpeed = speed_rad_per_sec;
}

void DummyRobot::SetJointAcceleration(float acc_rad_per_sec2)
{
    if (acc_rad_per_sec2 < 0) acc_rad_per_sec2 = 0;
    
    for (int i = 1; i <= 6; i++) {
        float acc_base = DEFAULT_JOINT_ACCELERATION_BASES.j[i-1];
        float actual_acc = (acc_rad_per_sec2 / DEFAULT_JOINT_ACCELERATION_HIGH) * acc_base;
        motorJ[i]->SetAcceleration(actual_acc);  // 直接传递弧度
    }
}

void DummyRobot::CalibrateHomeOffset()
{
    for(int i = 1; i <= 6; i++){
        motorJ[i]->ApplyPositionAsHome();
        osDelay(100);
    }
    osDelay(500);
}

void DummyRobot::Homing()
{
    float lastSpeed = jointSpeed;
    SetJointSpeed(0.1745f);  // 0.1 rad/s ≈ 10°/s
    
    MoveJ(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);  // π/4 弧度
    MoveJoints(targetJoints);
    if (!WaitMoveDone(1500)) { 
        std::cerr << ">>> 运动失败" << std::endl;
        SetEnable(false, damiao::POS_VEL_MODE); // 紧急掉电或切换模式
    } else {
        std::cout << ">>> 已到达目标" << std::endl;
    }
    SetJointSpeed(lastSpeed);
}

void DummyRobot::Resting()
{
    float lastSpeed = jointSpeed;
    SetJointSpeed(0.1745f);
    
    MoveJ(REST_POSE.j[0], REST_POSE.j[1], REST_POSE.j[2],
          REST_POSE.j[3], REST_POSE.j[4], REST_POSE.j[5]);
    MoveJoints(targetJoints);
    if (!WaitMoveDone(1500)) { 
        std::cerr << ">>> 运动失败" << std::endl;
        SetEnable(false, damiao::POS_VEL_MODE); // 紧急掉电或切换模式
    } else {
        std::cout << ">>> 已到达目标" << std::endl;
    }
    
    SetJointSpeed(lastSpeed);
}

void DummyRobot::SetEnable(bool _enable,damiao::Control_Mode mode)
{
    for(int i = 1; i <= 6; i++){
        motorJ[i]->SetEnable(_enable,mode);
    }
    isEnabled = _enable;
}

bool DummyRobot::IsMoving()
{
    for (int i = 1; i <= 6; i++) {
        if (motorJ[i]->state == CtrlStepMotor::RUNNING) return true;
    }
    return false;
}

bool DummyRobot::IsEnabled()
{
    return isEnabled;
}

void DummyRobot::GetOffsets()
{

}

void DummyRobot::SetCommandMode(uint32_t _mode)
{
    if (_mode < COMMAND_TARGET_POINT_SEQUENTIAL ||
        _mode > COMMAND_CONTINUES_TRAJECTORY)
        return;

    commandMode = static_cast<CommandMode>(_mode);

    switch (commandMode)
    {
        case COMMAND_TARGET_POINT_SEQUENTIAL:
        case COMMAND_TARGET_POINT_INTERRUPTABLE:
            jointSpeedRatio = 1.0f;
            SetJointAcceleration(DEFAULT_JOINT_ACCELERATION_LOW);
            break;
        case COMMAND_CONTINUES_TRAJECTORY:
            SetJointAcceleration(DEFAULT_JOINT_ACCELERATION_HIGH);
            jointSpeedRatio = 0.3f;
            break;
    }
}
float DummyRobot::GetMaxJointError() {
    float max_err = 0.0f;
    for (int i = 1; i <= 6; i++) {
        float err = std::abs(targetJoints.j[i-1] - currentJoints.j[i-1]);
        if (err > max_err) max_err = err;
    }
    return max_err;
}

bool DummyRobot::WaitMoveDone(int extra_timeout_ms) {
    // 1. 计算动态超时时间 (毫秒)
    // 假设 jointSpeed 是 rad/s，需要防止除以 0
    float speed = (jointSpeed > 0.01f) ? jointSpeed : 0.1f; 
    float dist = GetMaxJointError();
    
    // 计算预计时间，并放大 2 倍余量，再加上固定补偿
    int dynamic_timeout = static_cast<int>((dist / speed) * 2000.0f) + extra_timeout_ms;
    
    // 最小超时保证（比如至少给 500ms）
    if (dynamic_timeout < 500) dynamic_timeout = 500;

    auto start_time = std::chrono::steady_clock::now();

    // 2. 循环监测
    while (IsMoving()) {
        auto now = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - start_time).count();

        if (elapsed > dynamic_timeout) {
            std::cerr << ">>> [Error] Robot move timeout! Expected: " << dynamic_timeout 
                      << "ms, Elapsed: " << elapsed << "ms" << std::endl;
            return false; // 超时退出
        }
        
        std::this_thread::sleep_for(std::chrono::milliseconds(20)); // 降低 CPU 占用
    }

    return true; // 正常到达
}