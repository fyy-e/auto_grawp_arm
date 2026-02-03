#include "dummy_robot.h"
#include <iostream>
#include <algorithm>

DummyHand::DummyHand(SocketCan* _hcan, uint8_t _id) :
    nodeID(_id), hcan(_hcan)
{
}

void DummyHand::SetAngle(float _angle_rad)
{
    // 机械手角度控制（弧度）
}

void DummyHand::SetMaxCurrent(float _val)
{
}

void DummyHand::SetEnable(bool _enable)
{
}

DummyRobot::DummyRobot(SocketCan* _hcan, const std::string& urdf_path) :
    hcan(_hcan)
{
    // 初始化关节电机（限位值使用弧度）
    motorJ[ALL] = new CtrlStepMotor(_hcan, 0, false, 15, -M_PI, M_PI);
    motorJ[1] = new CtrlStepMotor(_hcan, 1, false, 15, -M_PI, M_PI);
    motorJ[2] = new CtrlStepMotor(_hcan, 2, true, 15, -2, 2);
    motorJ[3] = new CtrlStepMotor(_hcan, 3, false, 15, -M_PI, M_PI);
    motorJ[4] = new CtrlStepMotor(_hcan, 4, true, 15, -2, 2);
    motorJ[5] = new CtrlStepMotor(_hcan, 5, false, 15, -M_PI, M_PI);
    motorJ[6] = new CtrlStepMotor(_hcan, 6, false, 15, -M_PI, M_PI);
    hand = new DummyHand(_hcan, 7);

    // 初始化Pinocchio运动学
    try {
        kinematics = new BerkeleyKinematics(urdf_path);
        dof = kinematics->getJointNum();
        std::cout << "[DummyRobot] Pinocchio 初始化成功，DOF=" << dof << std::endl;
    } catch (const std::exception& e) {
        std::cerr << "[DummyRobot] 运动学初始化失败: " << e.what() << std::endl;
        kinematics = nullptr;
        dof = 5;
    }

    currentJoints = REST_POSE;
    targetJoints = REST_POSE;
    initPose = REST_POSE;
}

DummyRobot::~DummyRobot()
{
    for (int j = 0; j <= 6; j++)
        delete motorJ[j];
    delete hand;
    delete kinematics;
}

void DummyRobot::Init()
{
    SetCommandMode(DEFAULT_COMMAND_MODE);
    SetJointSpeed(DEFAULT_JOINT_SPEED);
}

void DummyRobot::Reboot()
{
    for(int i = 1; i <= 6; i++){
        motorJ[i]->Reboot();
    }
    osDelay(500);
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

bool DummyRobot::MoveJ(float j1_rad, float j2_rad, float j3_rad, 
                       float j4_rad, float j5_rad, float j6_rad)
{
    Joint6D_t target(j1_rad, j2_rad, j3_rad, j4_rad, j5_rad, j6_rad);
    bool valid = true;

    // 检查关节限位（弧度）
    for (int j = 1; j <= 6; j++) {
        if (target.j[j-1] > motorJ[j]->angleLimitMax || 
            target.j[j-1] < motorJ[j]->angleLimitMin) {
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
    if (!kinematics) {
        std::cerr << "[MoveL] 运动学未初始化" << std::endl;
        return false;
    }

    Eigen::Vector3d target_pos(x_m, y_m, z_m);
    Eigen::Vector3d target_rpy(roll_rad, pitch_rad, yaw_rad);
    
    Eigen::VectorXd q_init(dof), q_out(dof);
    for (int i = 0; i < dof; i++) {
        q_init[i] = currentJoints.j[i];
    }
    
    bool success = kinematics->inverse(target_pos, target_rpy, q_init, q_out);
    
    if (!success) {
        std::cerr << "[MoveL] IK 求解失败" << std::endl;
        return false;
    }
    
    float j1 = q_out[0];
    float j2 = (dof > 1) ? q_out[1] : 0.0f;
    float j3 = (dof > 2) ? q_out[2] : 0.0f;
    float j4 = (dof > 3) ? q_out[3] : 0.0f;
    float j5 = (dof > 4) ? q_out[4] : 0.0f;
    float j6 = (dof > 5) ? q_out[5] : yaw_rad;
    
    return MoveJ(j1, j2, j3, j4, j5, j6);
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
        currentJoints.j[i-1] = motorJ[i]->angle + initPose.j[i-1];
        
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
    for(int i = 1; i <= 6; i++){
        motorJ[i]->save_settings_to_flash();
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
    while (IsMoving())
        osDelay(10);
    SetJointSpeed(lastSpeed);
}

void DummyRobot::Resting()
{
    float lastSpeed = jointSpeed;
    SetJointSpeed(0.1745f);
    
    MoveJ(REST_POSE.j[0], REST_POSE.j[1], REST_POSE.j[2],
          REST_POSE.j[3], REST_POSE.j[4], REST_POSE.j[5]);
    MoveJoints(targetJoints);
    while (IsMoving())
        osDelay(10);
    
    SetJointSpeed(lastSpeed);
}

void DummyRobot::SetEnable(bool _enable)
{
    for(int i = 1; i <= 6; i++){
        motorJ[i]->SetEnable(_enable);
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
    for(int i = 1; i <= 6; i++){
        float offset = motorJ[i]->get_offset();  // 假设返回弧度
        std::cout << i << "：offset: " << offset << " rad" << std::endl;
    }
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