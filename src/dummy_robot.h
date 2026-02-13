#ifndef REF_STM32F4_FW_DUMMY_ROBOT_H
#define REF_STM32F4_FW_DUMMY_ROBOT_H

#include "ctrl_step/ctrl_step.h"
#include "DmKinematics.h"
#include <string>
#include <cmath>
#include "src/u2can/SerialPort.h"
#include "src/u2can/damiao.h"
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define ALL 0

class DummyHand
{
public:
    uint8_t nodeID = 7;
    float maxCurrent = 0.7;

    DummyHand(SerialPort::SharedPtr serial = nullptr, uint8_t _id = 0);
    void SetAngle(float _angle_rad);  // 弧度
    void SetMaxCurrent(float _val);
    void SetEnable(bool _enable);
private:
    SerialPort::SharedPtr serial_;
    float minAngle = 0;    // 弧度
    float maxAngle = 0.785398f;  // 45度 = π/4 弧度
};

class DummyRobot
{
public:
    struct Joint6D_t {
        float j[6] = {0};  // 弧度 (rad)
        
        Joint6D_t() = default;
        Joint6D_t(float j1, float j2, float j3, float j4, float j5, float j6) {
            j[0]=j1; j[1]=j2; j[2]=j3; j[3]=j4; j[4]=j5; j[5]=j6;
        }
        
        Joint6D_t operator-(const Joint6D_t& other) const {
            Joint6D_t res;
            for(int i=0; i<6; i++) res.j[i] = j[i] - other.j[i];
            return res;
        }
        
        Joint6D_t operator+(const Joint6D_t& other) const {
            Joint6D_t res;
            for(int i=0; i<6; i++) res.j[i] = j[i] + other.j[i];
            return res;
        }
    };

    struct Pose6D_t {
        float X = 0, Y = 0, Z = 0;           // 米 (m)
        float roll = 0, pitch = 0, yaw = 0;  // 弧度 (rad)
        
        Pose6D_t() = default;
        Pose6D_t(float x, float y, float z, float r, float p, float yw)
            : X(x), Y(y), Z(z), roll(r), pitch(p), yaw(yw) {}
    };

    enum CommandMode
    {
        COMMAND_TARGET_POINT_SEQUENTIAL = 1,
        COMMAND_TARGET_POINT_INTERRUPTABLE,
        COMMAND_CONTINUES_TRAJECTORY,
    };

    // 默认参数（弧度制）
    const Joint6D_t INIT_POSE = {0.0f, 0.0, 0.0, 0.0f, 0.0f, 0.0f};  // 90° = π/2
    const Joint6D_t REST_POSE = {0.0f, 0.785f, 0.785f, 0.0f, 0.0f, 0.0f};  // 90° = π/2
    
    // 速度：rad/s
    const float DEFAULT_JOINT_SPEED = 10.0f;  // 30°/s
    
    // 加速度：rad/s²
    const Joint6D_t DEFAULT_JOINT_ACCELERATION_BASES = {2.618f, 1.745f, 3.490f, 3.490f, 3.490f, 3.490f};
    const float DEFAULT_JOINT_ACCELERATION_LOW = 0.524f;   // 30°/s²
    const float DEFAULT_JOINT_ACCELERATION_HIGH = 1.745f;  // 100°/s²
    const CommandMode DEFAULT_COMMAND_MODE = COMMAND_TARGET_POINT_INTERRUPTABLE;

    explicit DummyRobot(std::string port_name, uint32_t baudrate, const std::string& urdf_path);
    ~DummyRobot();

    void Init();
    void Reboot();
    
    // 所有角度参数均为弧度，所有位置参数均为米
    bool MoveJ(float j1_rad, float j2_rad, float j3_rad, 
               float j4_rad, float j5_rad, float j6_rad);
    
    bool MoveL(float x_m, float y_m, float z_m, 
               float roll_rad, float pitch_rad, float yaw_rad);
    
    void MoveJoints(const Joint6D_t& joints_rad);
    void SetJointSpeed(float speed_rad_per_sec);
    void SetJointAcceleration(float acc_rad_per_sec2);
    
    void UpdateJointAngles();
    void UpdateJointAnglesCallback();
    void UpdateJointPose6D();
    
    void CalibrateHomeOffset();
    void Homing();
    void Resting();
    void SetEnable(bool _enable,damiao::Control_Mode mode);
    void SetCommandMode(uint32_t _mode);
    
    bool IsMoving();
    bool IsEnabled();
    void GetOffsets();

    Joint6D_t GetCurrentJoints() const { return currentJoints; }
    Joint6D_t GetTargetJoints() const { return targetJoints; }
    Pose6D_t GetCurrentPose() const { return currentPose; }
    int GetDof() const { return dof; }

private:
    SerialPort::SharedPtr serial_;
    DmKinematics* kinematics;
    int dof;
    
    Joint6D_t currentJoints;
    Joint6D_t targetJoints;
    Joint6D_t initPose;
    Pose6D_t currentPose;
    Joint6D_t dynamicJointSpeeds = {1, 1, 1, 1, 1, 1}; // 各关节动态速度
    volatile uint8_t jointsStateFlag = 0;
    CommandMode commandMode = DEFAULT_COMMAND_MODE;
    bool isEnabled = false;
    
    float jointSpeed = DEFAULT_JOINT_SPEED;
    float jointSpeedRatio = 1.0f;
    
    CtrlStepMotor* motorJ[7] = {nullptr};
    DummyHand* hand = nullptr;
    
    void MoveJointsWithSpeed(const Joint6D_t& joints_rad, const Joint6D_t& speeds_rad_per_sec);
    float AbsMaxOf6(const Joint6D_t& joints, uint8_t& index) const;
};

#endif