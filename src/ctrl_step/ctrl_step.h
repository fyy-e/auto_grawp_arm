#ifndef DUMMY_CORE_FW_CTRL_STEP_HPP
#define DUMMY_CORE_FW_CTRL_STEP_HPP

#include "motor_controller.h"
#include "socketcan.h"
#define osDelay(ms) usleep((ms)*1000)
// static SocketCan hcan = SocketCan();
class CtrlStepMotor
{
public:
    enum State
    {
        RUNNING,
        FINISH,
        STOP
    };


    const uint32_t CTRL_CIRCLE_COUNT = 200 * 256;

    CtrlStepMotor(SocketCan* _hcan, size_t _id, bool _inverse = false, uint8_t _reduction = 15,
                  float _angleLimitMin = -180, float _angleLimitMax = 180);

    size_t nodeID;
    float angle = 0;
    float angleLimitMax;
    float angleLimitMin;
    bool inverseDirection;
    uint8_t reduction = 15;
    Mode motor_mode;           //电机运行模式
    float current_zero = 0;
    State state = STOP;

    
    void SetAngle(float _angle);
    void SetAngleWithVelocityLimit(float _angle, float _vel);
    // CAN Command
    void SetEnable(bool _enable);
    void DoCalibration();
    void SetCurrentSetPoint(float _val);
    void SetVelocitySetPoint(float _val);
    void SetPositionSetPoint(float _val);
    void SetPositionWithVelocityLimit(float _pos, float _vel);
    void SetNodeID(uint32_t _id);
    void SetCurrentLimit(float _val);
    void SetVelocityLimit(float _val);
    void SetAcceleration(float _val);
    void ApplyPositionAsHome();
    void SetEnableOnBoot(bool _enable);
    void SetEnableStallProtect(bool _enable);
    void Reboot();
    void EraseConfigs();
    void save_settings_to_flash();
    float get_offset();

    void UpdateAngle();
    void UpdateAngleCallback(float _pos, bool _isFinished);
private:
    MotorController motor;
    SocketCan* hcan;
};

#endif //DUMMY_CORE_FW_CTRL_STEP_HPP
