#ifndef DUMMY_CORE_FW_CTRL_STEP_HPP
#define DUMMY_CORE_FW_CTRL_STEP_HPP
#include "u2can/damiao.h"
#include "unistd.h"
#include <cmath>
#define osDelay(ms) usleep((ms)*1000)
class CtrlStepMotor
{
public:
    enum State
    {
        RUNNING,
        FINISH,
        STOP
    };
    typedef struct
    {
        /* data */
        float kp = 0.1;
        float kd = 0.2;
        float q = 0;
        float dq = 0;
        float tau = 0.1;
        float i = 0;

    }MIT_param;
    
    CtrlStepMotor(damiao::DM_Motor_Type Motor_Type, Motor_id Slave_id, Motor_id Master_id);

    Motor_id Master_id;
    Motor_id Slave_id;
    State state = STOP;
    MIT_param mit_param;
    float position;
    float velocity;
    float tau;
    float target_pos;
    damiao::Limit_param  limit_param{};
    damiao::DM_Motor_Type Motor_Type;
    damiao::Control_Mode cur_control_mode;

    static std::shared_ptr<SerialPort> serial;
    static damiao::Motor_Control dm;
    
    void SetAngle(float _angle);
    void SetAngleWithVelocityLimit(float _angle, float _vel);
    // CAN Command
    void SetEnable(bool _enable,damiao::Control_Mode mode);
    void SetCurrentSetPoint(float _val);
    void SetVelocitySetPoint(float _val);
    void SetPositionSetPoint(float _val);
    void SetPositionWithVelocityLimit(float _pos, float _vel);
    void SetAcceleration(float _val);
    void ApplyPositionAsHome();
    void SetEnableOnBoot(bool _enable);

    void UpdateAngle();
    void UpdateAngleCallback(float _pos, bool _isFinished);
        /**
     * @brief 设置目标位置、速度及前馈力矩（MIT 模式）
     * @param pos   目标位置（弧度，已扣除零偏）
     * @param vel   目标速度（弧度/秒）
     * @param tau   前馈力矩（N·m）
     */
    void setPositionVelocityTorque(float pos, float vel, float tau,float kp, float kd);
private:
    damiao::Motor motor;
};

#endif //DUMMY_CORE_FW_CTRL_STEP_HPP
