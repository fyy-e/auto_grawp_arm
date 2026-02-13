#ifndef DUMMY_CORE_FW_CTRL_STEP_HPP
#define DUMMY_CORE_FW_CTRL_STEP_HPP
#include "src/u2can/damiao.h"
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
        float kp = 0;
        float kd = 0;
        float q = 0;
        float dq = 0;
        float tau = 0;
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
private:
    damiao::Motor motor;
};

#endif //DUMMY_CORE_FW_CTRL_STEP_HPP
