#include "ctrl_step.h"
#include "iostream"
CtrlStepMotor::CtrlStepMotor(damiao::DM_Motor_Type Motor_Type, Motor_id Slave_id, Motor_id Master_id)
    : Master_id(Master_id), Slave_id(Slave_id), Motor_Type(Motor_Type)
{
    this->limit_param = damiao::limit_param[Motor_Type];
    // 初始化当前实例的电机对象
    this->motor = damiao::Motor(Motor_Type, Slave_id, Master_id);
    cur_control_mode = damiao::MIT_MODE;
    // 自动将“自己”注册到全局唯一的 dm 控制器中
    dm.addMotor(&this->motor);
    dm.disable(this->motor);
}

void CtrlStepMotor::SetEnable(bool _enable,damiao::Control_Mode mode = damiao::MIT_MODE)
{
    state = _enable ? FINISH : STOP;
    cur_control_mode = mode;
    if (_enable) {
        if(dm.switchControlMode(this->motor,mode)){
            std::cout << "\n成功修改模式为： "<< mode << std::endl;
        }
        osDelay(1);
        dm.save_motor_param(this->motor);
        osDelay(1);
        dm.enable(this->motor);
        osDelay(1);
    } else {
        // 禁用电机：设置为空闲模式
       dm.disable(this->motor);
       osDelay(1);
    }
    
}
void CtrlStepMotor::SetCurrentSetPoint(float _val)
{
    state = RUNNING;
    if(cur_control_mode ==  damiao::MIT_MODE){
        mit_param.i = _val;
    }
}

void CtrlStepMotor::SetVelocitySetPoint(float _val)
{
    state = RUNNING;
    mit_param.dq = _val;
    switch (cur_control_mode)
    {
    case damiao::MIT_MODE:
        dm.control_mit(this->motor,mit_param.kp,mit_param.kd,mit_param.q,mit_param.dq,mit_param.tau);
        break;
    case damiao::POS_VEL_MODE:
        dm.control_pos_vel(this->motor,mit_param.q,mit_param.dq);
        break;
    case damiao::VEL_MODE:
        /* code */
        dm.control_vel(this->motor,mit_param.dq);
        break;
    default:
        break;
    }
}


void CtrlStepMotor::SetPositionSetPoint(float _val)
{   
    state = RUNNING;
    // 检查角度限位
    if (_val > limit_param.Q_MAX) _val = limit_param.Q_MAX;
    if (_val < -limit_param.Q_MAX) _val = -limit_param.Q_MAX;
    
    target_pos = _val;
    mit_param.q = _val;
    switch (cur_control_mode)
    {
    case damiao::MIT_MODE:
        dm.control_mit(this->motor,mit_param.kp,mit_param.kd,mit_param.q,mit_param.dq,mit_param.tau);
        break;
    case damiao::POS_VEL_MODE:
        dm.control_pos_vel(this->motor,mit_param.q,mit_param.dq);
        break;
    case damiao::VEL_MODE:
        /* code */
        dm.control_vel(this->motor,mit_param.dq);
        break;
    default:
        break;
    }
}


void CtrlStepMotor::SetPositionWithVelocityLimit(float _pos, float _vel)
{
    state = RUNNING;
    // 检查角度限位
    if (_pos > limit_param.Q_MAX) _pos = limit_param.Q_MAX;
    if (_pos < -limit_param.Q_MAX) _pos = -limit_param.Q_MAX;
    // std::cout << "\n速度限制最大值： "<< limit_param.Q_MAX << std::endl;
    target_pos = _pos;
    mit_param.q =_pos;
    mit_param.dq = _vel;
    switch (cur_control_mode)
    {
    case damiao::MIT_MODE:
        dm.control_mit(this->motor,mit_param.kp,mit_param.kd,mit_param.q,mit_param.dq,mit_param.tau);
        break;
    case damiao::POS_VEL_MODE:
        dm.control_pos_vel(this->motor,mit_param.q,mit_param.dq);
        // std::cout << "\nmit_param.q： "<< mit_param.q << "mit_param.dq： "<< mit_param.dq << std::endl;
        break;
    case damiao::VEL_MODE:
        /* code */
        dm.control_vel(this->motor,mit_param.dq);
        break;
    default:
        break;
    }

}

void CtrlStepMotor::SetAcceleration(float _val)
{
    std::cout<<"设置加速度功能未实现"<<std::endl;
}


void CtrlStepMotor::ApplyPositionAsHome()
{
// 1. 只有主线程能发指令，确保 UpdateThread 此时处于暂停状态！

    dm.disable(this->motor);
    std::cout << "Motor " << Slave_id << ": 开始校准零点..." << std::endl;
    dm.refresh_motor_status(this->motor);
    std::cout<<"motor:"<< Slave_id <<"--- POS:"<<this->motor.Get_Position()<<" VEL:"<<this->motor.Get_Velocity()<<" CUR:"<<this->motor.Get_tau()<<std::endl;
    dm.set_zero_position(this->motor); 
    osDelay(100); // 等待生效
    dm.refresh_motor_status(this->motor);
    std::cout<<"motor:"<< Slave_id <<"--- POS:"<<this->motor.Get_Position()<<" VEL:"<<this->motor.Get_Velocity()<<" CUR:"<<this->motor.Get_tau()<<std::endl;
    dm.save_motor_param(this->motor);
}


void CtrlStepMotor::SetEnableOnBoot(bool _enable)
{
        ApplyPositionAsHome();
        cur_control_mode = damiao::MIT_MODE;
        dm.switchControlMode(this->motor,cur_control_mode);  // 直接使用配置文件中的枚举，避免硬编码
        std::cout << "\n设置电机"<<Slave_id<<"为:"<< damiao::MIT_MODE <<"模式"<< std::endl;
        osDelay(20);  // 等待模式切换完成（电机内部需要时间切换控制逻辑）
}

void CtrlStepMotor::SetAngle(float _angle)
{
    SetPositionSetPoint(_angle);
}
void CtrlStepMotor::SetAngleWithVelocityLimit(float _angle, float _vel){
    SetPositionWithVelocityLimit(_angle,_vel);
}

void CtrlStepMotor::UpdateAngle() {
    dm.refresh_motor_status(this->motor);
    position = motor.Get_Position();
    velocity = motor.Get_Velocity();
    tau = motor.Get_tau();
    
    float error = std::abs(target_pos - position);
    
    // 【优化点】放宽位置阈值至 0.02 rad (~1.1度)。
    // 如果误差在 0.05 rad 以内，并且速度极低（已经卡住/停稳），也视为完成。
    if (error < 0.02f || (error < 0.05f && std::abs(velocity) < 0.05f)) {
        state = FINISH;
    } else {
        state = RUNNING;
    }
}

void CtrlStepMotor::UpdateAngleCallback(float _pos, bool _isFinished)
{
    state = _isFinished ? FINISH : RUNNING;
}

void CtrlStepMotor::setPositionVelocityTorque(float pos, float vel, float tau,float kp, float kd) {
    state = RUNNING;

    // 限位检查（位置、速度、力矩）
    if (pos > limit_param.Q_MAX)  pos = limit_param.Q_MAX;
    if (pos < -limit_param.Q_MAX) pos = -limit_param.Q_MAX;
    if (vel > limit_param.DQ_MAX) vel = limit_param.DQ_MAX;
    if (vel < -limit_param.DQ_MAX) vel = -limit_param.DQ_MAX;
    if (tau > limit_param.TAU_MAX) tau = limit_param.TAU_MAX;
    if (tau < -limit_param.TAU_MAX) tau = -limit_param.TAU_MAX;

    target_pos = pos;
    mit_param.q = pos;
    mit_param.dq = vel;
    mit_param.tau = tau;   // 存入前馈力矩
    mit_param.kp = kp;
    mit_param.kd = kd;
    // 确保当前控制模式为 MIT
    if (cur_control_mode != damiao::MIT_MODE) {
        std::cerr << "警告: 电机 " << Slave_id << " 当前模式非 MIT，力矩前馈将失效" << std::endl;
    }

    // 发送指令（dm.control_mit 内部会归一化）
    dm.control_mit(this->motor,
                   mit_param.kp,
                   mit_param.kd,
                   mit_param.q,
                   mit_param.dq,
                   mit_param.tau);
}