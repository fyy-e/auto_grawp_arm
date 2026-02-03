#include "ctrl_step.h"
#include "iostream"
CtrlStepMotor::CtrlStepMotor(SocketCan* _hcan, size_t _id, bool _inverse,
                             uint8_t _reduction, float _angleLimitMin, float _angleLimitMax) :
    hcan(_hcan)
{
    nodeID = _id;
    angleLimitMax = _angleLimitMax;
    angleLimitMin = _angleLimitMin;
    inverseDirection = _inverse;
    // reduction = _reduction;
    motor = MotorController(_hcan,nodeID);
    motor.set_mode(MODE_IDLE);
}


void CtrlStepMotor::SetEnable(bool _enable)
{
    state = _enable ? FINISH : STOP;
    motor.feed();  // 发送心跳
    if (_enable) {
        // 启用电机：设置为位置模式
        motor.set_mode(MODE_POSITION);
    } else {
        // 禁用电机：设置为空闲模式
        motor.set_mode(MODE_IDLE);
    }
    osDelay(10);
}
void CtrlStepMotor::SetCurrentSetPoint(float _val)
{
    state = RUNNING;
    
    // 转换为扭矩设定值（假设单位转换）
    float torque = _val * 0.1f;  // 比例系数需要根据实际电机调整
    motor.write_torque_target(torque);
    // 切换为扭矩模式
    motor.set_mode(MODE_TORQUE);
}


void CtrlStepMotor::SetVelocitySetPoint(float _val)
{
    state = RUNNING;
    // 转换单位：度/秒 -> 弧度/秒
    float velocity_rad = _val *(float)reduction;
    if(inverseDirection){
        velocity_rad = -velocity_rad;
    }
    motor.set_target_velocity(velocity_rad);
    // 切换为速度模式
    motor.set_mode(MODE_VELOCITY);

    motor.write_pdo_2();
    osDelay(2);
}


void CtrlStepMotor::SetPositionSetPoint(float _val)
{
    // 检查角度限位
    if (_val > angleLimitMax) _val = angleLimitMax;
    if (_val < angleLimitMin) _val = angleLimitMin;
    
    // 转换为弧度：步进电机计数 -> 弧度
    float position_rad = _val * (float)reduction;
    if(inverseDirection){
        position_rad = -position_rad;
    }
    // 设置目标位置
    motor.write_position_target(position_rad);
    // 更新状态
    state = RUNNING;
}


void CtrlStepMotor::SetPositionWithVelocityLimit(float _pos, float _vel)
{
    // 检查角度限位
    if (_pos > angleLimitMax) _pos = angleLimitMax;
    if (_pos < angleLimitMin) _pos = angleLimitMin;
    
    // 考虑减速比
    float position_rad = _pos *(float)reduction;
    
    // 速度转换：度/秒 -> 弧度/秒
    float velocity_rad = _vel *(float)reduction;
        if(inverseDirection){
        position_rad = -position_rad;
        velocity_rad = -velocity_rad;
    }
    motor.set_target_position(position_rad);
    motor.set_target_velocity(velocity_rad);
    
    // 发送PDO2控制指令
    motor.write_pdo_2();
    // 更新状态
    state = RUNNING;
}

void CtrlStepMotor::SetNodeID(uint32_t _id)
{
    std::cout << "电机ID不可随意更改" << std::endl;
}


void CtrlStepMotor::SetCurrentLimit(float _val)
{
    motor.write_current_limit(_val);
    std::cout << "Motor " << (int)nodeID << ": Current limit set to " << _val << " A" << std::endl;
}


void CtrlStepMotor::SetVelocityLimit(float _val)
{
    // 转换单位：度/秒 -> 弧度/秒
    float velocity_rad = _val*(float)reduction;
    motor.write_velocity_limit(velocity_rad);
    std::cout << "Motor " << (int)nodeID << ": Velocity limit set to " << _val << " deg/s" << std::endl;
}


void CtrlStepMotor::SetAcceleration(float _val)
{
    std::cout<<"设置加速度功能未实现"<<std::endl;
    SetVelocityLimit(_val);
}


void CtrlStepMotor::ApplyPositionAsHome()
{
// 1. 只有主线程能发指令，确保 UpdateThread 此时处于暂停状态！
    // (如果在初始化阶段调用此函数，确保还没启动线程)
    
    std::cout << "Motor " << nodeID << ": 开始校准零点..." << std::endl;

    // 2. 先清零，以便读取纯净的物理值
    motor.set_encoder_offset(0.0f); // <--- 调用新写的公有函数
    osDelay(100); // 等待生效

    // 3. 读取当前位置 (注意：read_parameter_f32 如果也是私有，同样需要写个公有 wrapper)
    // 假设这里你已经有了读取位置的方法，比如 read_position_measured()
    // 或者是通过公有接口 motor.read_parameter_f32_public(...)
    
    // 如果 motor.read_parameter_f32 也是私有的，你需要去 motor_controller 加一个 read_position_measured_raw()
    // 这里暂时假设你之前的 read_position_measured() 可用且能读到底层值
    float current_output_pos = motor.read_position_measured();

    // 4. 核心修正：计算抵消该位置所需的编码器偏移量
    // 公式：Offset = - (输出轴角度 * 减速比)
    // 注意方向：如果编码器方向和输出轴定义相反，这里可能不需要负号，先试带负号的
    float new_offset = - (current_output_pos);

    std::cout << "当前输出轴: " << current_output_pos 
              << " 减速比: " << reduction 
              << " 写入偏移: " << new_offset << std::endl;

    // 5. 写入计算好的偏移量
    motor.set_encoder_offset(new_offset); // <--- 调用新写的公有函数
    osDelay(50);
    
    // 6. 此时应该保存配置
    // motor.save_config();
}


void CtrlStepMotor::SetEnableOnBoot(bool _enable)
{
        motor.ping();
        motor.feed();
        ApplyPositionAsHome();
        motor_mode = MODE_POSITION;
        motor.set_mode(motor_mode);  // 直接使用配置文件中的枚举，避免硬编码
        std::cout << "\n设置电机"<<nodeID<<"为:"<< motor_mode <<"模式"<< std::endl;
        osDelay(20);  // 等待模式切换完成（电机内部需要时间切换控制逻辑）
}


void CtrlStepMotor::SetEnableStallProtect(bool _enable)
{
    std::cout<<"失速保护未实现"<<std::endl;
}


void CtrlStepMotor::Reboot()
{
    std::cout << "Motor " << (int)nodeID << ": Reboot command sent" << std::endl;
    // 发送重启命令（需要MotorController支持）
    SetPositionWithVelocityLimit(0.0,0.0);
    osDelay(20);
    // motor.set_mode(MODE_IDLE);
    // // osDelay(100);  // 100ms延迟
    motor.set_mode(MODE_IDLE);    
}


void CtrlStepMotor::EraseConfigs()
{
        std::cout<<"不能擦除带电机配置"<<std::endl;
}


void CtrlStepMotor::SetAngle(float _angle)
{
    if(inverseDirection){
        angle = -angle;
    }
    SetPositionSetPoint(_angle);
}


void CtrlStepMotor::SetAngleWithVelocityLimit(float _angle, float _vel) {
    SetPositionWithVelocityLimit(_angle, _vel);
}

void CtrlStepMotor::UpdateAngle() {
    float new_angle = motor.read_position_measured();
    // 检查 read 是否真的成功了（建议在 MotorController 里增加成功标志）
    // 如果通信超时，不要更新 angle，也不要改变 state
    angle = new_angle / (float)reduction;
    if(inverseDirection){
        angle = -angle;
    }
    // 增加一个物理位置判断
    float error = std::abs(motor.read_position_target()/reduction - angle);
    if (error > 0.01) { // 这里的阈值根据实际情况定
        state = RUNNING;
    } else {
        state = FINISH;
    }

}


void CtrlStepMotor::UpdateAngleCallback(float _pos, bool _isFinished)
{
    state = _isFinished ? FINISH : RUNNING;
}

float CtrlStepMotor::get_offset(){
    return motor.read_position_offset();
}

void CtrlStepMotor::save_settings_to_flash(){
    motor.store_settings_to_flash();
}