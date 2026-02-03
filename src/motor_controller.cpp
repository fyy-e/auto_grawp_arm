// Copyright (c) 2025, The Berkeley Humanoid Lite Project Developers.

#include "motor_controller.h"
#include "motor_controller_conf.h"


MotorController::MotorController() {
}
/**
 * @brief MotorController 类：电机控制器核心控制类
 * @details 基于CAN总线实现电机的全功能控制，包括设备通信、模式配置、参数读写、PDO数据交互等
 *          依赖 SocketCan 类实现CAN帧的收发，通过设备ID区分不同电机节点
 */
// 构造函数：初始化电机控制器，绑定CAN总线对象和设备ID
MotorController::MotorController(SocketCan *bus, size_t device_id) : bus(bus), device_id(device_id) {
}

/**
 * @brief 电机Ping测试
 * @details 向目标电机发送Ping指令，检测电机是否在线
 *          发送空数据帧到指定设备的FUNC_RECEIVE_PDO_1功能ID，等待响应
 */
void MotorController::ping() {
  printf("Pinging motor controller %d\n", (int)device_id);

  // 构造Ping指令CAN帧
  can_frame tx_frame;
  tx_frame.can_id = make_can_id(device_id, FUNC_RECEIVE_PDO_1);  // 生成目标电机+功能ID的CAN标识符
  tx_frame.len = 0;  // Ping指令无数据段

  bus->write(&tx_frame);  // 发送Ping指令
  
  // 等待电机响应
  can_frame rx_frame = bus->read();

  // 注：以下为注释掉的响应校验逻辑，可根据需求启用
  if (get_device_id(rx_frame.can_id) == device_id) {
    printf("Received ping response from motor joint %ld\n", device_id);
  }
}

/**
 * @brief 发送心跳包
 * @details 向电机发送心跳指令，维持通信连接，防止电机进入保护状态
 *          发送空数据帧到FUNC_HEARTBEAT功能ID
 */
void MotorController::feed() {
  can_frame tx_frame;
  tx_frame.can_id = make_can_id(device_id, FUNC_HEARTBEAT);  // 心跳功能ID
  tx_frame.len = 0;  // 心跳包无数据段
  bus->write(&tx_frame);  // 发送心跳包
}

/**
 * @brief 设置电机工作模式
 * @param mode 目标工作模式（具体取值参考motor_controller_conf.h中的模式定义）
 * @details 通过NMT功能ID发送模式配置指令，数据段包含模式值和设备ID
 */
void MotorController::set_mode(uint8_t mode) {
  can_frame tx_frame;
  tx_frame.can_id = make_can_id(device_id, FUNC_NMT);  // NMT（网络管理）功能ID
  tx_frame.len = 2;  // 模式配置指令数据长度为2字节
  *((uint8_t *)(tx_frame.data)) = mode;                // 第1字节：模式值
  *((uint8_t *)(tx_frame.data + 1)) = device_id;       // 第2字节：目标设备ID
  bus->write(&tx_frame);  // 发送模式配置指令
}

/**
 * @brief 从Flash加载配置参数
 * @details 通过FLASH功能ID发送加载指令，指令数据为2（对应加载操作码）
 *          用于将电机Flash中存储的配置参数加载到运行内存
 */
void MotorController::load_settings_from_flash() {
  can_frame tx_frame;
  tx_frame.can_id = make_can_id(device_id, FUNC_FLASH);  // Flash操作功能ID
  tx_frame.len = 1;  // Flash操作指令数据长度为1字节
  *((uint8_t *)(tx_frame.data)) = 2;  // 操作码：2表示从Flash加载配置
  bus->write(&tx_frame);  // 发送加载指令
}

/**
 * @brief 将配置参数存储到Flash
 * @details 通过FLASH功能ID发送存储指令，指令数据为1（对应存储操作码）
 *          用于将当前运行内存中的配置参数保存到电机Flash，掉电不丢失
 */
void MotorController::store_settings_to_flash() {
  can_frame tx_frame;
  tx_frame.can_id = make_can_id(device_id, FUNC_FLASH);  // Flash操作功能ID
  tx_frame.len = 1;  // Flash操作指令数据长度为1字节
  *((uint8_t *)(tx_frame.data)) = 1;  // 操作码：1表示将配置存储到Flash
  bus->write(&tx_frame);  // 发送存储指令
}

/**
 * @brief 读取32位浮点型参数
 * @param param_id 参数ID（具体取值参考motor_controller_conf.h中的参数定义）
 * @return 读取到的浮点型参数值，读取失败返回0
 * @details 通过SDO（服务数据对象）功能ID发送读取请求，数据段包含读取标识和参数ID
 *          等待电机响应后，解析数据段中的32位浮点型数据
 */
float MotorController::read_parameter_f32(Parameter param_id) {
  can_frame tx_frame;
  tx_frame.can_id = make_can_id(device_id, FUNC_RECEIVE_SDO);  // SDO接收功能ID
  tx_frame.len = 3;  // 读取参数指令数据长度为3字节
  *((uint8_t *)(tx_frame.data)) = 0x02 << 5;  // 第1字节：0x40（二进制1000000），标识为读取操作
  *((uint16_t *)(tx_frame.data + 1)) = param_id;  // 第2-3字节：参数ID
  bus->write(&tx_frame);  // 发送参数读取请求

  can_frame rx_frame = bus->read();  // 等待电机响应
  // 校验响应的设备ID是否匹配当前电机
  if (get_device_id(rx_frame.can_id) == device_id) {
    return *((float *)(rx_frame.data));  // 解析响应数据中的32位浮点数
  }
  return 0;  // 读取失败返回0
}

/**
 * @brief 读取32位有符号整型参数
 * @param param_id 参数ID（具体取值参考motor_controller_conf.h中的参数定义）
 * @return 读取到的有符号整型参数值，读取失败返回0
 * @details 与read_parameter_f32逻辑一致，仅数据解析格式为int32_t
 */
int32_t MotorController::read_parameter_i32(Parameter param_id) {
  can_frame tx_frame;
  tx_frame.can_id = make_can_id(device_id, FUNC_RECEIVE_SDO);
  tx_frame.len = 3;
  *((uint8_t *)(tx_frame.data)) = 0x02 << 5;  // 读取操作标识
  *((uint16_t *)(tx_frame.data + 1)) = param_id;
  bus->write(&tx_frame);

  can_frame rx_frame = bus->read();
  if (get_device_id(rx_frame.can_id) == device_id) {
    return *((int32_t *)(rx_frame.data));  // 解析为32位有符号整数
  }
  return 0;
}

/**
 * @brief 读取32位无符号整型参数
 * @param param_id 参数ID（具体取值参考motor_controller_conf.h中的参数定义）
 * @return 读取到的无符号整型参数值，读取失败返回0
 * @details 与read_parameter_f32逻辑一致，仅数据解析格式为uint32_t
 */
uint32_t MotorController::read_parameter_u32(Parameter param_id) {
  can_frame tx_frame;
  tx_frame.can_id = make_can_id(device_id, FUNC_RECEIVE_SDO);
  tx_frame.len = 3;
  *((uint8_t *)(tx_frame.data)) = 0x02 << 5;  // 读取操作标识
  *((uint16_t *)(tx_frame.data + 1)) = param_id;
  bus->write(&tx_frame);

  can_frame rx_frame = bus->read();
  if (get_device_id(rx_frame.can_id) == device_id) {
    return *((uint32_t *)(rx_frame.data));  // 解析为32位无符号整数
  }
  return 0;
}

/**
 * @brief 写入32位浮点型参数
 * @param param_id 参数ID（具体取值参考motor_controller_conf.h中的参数定义）
 * @param value 要写入的浮点型参数值
 * @details 通过SDO功能ID发送写入请求，数据段包含写入标识、参数ID和参数值
 *          数据长度为8字节，确保参数值完整传输
 */
void MotorController::write_parameter_f32(Parameter param_id, float value) {
  can_frame tx_frame;
  tx_frame.can_id = make_can_id(device_id, FUNC_RECEIVE_SDO);
  tx_frame.len = 8;  // 写入浮点型参数指令数据长度为8字节
  *((uint8_t *)(tx_frame.data)) = 0x01 << 5;  // 第1字节：0x20（二进制0100000），标识为写入操作
  *((uint16_t *)(tx_frame.data + 1)) = param_id;  // 第2-3字节：参数ID
  *((uint8_t *)(tx_frame.data + 3)) = 0;  // 第4字节：保留位，置0
  *((float *)(tx_frame.data + 4)) = value;  // 第5-8字节：要写入的浮点型参数值
  bus->write(&tx_frame);  // 发送参数写入请求
}

/**
 * @brief 写入32位有符号整型参数
 * @param param_id 参数ID（具体取值参考motor_controller_conf.h中的参数定义）
 * @param value 要写入的有符号整型参数值
 * @details 与write_parameter_f32逻辑一致，仅写入数据格式为int32_t，数据长度为7字节
 */
void MotorController::write_parameter_i32(Parameter param_id, int32_t value) {
  can_frame tx_frame;
  tx_frame.can_id = make_can_id(device_id, FUNC_RECEIVE_SDO);
  tx_frame.len = 7;  // 写入整型参数指令数据长度为7字节
  *((uint8_t *)(tx_frame.data)) = 0x02 << 5;  // 写入操作标识
  *((uint16_t *)(tx_frame.data + 1)) = param_id;  // 参数ID
  *((uint8_t *)(tx_frame.data + 3)) = 0;  // 保留位
  *((int32_t *)(tx_frame.data + 4)) = value;  // 要写入的32位有符号整数
  bus->write(&tx_frame);
}

/**
 * @brief 写入32位无符号整型参数
 * @param param_id 参数ID（具体取值参考motor_controller_conf.h中的参数定义）
 * @param value 要写入的无符号整型参数值
 * @details 与write_parameter_f32逻辑一致，仅写入数据格式为uint32_t，数据长度为7字节
 */
void MotorController::write_parameter_u32(Parameter param_id, uint32_t value) {
  can_frame tx_frame;
  tx_frame.can_id = make_can_id(device_id, FUNC_RECEIVE_SDO);
  tx_frame.len = 7;  // 写入整型参数指令数据长度为7字节
  *((uint8_t *)(tx_frame.data)) = 0x02 << 5;  // 写入操作标识
  *((uint16_t *)(tx_frame.data + 1)) = param_id;  // 参数ID
  *((uint8_t *)(tx_frame.data + 3)) = 0;  // 保留位
  *((uint32_t *)(tx_frame.data + 4)) = value;  // 要写入的32位无符号整数
  bus->write(&tx_frame);
}

/**
 * @brief 读取快速帧频率参数
 * @return 快速帧频率值（浮点型）
 * @details 封装read_parameter_f32，专门读取PARAM_FAST_FRAME_FREQUENCY参数
 */
uint32_t MotorController::read_fast_frame_frequency() {
  return read_parameter_f32(PARAM_FAST_FRAME_FREQUENCY);
}

/**
 * @brief 写入快速帧频率参数
 * @param frequency 目标快速帧频率值
 * @details 封装write_parameter_f32，专门写入PARAM_FAST_FRAME_FREQUENCY参数
 */
void MotorController::write_fast_frame_frequency(uint32_t frequency) {
  write_parameter_f32(PARAM_FAST_FRAME_FREQUENCY, frequency);
}

/**
 * @brief 读取齿轮比参数
 * @return 齿轮比数值（浮点型）
 * @details 封装read_parameter_f32，专门读取PARAM_POSITION_CONTROLLER_GEAR_RATIO参数
 */
float MotorController::read_gear_ratio() {
  return read_parameter_f32(PARAM_POSITION_CONTROLLER_GEAR_RATIO);
}

/**
 * @brief 写入齿轮比参数
 * @param ratio 目标齿轮比数值
 * @details 封装write_parameter_f32，专门写入PARAM_POSITION_CONTROLLER_GEAR_RATIO参数
 */
void MotorController::write_gear_ratio(float ratio) {
  write_parameter_f32(PARAM_POSITION_CONTROLLER_GEAR_RATIO, ratio);
}

/**
 * @brief 读取位置环比例系数（KP）
 * @return 位置KP值（浮点型）
 * @details 封装read_parameter_f32，专门读取PARAM_POSITION_CONTROLLER_POSITION_KP参数
 */
float MotorController::read_position_kp() {
  return read_parameter_f32(PARAM_POSITION_CONTROLLER_POSITION_KP);
}

/**
 * @brief 写入位置环比例系数（KP）
 * @param kp 目标位置KP值
 * @details 封装write_parameter_f32，专门写入PARAM_POSITION_CONTROLLER_POSITION_KP参数
 */
void MotorController::write_position_kp(float kp) {
  write_parameter_f32(PARAM_POSITION_CONTROLLER_POSITION_KP, kp);
}

/**
 * @brief 读取位置环微分系数（KD）
 * @return 位置KD值（浮点型）
 * @details 封装read_parameter_f32，专门读取PARAM_POSITION_CONTROLLER_VELOCITY_KP参数
 *          注：此处参数ID为VELOCITY_KP，推测为代码设计中位置环KD复用该参数ID
 */
float MotorController::read_position_kd() {
  return read_parameter_f32(PARAM_POSITION_CONTROLLER_VELOCITY_KP);
}

/**
 * @brief 写入位置环微分系数（KD）
 * @param kd 目标位置KD值
 * @details 封装write_parameter_f32，专门写入PARAM_POSITION_CONTROLLER_VELOCITY_KP参数
 */
void MotorController::write_position_kd(float kd) {
  write_parameter_f32(PARAM_POSITION_CONTROLLER_VELOCITY_KP, kd);
}

/**
 * @brief 读取位置环积分系数（KI）
 * @return 位置KI值（浮点型）
 * @details 封装read_parameter_f32，专门读取PARAM_POSITION_CONTROLLER_POSITION_KI参数
 */
float MotorController::read_position_ki() {
  return read_parameter_f32(PARAM_POSITION_CONTROLLER_POSITION_KI);
}

/**
 * @brief 写入位置环积分系数（KI）
 * @param ki 目标位置KI值
 * @details 封装write_parameter_f32，专门写入PARAM_POSITION_CONTROLLER_POSITION_KI参数
 */
void MotorController::write_position_ki(float ki) {
  write_parameter_f32(PARAM_POSITION_CONTROLLER_POSITION_KI, ki);
}

/**
 * @brief 读取速度环比例系数（KP）
 * @return 速度KP值（浮点型）
 * @details 封装read_parameter_f32，专门读取PARAM_POSITION_CONTROLLER_VELOCITY_KP参数
 */
float MotorController::read_velocity_kp() {
  return read_parameter_f32(PARAM_POSITION_CONTROLLER_VELOCITY_KP);
}

/**
 * @brief 写入速度环比例系数（KP）
 * @param kp 目标速度KP值
 * @details 封装write_parameter_f32，专门写入PARAM_POSITION_CONTROLLER_VELOCITY_KP参数
 */
void MotorController::write_velocity_kp(float kp) {
  write_parameter_f32(PARAM_POSITION_CONTROLLER_VELOCITY_KP, kp);
}

/**
 * @brief 读取速度环积分系数（KI）
 * @return 速度KI值（浮点型）
 * @details 封装read_parameter_f32，专门读取PARAM_POSITION_CONTROLLER_VELOCITY_KI参数
 */
float MotorController::read_velocity_ki() {
  return read_parameter_f32(PARAM_POSITION_CONTROLLER_VELOCITY_KI);
}

/**
 * @brief 写入速度环积分系数（KI）
 * @param ki 目标速度KI值
 * @details 封装write_parameter_f32，专门写入PARAM_POSITION_CONTROLLER_VELOCITY_KI参数
 */
void MotorController::write_velocity_ki(float ki) {
  write_parameter_f32(PARAM_POSITION_CONTROLLER_VELOCITY_KI, ki);
}

/**
 * @brief 读取扭矩限制值
 * @return 扭矩限制值（浮点型）
 * @details 封装read_parameter_f32，专门读取PARAM_POSITION_CONTROLLER_TORQUE_LIMIT参数
 */
float MotorController::read_torque_limit() {
  return read_parameter_f32(PARAM_POSITION_CONTROLLER_TORQUE_LIMIT);
}

/**
 * @brief 写入扭矩限制值
 * @param torque 目标扭矩限制值
 * @details 封装write_parameter_f32，专门写入PARAM_POSITION_CONTROLLER_TORQUE_LIMIT参数
 */
void MotorController::write_torque_limit(float torque) {
  write_parameter_f32(PARAM_POSITION_CONTROLLER_TORQUE_LIMIT, torque);
}

/**
 * @brief 读取速度限制值
 * @return 速度限制值（浮点型）
 * @details 封装read_parameter_f32，专门读取PARAM_POSITION_CONTROLLER_VELOCITY_LIMIT参数
 */
float MotorController::read_velocity_limit() {
  return read_parameter_f32(PARAM_POSITION_CONTROLLER_VELOCITY_LIMIT);
}

/**
 * @brief 写入速度限制值
 * @param limit 目标速度限制值
 * @details 封装write_parameter_f32，专门写入PARAM_POSITION_CONTROLLER_VELOCITY_LIMIT参数
 */
void MotorController::write_velocity_limit(float limit) {
  write_parameter_f32(PARAM_POSITION_CONTROLLER_VELOCITY_LIMIT, limit);
}

/**
 * @brief 读取位置下限值
 * @return 位置下限值（浮点型）
 * @details 封装read_parameter_f32，专门读取PARAM_POSITION_CONTROLLER_POSITION_LIMIT_LOWER参数
 */
float MotorController::read_position_limit_lower() {
  return read_parameter_f32(PARAM_POSITION_CONTROLLER_POSITION_LIMIT_LOWER);
}

/**
 * @brief 写入位置下限值
 * @param limit 目标位置下限值
 * @details 封装write_parameter_f32，专门写入PARAM_POSITION_CONTROLLER_POSITION_LIMIT_LOWER参数
 */
void MotorController::write_position_limit_lower(float limit) {
  write_parameter_f32(PARAM_POSITION_CONTROLLER_POSITION_LIMIT_LOWER, limit);
}

/**
 * @brief 读取位置上限值
 * @return 位置上限值（浮点型）
 * @details 封装read_parameter_f32，专门读取PARAM_POSITION_CONTROLLER_POSITION_LIMIT_UPPER参数
 */
float MotorController::read_position_limit_upper() {
  return read_parameter_f32(PARAM_POSITION_CONTROLLER_POSITION_LIMIT_UPPER);
}

/**
 * @brief 写入位置上限值
 * @param limit 目标位置上限值
 * @details 封装write_parameter_f32，专门写入PARAM_POSITION_CONTROLLER_POSITION_LIMIT_UPPER参数
 */
void MotorController::write_position_limit_upper(float limit) {
  write_parameter_f32(PARAM_POSITION_CONTROLLER_POSITION_LIMIT_UPPER, limit);
}

/**
 * @brief 读取位置偏移量
 * @return 位置偏移量（浮点型）
 * @details 封装read_parameter_f32，专门读取PARAM_POSITION_CONTROLLER_POSITION_OFFSET参数
 */
float MotorController::read_position_offset() {
  return read_parameter_f32(PARAM_POSITION_CONTROLLER_POSITION_OFFSET);
}

/**
 * @brief 写入位置偏移量
 * @param offset 目标位置偏移量
 * @details 封装write_parameter_f32，专门写入PARAM_POSITION_CONTROLLER_POSITION_OFFSET参数
 */
void MotorController::write_position_offset(float offset) {
  write_parameter_f32(PARAM_POSITION_CONTROLLER_POSITION_OFFSET, offset);
}

/**
 * @brief 读取目标扭矩值
 * @return 目标扭矩值（浮点型）
 * @details 封装read_parameter_f32，专门读取PARAM_POSITION_CONTROLLER_TORQUE_TARGET参数
 */
float MotorController::read_torque_target() {
  return read_parameter_f32(PARAM_POSITION_CONTROLLER_TORQUE_TARGET);
}

/**
 * @brief 写入目标扭矩值
 * @param torque 目标扭矩值
 * @details 封装write_parameter_f32，专门写入PARAM_POSITION_CONTROLLER_TORQUE_TARGET参数
 */
void MotorController::write_torque_target(float torque) {
  write_parameter_f32(PARAM_POSITION_CONTROLLER_TORQUE_TARGET, torque);
}

/**
 * @brief 读取实际扭矩测量值
 * @return 实际扭矩测量值（浮点型）
 * @details 封装read_parameter_f32，专门读取PARAM_POSITION_CONTROLLER_TORQUE_MEASURED参数
 */
float MotorController::read_torque_measured() {
  return read_parameter_f32(PARAM_POSITION_CONTROLLER_TORQUE_MEASURED);
}

/**
 * @brief 读取实际速度测量值
 * @return 实际速度测量值（浮点型）
 * @details 封装read_parameter_f32，专门读取PARAM_POSITION_CONTROLLER_VELOCITY_MEASURED参数
 */
float MotorController::read_velocity_measured() {
  return read_parameter_f32(PARAM_POSITION_CONTROLLER_VELOCITY_MEASURED);
}

/**
 * @brief 读取目标位置值
 * @return 目标位置值（浮点型）
 * @details 封装read_parameter_f32，专门读取PARAM_POSITION_CONTROLLER_POSITION_TARGET参数
 */
float MotorController::read_position_target() {
  return read_parameter_f32(PARAM_POSITION_CONTROLLER_POSITION_TARGET);
}

/**
 * @brief 写入目标位置值
 * @param position 目标位置值
 * @details 封装write_parameter_f32，专门写入PARAM_POSITION_CONTROLLER_POSITION_TARGET参数
 */
void MotorController::write_position_target(float position) {
  write_parameter_f32(PARAM_POSITION_CONTROLLER_POSITION_TARGET, position);
}

/**
 * @brief 读取实际位置测量值
 * @return 实际位置测量值（浮点型）
 * @details 封装read_parameter_f32，专门读取PARAM_POSITION_CONTROLLER_POSITION_MEASURED参数
 */
float MotorController::read_position_measured() {
  return read_parameter_f32(PARAM_POSITION_CONTROLLER_POSITION_MEASURED);
}

/**
 * @brief 读取扭矩滤波系数Alpha
 * @return 扭矩滤波Alpha值（浮点型）
 * @details 封装read_parameter_f32，专门读取PARAM_POSITION_CONTROLLER_TORQUE_FILTER_ALPHA参数
 */
float MotorController::read_torque_filter_alpha() {
  return read_parameter_f32(PARAM_POSITION_CONTROLLER_TORQUE_FILTER_ALPHA);
}

/**
 * @brief 写入扭矩滤波系数Alpha
 * @param alpha 目标扭矩滤波Alpha值（范围0~1）
 * @details 封装write_parameter_f32，专门写入PARAM_POSITION_CONTROLLER_TORQUE_FILTER_ALPHA参数
 */
void MotorController::write_torque_filter_alpha(float alpha) {
  write_parameter_f32(PARAM_POSITION_CONTROLLER_TORQUE_FILTER_ALPHA, alpha);
}

/**
 * @brief 读取电流限制值
 * @return 电流限制值（浮点型）
 * @details 封装read_parameter_f32，专门读取PARAM_CURRENT_CONTROLLER_I_LIMIT参数
 */
float MotorController::read_current_limit() {
  return read_parameter_f32(PARAM_CURRENT_CONTROLLER_I_LIMIT);
}

/**
 * @brief 写入电流限制值
 * @param current 目标电流限制值
 * @details 封装write_parameter_f32，专门写入PARAM_CURRENT_CONTROLLER_I_LIMIT参数
 */
void MotorController::write_current_limit(float current) {
  write_parameter_f32(PARAM_CURRENT_CONTROLLER_I_LIMIT, current);
}

/**
 * @brief 读取电流环比例系数（KP）
 * @return 电流KP值（浮点型）
 * @details 封装read_parameter_f32，专门读取PARAM_CURRENT_CONTROLLER_I_KP参数
 */
float MotorController::read_current_kp() {
  return read_parameter_f32(PARAM_CURRENT_CONTROLLER_I_KP);
}

/**
 * @brief 写入电流环比例系数（KP）
 * @param kp 目标电流KP值
 * @details 封装write_parameter_f32，专门写入PARAM_CURRENT_CONTROLLER_I_KP参数
 */
void MotorController::write_current_kp(float kp) {
  write_parameter_f32(PARAM_CURRENT_CONTROLLER_I_KP, kp);
}

/**
 * @brief 读取电流环积分系数（KI）
 * @return 电流KI值（浮点型）
 * @details 封装read_parameter_f32，专门读取PARAM_CURRENT_CONTROLLER_I_KI参数
 */
float MotorController::read_current_ki() {
  return read_parameter_f32(PARAM_CURRENT_CONTROLLER_I_KI);
}

/**
 * @brief 写入电流环积分系数（KI）
 * @param ki 目标电流KI值
 * @details 封装write_parameter_f32，专门写入PARAM_CURRENT_CONTROLLER_I_KI参数
 */
void MotorController::write_current_ki(float ki) {
  write_parameter_f32(PARAM_CURRENT_CONTROLLER_I_KI, ki);
}

/**
 * @brief 读取母线电压滤波系数Alpha
 * @return 母线电压滤波Alpha值（浮点型）
 * @details 封装read_parameter_f32，专门读取PARAM_POWERSTAGE_BUS_VOLTAGE_FILTER_ALPHA参数
 */
float MotorController::read_bus_voltage_filter_alpha() {
  return read_parameter_f32(PARAM_POWERSTAGE_BUS_VOLTAGE_FILTER_ALPHA);
}

/**
 * @brief 写入母线电压滤波系数Alpha
 * @param alpha 目标母线电压滤波Alpha值（范围0~1）
 * @details 封装write_parameter_f32，专门写入PARAM_POWERSTAGE_BUS_VOLTAGE_FILTER_ALPHA参数
 */
void MotorController::write_bus_voltage_filter_alpha(float alpha) {
  write_parameter_f32(PARAM_POWERSTAGE_BUS_VOLTAGE_FILTER_ALPHA, alpha);
}

/**
 * @brief 读取电机扭矩常数
 * @return 电机扭矩常数值（浮点型）
 * @details 封装read_parameter_f32，专门读取PARAM_MOTOR_TORQUE_CONSTANT参数
 */
float MotorController::read_motor_torque_constant() {
  return read_parameter_f32(PARAM_MOTOR_TORQUE_CONSTANT);
}

/**
 * @brief 写入电机扭矩常数
 * @param torque_constant 目标电机扭矩常数值
 * @details 封装write_parameter_f32，专门写入PARAM_MOTOR_TORQUE_CONSTANT参数
 */
void MotorController::write_motor_torque_constant(float torque_constant) {
  write_parameter_f32(PARAM_MOTOR_TORQUE_CONSTANT, torque_constant);
}

/**
 * @brief 读取电机相序
 * @return 电机相序值（整型）
 * @details 封装read_parameter_i32，专门读取PARAM_MOTOR_PHASE_ORDER参数
 */
int MotorController::read_motor_phase_order() {
  return read_parameter_i32(PARAM_MOTOR_PHASE_ORDER);
}

/**
 * @brief 读取编码器速度滤波系数Alpha
 * @return 编码器速度滤波Alpha值（浮点型）
 * @details 封装read_parameter_f32，专门读取PARAM_ENCODER_VELOCITY_FILTER_ALPHA参数
 */
float MotorController::read_encoder_velocity_filter_alpha() {
  return read_parameter_f32(PARAM_ENCODER_VELOCITY_FILTER_ALPHA);
}

/**
 * @brief 写入编码器速度滤波系数Alpha
 * @param alpha 目标编码器速度滤波Alpha值（范围0~1）
 * @details 封装write_parameter_f32，专门写入PARAM_ENCODER_VELOCITY_FILTER_ALPHA参数
 */
void MotorController::write_encoder_velocity_filter_alpha(float alpha) {
  write_parameter_f32(PARAM_ENCODER_VELOCITY_FILTER_ALPHA, alpha);
}

/**
 * @brief 自动计算并设置电流环带宽参数（KP、KI）
 * @param bandwidth_hz 目标带宽（单位：Hz）
 * @param phase_resistance 电机相电阻（单位：Ω）
 * @param phase_inductance 电机相电感（单位：H）
 * @details 根据电流环带宽公式计算KP和KI值，自动写入对应参数
 *          公式：KP = 2π×带宽×电感；KI = 电阻/电感
 */
void MotorController::set_current_bandwidth(float bandwidth_hz, float phase_resistance, float phase_inductance) {
  float kp = bandwidth_hz * 2.0 * M_PI * phase_inductance;  // 计算电流环KP
  float ki = phase_resistance / phase_inductance;            // 计算电流环KI
  write_current_kp(kp);  // 写入计算得到的KP值
  write_current_ki(ki);  // 写入计算得到的KI值
}

/**
 * @brief 自动计算并设置扭矩滤波带宽
 * @param bandwidth_hz 目标带宽（单位：Hz）
 * @param position_loop_rate 位置环更新频率（单位：Hz）
 * @details 根据一阶低通滤波公式计算Alpha系数，自动写入扭矩滤波Alpha参数
 *          公式：Alpha = 1 - exp(-2π×带宽/更新频率)，并限制范围在0~1
 */
void MotorController::set_torque_bandwidth(float bandwidth_hz, float position_loop_rate) {
  float alpha = fmin(fmax(1. - exp(-2. * M_PI * (bandwidth_hz / position_loop_rate)), 0.), 1.);
  write_torque_filter_alpha(alpha);  // 写入计算得到的扭矩滤波Alpha值
}

/**
 * @brief 自动计算并设置母线电压滤波带宽
 * @param bandwidth_hz 目标带宽（单位：Hz）
 * @param bus_voltage_update_rate 母线电压更新频率（单位：Hz）
 * @details 与set_torque_bandwidth逻辑一致，计算母线电压滤波Alpha系数并写入
 */
void MotorController::set_bus_voltage_bandwidth(float bandwidth_hz, float bus_voltage_update_rate) {
  float alpha = fmin(fmax(1. - exp(-2. * M_PI * (bandwidth_hz / bus_voltage_update_rate)), 0.), 1.);
  write_bus_voltage_filter_alpha(alpha);  // 写入计算得到的母线电压滤波Alpha值
}

/**
 * @brief 自动计算并设置编码器速度滤波带宽
 * @param bandwidth_hz 目标带宽（单位：Hz）
 * @param encoder_update_rate 编码器更新频率（单位：Hz）
 * @details 与set_torque_bandwidth逻辑一致，计算编码器速度滤波Alpha系数并写入
 */
void MotorController::set_encoder_velocity_bandwidth(float bandwidth_hz, float encoder_update_rate) {
  float alpha = fmin(fmax(1. - exp(-2. * M_PI * (bandwidth_hz / encoder_update_rate)), 0.), 1.);
  write_encoder_velocity_filter_alpha(alpha);  // 写入计算得到的编码器速度滤波Alpha值
}

/**
 * @brief 读取PDO2数据（位置、速度测量值）
 * @details 从CAN总线读取PDO2数据帧，解析出实际位置和速度测量值并存储到成员变量
 *          PDO2为过程数据对象，用于快速传输实时控制数据
 */
void MotorController::read_pdo_2() {
  can_frame rx_frame = bus->read();  // 读取CAN总线数据
  // 校验响应的设备ID是否匹配当前电机
  if (get_device_id(rx_frame.can_id) == device_id) {
    // 解析数据段：第0-3字节为位置测量值，第4-7字节为速度测量值
    this->position_measured = *((float *)rx_frame.data + 0);
    this->velocity_measured = *((float *)rx_frame.data + 1);
  }
}

/**
 * @brief 写入PDO2数据（位置、速度目标值）
 * @details 构造PDO2数据帧，将成员变量中的位置和速度目标值写入数据段并发送
 *          用于向电机发送实时控制指令
 */
void MotorController::write_pdo_2() {
  can_frame tx_frame;
  tx_frame.can_id = make_can_id(device_id, FUNC_RECEIVE_PDO_2);  // PDO2接收功能ID
  tx_frame.len = 8;  // PDO2数据长度为8字节（2个32位浮点数）
  // 封装数据：第0-3字节为位置目标值，第4-7字节为速度目标值
  *((float *)tx_frame.data + 0) = this->position_target;
  *((float *)tx_frame.data + 1) = this->velocity_target;

  bus->write(&tx_frame);  // 发送PDO2控制指令
}

/**
 * @brief 获取存储的位置测量值
 * @return 位置测量值（浮点型）
 * @details 读取read_pdo_2解析后存储在成员变量position_measured的值
 */
float MotorController::get_measured_position() {
  return this->position_measured;
}

/**
 * @brief 获取存储的速度测量值
 * @return 速度测量值（浮点型）
 * @details 读取read_pdo_2解析后存储在成员变量velocity_measured的值
 */
float MotorController::get_measured_velocity() {
  return this->velocity_measured;
}

/**
 * @brief 设置位置目标值
 * @param position 目标位置值
 * @details 将输入的位置目标值存储到成员变量position_target，供write_pdo_2使用
 */
void MotorController::set_target_position(float position) {
  this->position_target = position;
}

/**
 * @brief 设置速度目标值
 * @param velocity 目标速度值
 * @details 将输入的速度目标值存储到成员变量velocity_target，供write_pdo_2使用
 */
void MotorController::set_target_velocity(float velocity) {
  this->velocity_target = velocity;
}
// --- 新增函数的实现 ---
void MotorController::set_encoder_offset(float offset) {
    // 这是一个公有包装器，它在内部调用私有函数
    // PARAM_ENCODER_POSITION_OFFSET 是你在 conf.h 里定义的地址 (通常是 0x124U 或类似)
    write_parameter_f32(PARAM_ENCODER_POSITION_OFFSET, offset);
}

void MotorController::save_config() {
    // 假设 PARAM_STORE_CONFIG 是触发保存的参数ID，值写1触发
    // 具体的参数ID请查看你的 motor_controller_conf.h
    // write_parameter_u32(PARAM_STORE_CONFIG, 1); 
    // 如果没有u32写函数，暂时忽略或补充一个
}