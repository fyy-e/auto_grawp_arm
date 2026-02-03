
// Copyright (c) 2025, The Berkeley Humanoid Lite Project Developers.

/**
 * @file motor_controller.h
 * @brief 电机控制器核心类头文件
 * @details 定义了 MotorController 类的接口，封装了基于 CAN 总线的电机控制逻辑，
 *          包括 CAN 帧解析工具函数、电机参数读写、控制指令发送、状态数据读取等功能，
 *          依赖 SocketCan 类实现底层 CAN 通信，依赖 motor_controller_conf.h 定义配置常量
 */
#pragma once

#include <stdint.h>
#include <math.h>

#include "socketcan.h"       // 底层 CAN 总线通信类头文件
#include "motor_controller_conf.h"  // 电机控制器配置常量头文件（含参数ID、功能ID等）

/**
 * @brief CAN ID 解析相关宏定义
 * @details CAN ID 由「功能ID」和「设备ID」两部分组成，共32位，格式如下：
 *          [31:11] 保留位 | [10:7] 功能ID | [6:0] 设备ID
 */
#define DEVICE_ID_MSK  0b1111111    // 设备ID掩码（低7位），用于提取CAN ID中的设备ID
#define FUNC_ID_POS    7             // 功能ID起始位（从第7位开始，占4位）
#define FUNC_ID_MSK    (0b1111 << FUNC_ID_POS)  // 功能ID掩码（第7-10位），用于提取CAN ID中的功能ID

/**
 * @brief CAN ID 辅助工具函数：生成完整的 CAN ID
 * @param device_id 电机设备ID（0-127，对应CAN ID低7位）
 * @param func_id 功能ID（0-15，对应CAN ID第7-10位）
 * @return 组合后的32位CAN ID
 */

inline uint32_t make_can_id(uint8_t device_id, uint8_t func_id) {
  return ((uint32_t)func_id << FUNC_ID_POS) | device_id;
}

/**
 * @brief CAN ID 辅助工具函数：从 CAN ID 中提取设备ID
 * @param can_id 完整的32位CAN ID
 * @return 提取出的设备ID（0-127）
 */
inline uint8_t get_device_id(uint32_t can_id) {
  return can_id & DEVICE_ID_MSK;
}

/**
 * @brief MotorController 类：电机控制器核心控制类
 * @details 封装了电机的全生命周期控制接口，包括设备初始化、参数配置、实时控制、状态监测等，
 *          通过组合设备ID和功能ID生成CAN指令，实现与电机的通信交互
 */
class MotorController {
  public:
    /**
     * @brief 构造函数：初始化电机控制器
     * @param bus SocketCan 对象指针，用于底层 CAN 总线数据收发
     * @param device_id 目标电机的设备ID（唯一标识一个电机）
     */
    MotorController();
    MotorController(SocketCan *bus, size_t device_id);

    // -------------------------- 基础通信与配置接口 --------------------------
    /**
     * @brief 电机在线检测（Ping）
     * @details 向目标电机发送Ping指令，检测电机是否正常在线
     */
    void ping();

    /**
     * @brief 发送心跳包
     * @details 定期向电机发送心跳指令，维持通信连接，防止电机进入保护停机状态
     */
    void feed();

    /**
     * @brief 设置电机工作模式
     * @param mode 目标工作模式（具体取值参考 motor_controller_conf.h 中的模式定义，如位置模式、速度模式等）
     */
    void set_mode(uint8_t mode);

    /**
     * @brief 从Flash加载配置参数
     * @details 将电机Flash中存储的默认配置参数加载到运行内存，掉电后配置不丢失
     */
    void load_settings_from_flash();

    /**
     * @brief 将配置参数存储到Flash
     * @details 将当前运行内存中的配置参数保存到电机Flash，确保掉电后参数不丢失
     */
    void store_settings_to_flash();

    // -------------------------- 电机参数读写接口（封装底层参数读写函数） --------------------------
    uint32_t read_fast_frame_frequency();        // 读取快速帧频率
    void write_fast_frame_frequency(uint32_t frequency);  // 写入快速帧频率
    float read_gear_ratio();                     // 读取齿轮比
    void write_gear_ratio(float ratio);          // 写入齿轮比
    float read_position_kp();                    // 读取位置环比例系数（KP）
    void write_position_kp(float kp);            // 写入位置环比例系数（KP）
    float read_position_kd();                    // 读取位置环微分系数（KD）
    void write_position_kd(float kd);            // 写入位置环微分系数（KD）
    float read_position_ki();                    // 读取位置环积分系数（KI）
    void write_position_ki(float ki);            // 写入位置环积分系数（KI）
    float read_velocity_kp();                    // 读取速度环比例系数（KP）
    void write_velocity_kp(float kp);            // 写入速度环比例系数（KP）
    float read_velocity_ki();                    // 读取速度环积分系数（KI）
    void write_velocity_ki(float ki);            // 写入速度环积分系数（KI）
    float read_torque_limit();                   // 读取扭矩限制值
    void write_torque_limit(float torque);       // 写入扭矩限制值
    float read_velocity_limit();                 // 读取速度限制值
    void write_velocity_limit(float limit);      // 写入速度限制值
    float read_position_limit_lower();           // 读取位置下限值
    void write_position_limit_lower(float limit); // 写入位置下限值
    float read_position_limit_upper();           // 读取位置上限值
    void write_position_limit_upper(float limit); // 写入位置上限值
    float read_position_offset();                // 读取位置偏移量
    void write_position_offset(float offset);    // 写入位置偏移量
    float read_torque_target();                  // 读取目标扭矩值
    void write_torque_target(float torque);      // 写入目标扭矩值
    float read_torque_measured();                // 读取实际扭矩测量值
    float read_velocity_measured();              // 读取实际速度测量值
    float read_position_target();                // 读取目标位置值
    void write_position_target(float position);  // 写入目标位置值
    float read_position_measured();              // 读取实际位置测量值
    float read_torque_filter_alpha();            // 读取扭矩滤波系数（Alpha）
    void write_torque_filter_alpha(float alpha); // 写入扭矩滤波系数（Alpha）
    float read_current_limit();                  // 读取电流限制值
    void write_current_limit(float current);     // 写入电流限制值
    float read_current_kp();                     // 读取电流环比例系数（KP）
    void write_current_kp(float kp);             // 写入电流环比例系数（KP）
    float read_current_ki();                     // 读取电流环积分系数（KI）
    void write_current_ki(float ki);             // 写入电流环积分系数（KI）
    float read_bus_voltage_filter_alpha();       // 读取母线电压滤波系数（Alpha）
    void write_bus_voltage_filter_alpha(float alpha); // 写入母线电压滤波系数（Alpha）
    float read_motor_torque_constant();          // 读取电机扭矩常数
    void write_motor_torque_constant(float torque_constant); // 写入电机扭矩常数
    int read_motor_phase_order();                // 读取电机相序
    void write_motor_phase_order(int order);     // 写入电机相序
    float read_encoder_velocity_filter_alpha();  // 读取编码器速度滤波系数（Alpha）
    void write_encoder_velocity_filter_alpha(float alpha); // 写入编码器速度滤波系数（Alpha）

    // -------------------------- PDO 数据交互接口（实时控制/状态读取） --------------------------
    /**
     * @brief 写入 PDO2 数据（位置、速度目标值）
     * @details PDO（过程数据对象）用于快速传输实时控制数据，PDO2 传输位置和速度目标值，
     *          数据帧长度为8字节，包含2个32位浮点数（位置目标值、速度目标值）
     */
    void write_pdo_2();

    /**
     * @brief 读取 PDO2 数据（位置、速度测量值）
     * @details 从 CAN 总线读取电机发送的 PDO2 数据帧，解析出实际位置和速度测量值，
     *          并存储到对应的成员变量中
     */
    void read_pdo_2();

    // -------------------------- 带宽自动配置接口（简化参数调试） --------------------------
    /**
     * @brief 自动计算并配置电流环带宽
     * @param bandwidth_hz 目标带宽（单位：Hz）
     * @param phase_resistance 电机相电阻（单位：Ω）
     * @param phase_inductance 电机相电感（单位：H）
     * @details 根据电流环动力学模型，自动计算电流环 KP、KI 系数并写入电机
     */
    void set_current_bandwidth(float bandwidth_hz, float phase_resistance, float phase_inductance);

    /**
     * @brief 自动计算并配置扭矩滤波带宽
     * @param bandwidth_hz 目标带宽（单位：Hz）
     * @param position_loop_rate 位置环更新频率（单位：Hz）
     * @details 根据一阶低通滤波模型，自动计算扭矩滤波 Alpha 系数并写入电机
     */
    void set_torque_bandwidth(float bandwidth_hz, float position_loop_rate);

    /**
     * @brief 自动计算并配置母线电压滤波带宽
     * @param bandwidth_hz 目标带宽（单位：Hz）
     * @param bus_voltage_update_rate 母线电压更新频率（单位：Hz）
     * @details 根据一阶低通滤波模型，自动计算母线电压滤波 Alpha 系数并写入电机
     */
    void set_bus_voltage_bandwidth(float bandwidth_hz, float bus_voltage_update_rate);

    /**
     * @brief 自动计算并配置编码器速度滤波带宽
     * @param bandwidth_hz 目标带宽（单位：Hz）
     * @param encoder_update_rate 编码器更新频率（单位：Hz）
     * @details 根据一阶低通滤波模型，自动计算编码器速度滤波 Alpha 系数并写入电机
     */
    void set_encoder_velocity_bandwidth(float bandwidth_hz, float encoder_update_rate);

    // -------------------------- 状态获取与目标设置接口（对外提供控制接口） --------------------------
    float get_measured_position();  // 获取读取到的实际位置值
    float get_measured_velocity();  // 获取读取到的实际速度值
    void set_target_position(float position);  // 设置位置目标值（供 write_pdo_2 使用）
    void set_target_velocity(float velocity);  // 设置速度目标值（供 write_pdo_2 使用）
    size_t get_ClassDev_id(){
      return device_id;
    }
    void set_encoder_offset(float offset); 
    
    // 也可以顺便加上保存配置的接口，方便后面用
    void save_config();
private:
    // -------------------------- 私有成员变量 --------------------------
    SocketCan *bus;                // CAN 总线通信对象指针，用于底层数据收发
    size_t device_id;              // 目标电机设备ID，唯一标识当前控制的电机

    float position_measured;       // 存储从 PDO2 读取的实际位置测量值（单位：rad 或 mm，根据电机类型）
    float velocity_measured;       // 存储从 PDO2 读取的实际速度测量值（单位：rad/s 或 mm/s）

    float position_target;         // 存储位置目标值，通过 write_pdo_2 发送给电机
    float velocity_target;         // 存储速度目标值，通过 write_pdo_2 发送给电机

    // -------------------------- 底层参数读写函数（私有，供上层接口封装使用） --------------------------
    /**
     * @brief 读取32位浮点型电机参数
     * @param param_id 参数ID（参考 motor_controller_conf.h 中的 Parameter 枚举）
     * @return 读取到的浮点型参数值，读取失败返回0
     */
    float read_parameter_f32(Parameter param_id);

    /**
     * @brief 读取32位有符号整型电机参数
     * @param param_id 参数ID（参考 motor_controller_conf.h 中的 Parameter 枚举）
     * @return 读取到的有符号整型参数值，读取失败返回0
     */
    int32_t read_parameter_i32(Parameter param_id);

    /**
     * @brief 读取32位无符号整型电机参数
     * @param param_id 参数ID（参考 motor_controller_conf.h 中的 Parameter 枚举）
     * @return 读取到的无符号整型参数值，读取失败返回0
     */
    uint32_t read_parameter_u32(Parameter param_id);

    /**
     * @brief 写入32位浮点型电机参数
     * @param param_id 参数ID（参考 motor_controller_conf.h 中的 Parameter 枚举）
     * @param value 要写入的浮点型参数值
     */
    void write_parameter_f32(Parameter param_id, float value);

    /**
     * @brief 写入32位有符号整型电机参数
     * @param param_id 参数ID（参考 motor_controller_conf.h 中的 Parameter 枚举）
     * @param value 要写入的有符号整型参数值
     */
    void write_parameter_i32(Parameter param_id, int32_t value);

    /**
     * @brief 写入32位无符号整型电机参数
     * @param param_id 参数ID（参考 motor_controller_conf.h 中的 Parameter 枚举）
     * @param value 要写入的无符号整型参数值
     */
    void write_parameter_u32(Parameter param_id, uint32_t value);
};

