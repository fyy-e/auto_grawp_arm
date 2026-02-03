// Copyright (c) 2025, The Berkeley Humanoid Lite Project Developers.
#pragma once

/**
 * @file controller_config.h
 * @brief 伯克利人形机器人轻量版 电机控制器核心配置头文件
 * @details 定义控制器固件版本、工作模式、错误码、CAN总线通信协议、全量控制参数ID，
 *          适用于多关节伺服电机控制器，遵循类CANopen通信规范，支持位置/速度/扭矩/电流多闭环控制，
 *          所有枚举值均为无符号类型，参数ID按4字节步长分配（适配32位数据读写）。
 * @note 硬件平台基于STM32系列MCU，支持CAN总线、I2C、ADC/PWM等外设，适用于人形机器人关节驱动。
 */

/** ======== 控制器基础配置 (Controller Settings) ======== **/
/**
 * @brief 固件版本定义规则
 * 原规则：32位十六进制，格式为 (主版本[7:4]).(次版本[3:2]).(补丁版本[1:0])，例：0x00010005 = V1.0.5
 * 当前规则：项目定制化日期编码，格式为 YYYYmmdd（年/月/日），便于快速追溯固件编译时间
 */
// 固件版本：2025年02月26日编译
#define FIRMWARE_VERSION                0x20250226


/** ======== 控制器状态定义 (Controller State Definitions) ======== **/
/**
 * @brief 控制器工作模式枚举
 * @details 按「安全等级+控制类型」分类，8位无符号整型，不同模式对应不同的硬件输出使能状态和控制逻辑，
 *          模式切换需满足安全条件（无故障、急停释放），部分模式为过渡态/特殊功能态。
 * @note 安全模式优先级最高，故障时会自动切至MODE_DISABLED
 */
typedef enum {
  // 安全模式（3种，无硬件输出/低功耗，故障默认切至此类别）
  MODE_DISABLED                   = 0x00U,  ///< 禁用模式：所有功率输出关闭，控制器仅保留基础通信，最高安全等级
  MODE_IDLE                       = 0x01U,  ///< 空闲模式：操作态，硬件使能但无控制指令，电机处于自由状态

  // 特殊模式（2种，非常规运动控制，用于故障处理/设备校准）
  MODE_DAMPING                    = 0x02U,  ///< 阻尼模式：电机锁定，提供被动阻尼力，用于设备停止/防飘
  MODE_CALIBRATION                = 0x05U,  ///< 校准模式：编码器零点/电机参数校准专用，禁止外部运动指令

  // 闭环控制模式（4种，核心运动控制，基于PID串级控制，精度由高到低：位置>速度>扭矩>电流）
  MODE_CURRENT                    = 0x10U,  ///< 电流闭环：最底层闭环，直接控制电机相电流，适用于高精度扭矩控制
  MODE_TORQUE                     = 0x11U,  ///< 扭矩闭环：基于电流环，控制电机输出扭矩，适用于力控场景
  MODE_VELOCITY                   = 0x12U,  ///< 速度闭环：基于扭矩环，控制电机转速，适用于恒速场景
  MODE_POSITION                   = 0x13U,  ///< 位置闭环：最外层串级闭环（位置-速度-扭矩-电流），适用于精确定位

  // 开环控制模式（3种，直接控制电机电压，无反馈校正，仅用于调试/特殊测试）
  MODE_VABC_OVERRIDE              = 0x20U,  ///< 开环：直接设置ABC三相电压，无闭环校正
  MODE_VALPHABETA_OVERRIDE        = 0x21U,  ///< 开环：直接设置αβ坐标系电压（Clark变换后），无闭环校正
  MODE_VQD_OVERRIDE               = 0x22U,  ///< 开环：直接设置dq坐标系电压（Park变换后），无闭环校正

  MODE_DEBUG                      = 0x80U,  ///< 调试模式：预操作态，仅开放数据采集/参数读取，禁止硬件输出
} Mode;

/**
 * @brief 控制器错误码枚举
 * @details 16位无符号位掩码（Bit0~Bit13），每一位对应一个独立故障，支持**多故障叠加**，
 *          故障清除后对应位清零，Bit14~Bit15保留未使用，故障发生时控制器会触发相应安全动作（如切禁用模式）。
 * @note 可通过CAN总线读取错误码，快速定位硬件/软件/通信故障
 */
typedef enum {
  ERROR_NO_ERROR                  = 0b0000000000000000U,  ///< 无故障：所有位清零，控制器正常工作
  ERROR_GENERAL                   = 0b0000000000000001U,  ///< 通用故障：未定义的未知故障
  ERROR_ESTOP                     = 0b0000000000000010U,  ///< 急停故障：硬件/软件急停信号触发，安全等级最高
  ERROR_INITIALIZATION_ERROR      = 0b0000000000000100U,  ///< 初始化故障：控制器启动时外设/参数初始化失败
  ERROR_CALIBRATION_ERROR         = 0b0000000000001000U,  ///< 校准故障：编码器/电机参数校准超时/失败
  ERROR_POWERSTAGE_ERROR          = 0b0000000000010000U,  ///< 功率板故障：MOS管/驱动芯片损坏或未检测到
  ERROR_INVALID_MODE              = 0b0000000000100000U,  ///< 无效模式：尝试切换至未定义/不支持的工作模式
  ERROR_WATCHDOG_TIMEOUT          = 0b0000000001000000U,  ///< 看门狗超时：主程序卡死，未及时喂狗
  ERROR_OVER_VOLTAGE              = 0b0000000010000000U,  ///< 过压故障：电源总线电压超过阈值
  ERROR_OVER_CURRENT              = 0b0000000100000000U,  ///< 过流故障：电机相电流/总线电流超过阈值
  ERROR_OVER_TEMPERATURE          = 0b0000001000000000U,  ///< 过温故障：电机/功率板/MCU温度超过阈值
  ERROR_CAN_RX_FAULT              = 0b0000010000000000U,  ///< CAN接收故障：CAN总线接收超时/帧错误/溢出
  ERROR_CAN_TX_FAULT              = 0b0000100000000000U,  ///< CAN发送故障：CAN总线发送超时/仲裁失败
  ERROR_I2C_FAULT                 = 0b0001000000000000U,  ///< I2C故障：I2C总线通信失败（编码器/传感器通信异常）
} ErrorCode;

/** ======== CAN总线报文定义 (CAN Packet Definitions) ======== **/
/**
 * @brief CAN帧功能码枚举
 * @details 4位无符号整型（Bit0~Bit3），定义CAN总线帧的**功能类型**，遵循**类CANopen协议规范**，
 *          用于区分CAN帧的用途（如过程数据传输、参数配置、心跳检测），与设备ID组合构成CAN帧ID。
 * @note PDO（过程数据对象）用于**实时高速**数据传输（如控制指令、状态采集），SDO（服务数据对象）用于**非实时**参数读写
 */
typedef enum {
  FUNC_NMT                      = 0b0000U,  ///< 网络管理：设备启动/停止/复位等网络指令
  FUNC_SYNC_EMCY                = 0b0001U,  ///< 同步/紧急：总线同步帧、故障紧急上报帧
  FUNC_TIME                     = 0b0010U,  ///< 时间同步：多设备时钟同步帧
  FUNC_TRANSMIT_PDO_1           = 0b0011U,  ///< 发送PDO1：控制器→上位机，传输核心状态（位置/速度/扭矩测量值）
  FUNC_RECEIVE_PDO_1            = 0b0100U,  ///< 接收PDO1：上位机→控制器，传输核心指令（位置/速度/扭矩目标值）
  FUNC_TRANSMIT_PDO_2           = 0b0101U,  ///< 发送PDO2：控制器→上位机，传输次要状态（电流/电压/温度）
  FUNC_RECEIVE_PDO_2            = 0b0110U,  ///< 接收PDO2：上位机→控制器，传输次要指令（控制参数临时修改）
  FUNC_TRANSMIT_PDO_3           = 0b0111U,  ///< 发送PDO3：控制器→上位机，传输调试数据（积分器值/滤波器值）
  FUNC_RECEIVE_PDO_3            = 0b1000U,  ///< 接收PDO3：上位机→控制器，传输调试指令
  FUNC_TRANSMIT_PDO_4           = 0b1001U,  ///< 发送PDO4：控制器→上位机，传输预留状态数据
  FUNC_RECEIVE_PDO_4            = 0b1010U,  ///< 接收PDO4：上位机→控制器，传输预留指令数据
  FUNC_TRANSMIT_SDO             = 0b1011U,  ///< 发送SDO：控制器→上位机，非实时参数读取响应
  FUNC_RECEIVE_SDO              = 0b1100U,  ///< 接收SDO：上位机→控制器，非实时参数写入/读取请求
  FUNC_FLASH                    = 0b1101U,  ///< 固件烧录：控制器固件在线升级/参数保存至Flash
  FUNC_HEARTBEAT                = 0b1110U,  ///< 心跳检测：控制器定时发送心跳帧，上位机检测设备在线状态
} FrameFunction;

/**
 * @brief CAN总线参数ID枚举
 * @details 16位无符号整型，**按4字节步长连续分配**（0x000~0x140），每个ID对应一个32位可配置/可读取参数，
 *          按**功能模块分组**（设备基础/位置控制器/电流控制器/功率板/电机/编码器），支持SDO协议读写、PDO协议实时采集。
 * @par 读写属性说明
 * - RW：可读写（支持上位机配置，掉电后可保存至Flash）
 * - R：只读（仅控制器内部计算/采集，上位机不可修改）
 * - W：只写（仅上位机下发指令，控制器不回传）
 * @note 所有浮点型参数均为32位float，整型参数为32位uint32_t/int32_t，统一按4字节传输
 */
typedef enum {
  // -------------------------- 设备基础参数 (RW) --------------------------
  PARAM_DEVICE_ID                                       = 0x000U,  ///< 设备ID：控制器唯一标识，多设备组网区分
  PARAM_FIRMWARE_VERSION                                = 0x004U,  ///< 固件版本：读取当前固件版本（与宏定义FIRMWARE_VERSION一致）
  PARAM_WATCHDOG_TIMEOUT                                = 0x008U,  ///< 看门狗超时时间(ms)：主程序喂狗周期，超时触发复位
  PARAM_FAST_FRAME_FREQUENCY                            = 0x00CU,  ///< 快速帧频率(Hz)：PDO实时帧的发送频率（如100/200/500Hz）
  PARAM_MODE                                            = 0x010U,  ///< 当前工作模式：读取/切换控制器Mode
  PARAM_ERROR                                           = 0x014U,  ///< 当前错误码：读取控制器ErrorCode（位掩码）
  PARAM_POSITION_CONTROLLER_UPDATE_COUNTER              = 0x018U,  ///< 位置控制器更新计数器：闭环控制周期计数，用于检测控制正常性

  // -------------------------- 位置控制器参数 (串级闭环：位置-速度-扭矩) --------------------------
  PARAM_POSITION_CONTROLLER_GEAR_RATIO                  = 0x01CU,  ///< 减速比 (RW)：电机与关节的减速比，用于位置/速度换算
  PARAM_POSITION_CONTROLLER_POSITION_KP                 = 0x020U,  ///< 位置环KP (RW)：位置比例系数，核心调参项
  PARAM_POSITION_CONTROLLER_POSITION_KI                 = 0x024U,  ///< 位置环KI (RW)：位置积分系数，消除静差
  PARAM_POSITION_CONTROLLER_VELOCITY_KP                 = 0x028U,  ///< 速度环KP (RW)：速度比例系数
  PARAM_POSITION_CONTROLLER_VELOCITY_KI                 = 0x02CU,  ///< 速度环KI (RW)：速度积分系数
  PARAM_POSITION_CONTROLLER_TORQUE_LIMIT                = 0x030U,  ///< 扭矩限制 (RW)：最大输出扭矩，防止过载
  PARAM_POSITION_CONTROLLER_VELOCITY_LIMIT              = 0x034U,  ///< 速度限制 (RW)：最大输出转速，防止飞车
  PARAM_POSITION_CONTROLLER_POSITION_LIMIT_LOWER        = 0x038U,  ///< 位置下限 (RW)：关节最小位置限制，超限位触发保护
  PARAM_POSITION_CONTROLLER_POSITION_LIMIT_UPPER        = 0x03CU,  ///< 位置上限 (RW)：关节最大位置限制，超限位触发保护
  PARAM_POSITION_CONTROLLER_POSITION_OFFSET             = 0x040U,  ///< 位置偏移 (RW)：编码器零点偏移，用于校准
  PARAM_POSITION_CONTROLLER_TORQUE_TARGET               = 0x044U,  ///< 扭矩目标值 (W)：上位机下发的扭矩指令
  PARAM_POSITION_CONTROLLER_TORQUE_MEASURED             = 0x048U,  ///< 扭矩测量值 (R)：控制器实时采集的实际输出扭矩
  PARAM_POSITION_CONTROLLER_TORQUE_SETPOINT             = 0x04CU,  ///< 扭矩设定值 (R)：位置环计算后下发给扭矩环的指令
  PARAM_POSITION_CONTROLLER_VELOCITY_TARGET             = 0x050U,  ///< 速度目标值 (W)：上位机下发的速度指令
  PARAM_POSITION_CONTROLLER_VELOCITY_MEASURED           = 0x054U,  ///< 速度测量值 (R)：编码器采集的实际转速
  PARAM_POSITION_CONTROLLER_VELOCITY_SETPOINT           = 0x058U,  ///< 速度设定值 (R)：位置环计算后下发给速度环的指令
  PARAM_POSITION_CONTROLLER_POSITION_TARGET             = 0x05CU,  ///< 位置目标值 (W)：上位机下发的位置指令
  PARAM_POSITION_CONTROLLER_POSITION_MEASURED           = 0x060U,  ///< 位置测量值 (R)：编码器采集的实际位置
  PARAM_POSITION_CONTROLLER_POSITION_SETPOINT           = 0x064U,  ///< 位置设定值 (R)：位置环PID计算后的输出值
  PARAM_POSITION_CONTROLLER_POSITION_INTEGRATOR         = 0x068U,  ///< 位置环积分值 (R)：PID积分器累计值，用于调试
  PARAM_POSITION_CONTROLLER_VELOCITY_INTEGRATOR         = 0x06CU,  ///< 速度环积分值 (R)：PID积分器累计值，用于调试
  PARAM_POSITION_CONTROLLER_TORQUE_FILTER_ALPHA         = 0x070U,  ///< 扭矩滤波系数 (RW)：一阶低通滤波系数（0~1），平滑扭矩输出

  // -------------------------- 电流控制器参数 (最底层闭环，直接控制电机) --------------------------
  PARAM_CURRENT_CONTROLLER_I_LIMIT                      = 0x074U,  ///< 电流限制 (RW)：电机相电流最大限制，防止过流
  PARAM_CURRENT_CONTROLLER_I_KP                         = 0x078U,  ///< 电流环KP (RW)：dq轴电流比例系数
  PARAM_CURRENT_CONTROLLER_I_KI                         = 0x07CU,  ///< 电流环KI (RW)：dq轴电流积分系数
  PARAM_CURRENT_CONTROLLER_I_A_MEASURED                 = 0x080U,  ///< A相电流测量值 (R)：ADC采集的A相实际电流
  PARAM_CURRENT_CONTROLLER_I_B_MEASURED                 = 0x084U,  ///< B相电流测量值 (R)：ADC采集的B相实际电流
  PARAM_CURRENT_CONTROLLER_I_C_MEASURED                 = 0x088U,  ///< C相电流测量值 (R)：ADC采集的C相实际电流
  PARAM_CURRENT_CONTROLLER_V_A_SETPOINT                 = 0x08CU,  ///< A相电压设定值 (R)：电流环计算后的A相输出电压
  PARAM_CURRENT_CONTROLLER_V_B_SETPOINT                 = 0x090U,  ///< B相电压设定值 (R)：电流环计算后的B相输出电压
  PARAM_CURRENT_CONTROLLER_V_C_SETPOINT                 = 0x094U,  ///< C相电压设定值 (R)：电流环计算后的C相输出电压
  PARAM_CURRENT_CONTROLLER_I_ALPHA_MEASURED             = 0x098U,  ///< α轴电流测量值 (R)：Clark变换后的α轴电流
  PARAM_CURRENT_CONTROLLER_I_BETA_MEASURED              = 0x09CU,  ///< β轴电流测量值 (R)：Clark变换后的β轴电流
  PARAM_CURRENT_CONTROLLER_V_ALPHA_SETPOINT             = 0x0A0U,  ///< α轴电压设定值 (R)：电流环计算后的α轴输出电压
  PARAM_CURRENT_CONTROLLER_V_BETA_SETPOINT              = 0x0A4U,  ///< β轴电压设定值 (R)：电流环计算后的β轴输出电压
  PARAM_CURRENT_CONTROLLER_V_Q_TARGET                   = 0x0A8U,  ///< q轴电压目标值 (W)：开环模式下直接下发的q轴电压
  PARAM_CURRENT_CONTROLLER_V_D_TARGET                   = 0x0ACU,  ///< d轴电压目标值 (W)：开环模式下直接下发的d轴电压
  PARAM_CURRENT_CONTROLLER_V_Q_SETPOINT                 = 0x0B0U,  ///< q轴电压设定值 (R)：电流环计算后的q轴输出电压
  PARAM_CURRENT_CONTROLLER_V_D_SETPOINT                 = 0x0B4U,  ///< d轴电压设定值 (R)：电流环计算后的d轴输出电压
  PARAM_CURRENT_CONTROLLER_I_Q_TARGET                   = 0x0B8U,  ///< q轴电流目标值 (W)：扭矩环下发的q轴电流指令
  PARAM_CURRENT_CONTROLLER_I_D_TARGET                   = 0x0BCU,  ///< d轴电流目标值 (W)：扭矩环下发的d轴电流指令
  PARAM_CURRENT_CONTROLLER_I_Q_MEASURED                 = 0x0C0U,  ///< q轴电流测量值 (R)：Park变换后的q轴实际电流
  PARAM_CURRENT_CONTROLLER_I_D_MEASURED                 = 0x0C4U,  ///< d轴电流测量值 (R)：Park变换后的d轴实际电流
  PARAM_CURRENT_CONTROLLER_I_Q_SETPOINT                 = 0x0C8U,  ///< q轴电流设定值 (R)：电流环PID计算后的q轴输出
  PARAM_CURRENT_CONTROLLER_I_D_SETPOINT                 = 0x0CCU,  ///< d轴电流设定值 (R)：电流环PID计算后的d轴输出
  PARAM_CURRENT_CONTROLLER_I_Q_INTEGRATOR               = 0x0D0U,  ///< q轴积分值 (R)：q轴电流PID积分器累计值
  PARAM_CURRENT_CONTROLLER_I_D_INTEGRATOR               = 0x0D4U,  ///< d轴积分值 (R)：d轴电流PID积分器累计值

  // -------------------------- 功率板参数 (电源/驱动/采集) --------------------------
  PARAM_POWERSTAGE_HTIM                                 = 0x0D8U,  ///< PWM定时器句柄 (RW)：硬件定时器ID，用于生成PWM波
  PARAM_POWERSTAGE_HADC1                                = 0x0DCU,  ///< ADC1句柄 (RW)：电流/电压采集ADC1 ID
  PARAM_POWERSTAGE_HADC2                                = 0x0E0U,  ///< ADC2句柄 (RW)：温度/备用采集ADC2 ID
  PARAM_POWERSTAGE_ADC_READING_RAW                      = 0x0E4U,  ///< ADC原始值 (R)：未校准的ADC采集原始数据
  PARAM_POWERSTAGE_ADC_READING_OFFSET                   = 0x0ECU,  ///< ADC偏移值 (RW)：ADC校准偏移量，消除零点漂移
  PARAM_POWERSTAGE_UNDERVOLTAGE_THRESHOLD               = 0x0F4U,  ///< 欠压阈值 (RW)：总线欠压保护阈值（低于此值触发保护）
  PARAM_POWERSTAGE_OVERVOLTAGE_THRESHOLD                = 0x0F8U,  ///< 过压阈值 (RW)：总线过压保护阈值（高于此值触发保护）
  PARAM_POWERSTAGE_BUS_VOLTAGE_FILTER_ALPHA             = 0x0FCU,  ///< 总线电压滤波系数 (RW)：一阶低通滤波系数（0~1）
  PARAM_POWERSTAGE_BUS_VOLTAGE_MEASURED                 = 0x100U,  ///< 总线电压测量值 (R)：ADC采集的电源总线实际电压

  // -------------------------- 电机参数 (电机固有属性/校准) --------------------------
  PARAM_MOTOR_POLE_PAIRS                                = 0x104U,  ///< 电机极对数 (RW)：永磁同步电机的极对数，用于转速换算
  PARAM_MOTOR_TORQUE_CONSTANT                           = 0x108U,  ///< 扭矩常数 (RW)：电机扭矩/电流比值（Nm/A），核心参数
  PARAM_MOTOR_PHASE_ORDER                               = 0x10CU,  ///< 电机相序 (RW)：ABC相序配置，0=正序，1=反序
  PARAM_MOTOR_MAX_CALIBRATION_CURRENT                   = 0x110U,  ///< 最大校准电流 (RW)：电机校准过程中的最大限制电流

  // -------------------------- 编码器参数 (位置/速度采集) --------------------------
  PARAM_ENCODER_HI2C                                    = 0x114U,  ///< 编码器I2C句柄 (RW)：编码器通信I2C总线ID
  PARAM_ENCODER_I2C_BUFFER                              = 0x118U,  ///< I2C缓冲区 (R)：编码器I2C通信的接收/发送缓冲区
  PARAM_ENCODER_I2C_UPDATE_COUNTER                      = 0x11CU,  ///< I2C更新计数器 (R)：编码器数据采集周期计数
  PARAM_ENCODER_CPR                                     = 0x120U,  ///< 编码器线数 (RW)：编码器每转脉冲数，用于位置换算
  PARAM_ENCODER_POSITION_OFFSET                         = 0x124U,  ///< 编码器位置偏移 (RW)：编码器零点偏移，用于校准
  PARAM_ENCODER_VELOCITY_FILTER_ALPHA                   = 0x128U,  ///< 速度滤波系数 (RW)：编码器速度采集滤波系数（0~1）
  PARAM_ENCODER_POSITION_RAW                            = 0x12CU,  ///< 编码器原始位置 (R)：未校准的编码器原始位置值
  PARAM_ENCODER_N_ROTATIONS                             = 0x130U,  ///< 编码器旋转圈数 (R)：编码器累计旋转圈数，用于多圈位置
  PARAM_ENCODER_POSITION                                = 0x134U,  ///< 编码器实际位置 (R)：校准后的多圈绝对位置
  PARAM_ENCODER_VELOCITY                                = 0x138U,  ///< 编码器实际速度 (R)：编码器计算的实际转速（rpm/rad/s）
  PARAM_ENCODER_FLUX_OFFSET                             = 0x13CU,  ///< 磁链偏移 (RW)：永磁同步电机磁链校准偏移量
  PARAM_ENCODER_FLUX_OFFSET_TABLE                       = 0x140U,  ///< 磁链偏移表 (RW)：电机磁链校准的查表数据
} Parameter;
