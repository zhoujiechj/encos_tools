#ifndef __MOTOR_CONTROL_H
#define __MOTOR_CONTROL_H

#include <cstdint>
#include <cstring>
#include <iostream>
#include <vector>
#include <map>
#include "config.h"
#include "encos_driver/math_ops.h"

constexpr uint8_t comm_ack = 0x00;
constexpr uint8_t comm_auto = 0x01;

enum class MotorInsType
{
    INS_NONE = 0x00,
    INS_GET_ID = 0x01,      // 指令 - 获取ID
    INS_SET_ID = 0x02,      // 指令 - 设置ID
    INS_RESET_ID = 0x03,    // 指令 - 重置ID
    INS_SET_ZERO = 0x04,    // 指令 - 设置零点
    INS_GET_PARAM = 0x05,   // 指令 - 获取参数
    INS_SET_PARAM = 0x06,   // 指令 - 设置参数
    INS_SET_TOR_POS = 0x07, // 指令 - 设置力位混控
    INS_SET_POS = 0x08,     // 指令 - 设置位置控制
    INS_SET_SPD = 0x09,     // 指令 - 设置速度控制
    INS_SET_TOR_CUR = 0x0A, // 指令 - 设置力矩电流控制
};

enum class MotorParamType
{
    PARAM_NONE = 0,
    PARAM_POS = 1,             // 参数 - 当前位置
    PARAM_SPD = 2,             // 参数 - 当前速度
    PARAM_CUR = 3,             // 参数 - 当前电流
    PARAM_PWR = 4,             // 参数 - 电机功率
    PARAM_ACC = 5,             // 参数 - 加速度
    PARAM_FLUX_GAIN = 6,       // 参数 - 磁链观测增益
    PARAM_DC_COE = 7,          // 参数 - 扰动补偿系数
    PARAM_FEEDBACK_GAIN = 8,   // 参数 - 反馈补偿增益
    PARAM_DAMP_COE = 9,        // 参数 - 阻尼系数
    PARAM_TOR_COE = 22,        // 参数 - 扭矩系数
    PARAM_TOR_POS_KP = 23,     // 参数 - 力位混控协议KP范围
    PARAM_TOR_POS_KD = 24,     // 参数 - 力位混控协议KD范围
    PARAM_TOR_POS_POS = 25,    // 参数 - 力位混控协议POS范围
    PARAM_TOR_POS_SPD = 26,    // 参数 - 力位混控协议SPD范围
    PARAM_TOR_POS_TOR = 27,    // 参数 - 力位混控协议TOR范围
    PARAM_TOR_POS_CUR = 28,    // 参数 - 力位混控协议CUR范围
    PARAM_CAN_TIMEOUT = 31,    // 参数 - CAN超时时间
    PARAM_CUR_LOOP_KP_KI = 32, // 参数 - 电流环 KP 和 KI
    PARAM_SPD_LOOP_KP_KI = 33, // 参数 - 速度环 KP 和 KI
    PARAM_POS_LOOP_KP_KD = 34, // 参数 - 位置环 KP 和 KD
    PARAM_TOR_COE_KT = 35,     // 参数 - 扭矩系数 Kt 校准使能状态
};

enum class MotorErrType
{
    ERR_NONE = 0,          // 无错误
    ERR_HEAT_OVER = 1,     // 电机过热
    ERR_CUR_OVER = 2,      // 电机过流
    ERR_VOL_OVER_HIGH = 3, // 电机电压过低
    ERR_VOL_OVER_LOW = 4,  // 电机电压过低
    ERR_ENCODER = 5,       // 电机编码器错误
    ERR_DRIVER_DRV = 7,    // DRV 驱动错误
};

struct MotorCommFbd
{
    uint16_t motor_id; // 电机ID
    uint8_t ins_code;  // 配置代码
    uint8_t motor_fbd; // 电机反馈
};

struct ODMotorMsg
{
    uint16_t motor_id;

    float angle_target;   // 目标角度 - (单位：rad 或 °)
    float speed_target;   // 目标速度 - (单位：rad/s 或 rpm)
    float torque_target;  // 目标扭矩 - (Nm)
    float current_target; // 目标电流 - (A)

    uint16_t angle_actual_int;  // 实际角度 - 整型
    float angle_actual;         // 实际角度 - (单位：°)
    float angle_actual_rad;     // 实际角度 - (单位：rad)
    int16_t speed_actual_int;   // 实际速度 - 整型
    float speed_actual;         // 实际速度 - (单位：rpm)
    float speed_actual_rad;     // 实际速度 - (单位：rad/s)
    int16_t current_actual_int; // 实际电流 - 整型
    float current_actual;       // 实际电流 - (单位：A)

    float power;        // 功率 (单位：W)
    uint8_t temp_motor; // 电机温度 (单位：℃)
    uint8_t temp_mos;   // MOS温度 (单位：℃)

    float acc;              // 加速度 (单位：rad/s)
    uint16_t flux_gain;     // 磁链观测增益
    uint16_t dc_coe;        // 扰动补偿系数
    uint16_t feedback_gain; // 反馈补偿增益
    uint16_t damp_coe;      // 阻尼系数
    float tor_coe;          // 扭矩系数
    float kp_min;           // 力位混控协议KP范围的最小值
    float kp_max;           // 力位混控协议KP范围的最大值
    float kp_target;        // 力位混控协议KP
    float kd_min;           // 力位混控协议KD范围的最小值
    float kd_max;           // 力位混控协议KD范围的最大值
    float kd_target;        // 力位混控协议KD
    float pos_min;          // 力位混控协议POS范围的最小值
    float pos_max;          // 力位混控协议POS范围的最大值
    float spd_min;          // 力位混控协议SPD范围的最小值
    float spd_max;          // 力位混控协议SPD范围的最大值
    float tor_min;          // 力位混控协议TOR范围的最小值
    float tor_max;          // 力位混控协议TOR范围的最大值
    float cur_min;          // 力位混控协议CUR范围的最小值
    float cur_max;          // 力位混控协议CUR范围的最大值
    float cur_loop_kp;      // 电流环KP
    float cur_loop_ki;      // 电流环KI
    float spd_loop_kp;      // 速度环KP
    float spd_loop_ki;      // 速度环KI
    float pos_loop_kp;      // 位置环KP
    float pos_loop_kd;      // 位置环KD
    bool tor_coe_kt_enable; // 扭矩系数 Kt 校准使能状态

    float can_timeout; // CAN超时时间

    uint8_t error; // 错误信息
};

class MotorControl
{
public:
    MotorControl(int motor_num);
    ~MotorControl() = default;

    // 读取电机ID
    void get_motor_id(EtherCAT_Msg *TxMessage);
    // 设置电机新ID
    void set_motor_id(EtherCAT_Msg *TxMessage, uint16_t motor_id, uint16_t motor_id_new);
    // 重置电机ID
    void reset_motor_id(EtherCAT_Msg *TxMessage);
    // 设置电机零点
    void set_motor_zero(EtherCAT_Msg *TxMessage, uint8_t passage, uint16_t motor_id);
    // 读取电机通信模式
    void get_motor_comm_mode(EtherCAT_Msg *TxMessage, uint16_t motor_id);
    // 清除电机发送缓冲区
    void clear_motor_buff(EtherCAT_Msg *TxMessage, uint8_t passage);
    // 查询电机参数
    void get_motor_param(EtherCAT_Msg *TxMessage, uint8_t passage, uint16_t motor_id, uint8_t code);
    // 设置电机加速度
    void set_motor_acc(EtherCAT_Msg *TxMessage, uint8_t passage, uint16_t motor_id, float acc, uint8_t ack_status);
    // 设置电机力位混控协议KP配置
    void set_motor_tor_pos_config_kp(EtherCAT_Msg *TxMessage, uint8_t passage, uint16_t motor_id, uint16_t kp_min, uint16_t kp_max);
    // 设置电机力位混控协议KD配置
    void set_motor_tor_pos_config_kd(EtherCAT_Msg *TxMessage, uint8_t passage, uint16_t motor_id, uint16_t kd_min, uint16_t kd_max);
    // 设置电机力位混控协议位置配置
    void set_motor_tor_pos_config_pos(EtherCAT_Msg *TxMessage, uint8_t passage, uint16_t motor_id, float pos_min, float pos_max);
    // 设置电机力位混控协议速度配置
    void set_motor_tor_pos_config_spd(EtherCAT_Msg *TxMessage, uint8_t passage, uint16_t motor_id, float spd_min, float spd_max);
    // 设置电机力位混控协议扭矩配置
    void set_motor_tor_pos_config_tor(EtherCAT_Msg *TxMessage, uint8_t passage, uint16_t motor_id, float tor_min, float tor_max);
    // 设置电机力位混控协议电流配置
    void set_motor_tor_pos_config_cur(EtherCAT_Msg *TxMessage, uint8_t passage, uint16_t motor_id, float cur_min, float cur_max);
    // 设置电机CAN超时时间
    void set_motor_can_timeout(EtherCAT_Msg *TxMessage, uint8_t passage, uint16_t motor_id, uint16_t timeout);
    // 设置电流环PI
    void set_motor_current_loop_pi(EtherCAT_Msg *TxMessage, uint8_t passage, uint16_t motor_id, float kp, float ki);
    // 设置速度环PI
    void set_motor_speed_loop_pi(EtherCAT_Msg *TxMessage, uint8_t passage, uint16_t motor_id, float kp, float ki);
    // 设置位置环PD
    void set_motor_position_loop_pd(EtherCAT_Msg *TxMessage, uint8_t passage, uint16_t motor_id, float kp, float kd);
    // 设置电机扭矩系数 Kt 校准使能
    void set_motor_kt_enable(EtherCAT_Msg *TxMessage, uint8_t passage, uint16_t motor_id, bool is_enable);

    // 设置力位混合模式
    void set_motor_tor_pos(EtherCAT_Msg *TxMessage, uint8_t passage, uint16_t motor_id, float kp, float kd, float pos, float spd, float tor);
    // 设置伺服位置控制模式
    void set_motor_pos(EtherCAT_Msg *TxMessage, uint8_t passage, uint16_t motor_id, float pos, float spd, float cur, uint8_t ack_status);
    // 设置伺服速度控制模式
    void set_motor_speed(EtherCAT_Msg *TxMessage, uint8_t passage, uint16_t motor_id, float spd, float cur, uint8_t ack_status);
    // 设置电流力矩控制模式
    void set_motor_tor_cur(EtherCAT_Msg *TxMessage, uint8_t passage, uint16_t motor_id, int16_t cur_tor, uint8_t ctrl_status, uint8_t ack_status);

    // 解析CAN数据
    uint8_t rv_can_data_repack(EtherCAT_Msg *RxMessage, uint8_t comm_mode, uint8_t slave_id);
    // 打印CAN数据
    void rv_message_print(uint8_t ack_status);

public:
    MotorCommFbd motor_comm_fbd;           // 电机反馈数据
    std::vector<ODMotorMsg> rv_motor_msg; // 电机反馈数据

private:
    union RVTypeConvert
    {
        float to_float;
        int to_int;
        unsigned int to_uint;
        uint8_t buf[4];
    } rv_type_convert;

    union RVTypeConvert2
    {
        int16_t to_int16;
        uint16_t to_uint16;
        uint8_t buf[2];
    } rv_type_convert2;

    MotorInsType motor_ins_type;     // 电机指令类型
    MotorParamType motor_param_type; // 电机参数类型
    uint16_t motor_id_check; // 电机ID检查
    MathOps math_ops_; // 数学运算工具

    int motor_num_;
};

#endif