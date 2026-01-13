#include "encos_driver/motor_control.h"

enum ParamIndex {
    KP_MIN = 0,
    KP_MAX,
    KD_MIN,
    KD_MAX,
    POS_MIN,
    POS_MAX,
    SPD_MIN,
    SPD_MAX,
    TOR_MIN,
    TOR_MAX,
    CUR_MIN,
    CUR_MAX,
    PARAM_COUNT
};

std::map<int, std::vector<float>> g_param_map {
    {1, {0.0, 500.0, 0.0, 5.0, -12.5, 12.5, -18.0, 18.0, -150.0, 150.0, -70.0, 70.0}},
    {2, {0.0, 500.0, 0.0, 5.0, -12.5, 12.5, -18.0, 18.0, -60.0, 60.0, -60.0, 60.0}},
    {3, {0.0, 500.0, 0.0, 5.0, -12.5, 12.5, -18.0, 18.0, -60.0, 60.0, -60.0, 60.0}},
    {4, {0.0, 500.0, 0.0, 50.0, -12.5, 12.5, -18.0, 18.0, -150.0, 150.0, -70.0, 70.0}},
    {5, {0.0, 500.0, 0.0, 5.0, -12.5, 12.5, -18.0, 18.0, -30.0, 30.0, -30.0, 30.0}},
    {6, {0.0, 500.0, 0.0, 5.0, -12.5, 12.5, -18.0, 18.0, -30.0, 30.0, -30.0, 30.0}},
    {7, {0.0, 500.0, 0.0, 5.0, -12.5, 12.5, -18.0, 18.0, -150.0, 150.0, -70.0, 70.0}},
    {8, {0.0, 500.0, 0.0, 5.0, -12.5, 12.5, -18.0, 18.0, -60.0, 60.0, -60.0, 60.0}},
    {9, {0.0, 500.0, 0.0, 5.0, -12.5, 12.5, -18.0, 18.0, -60.0, 60.0, -60.0, 60.0}},
    {10, {0.0, 500.0, 0.0, 50.0, -12.5, 12.5, -18.0, 18.0, -150.0, 150.0, -70.0, 70.0}},
    {11, {0.0, 500.0, 0.0, 5.0, -12.5, 12.5, -18.0, 18.0, -30.0, 30.0, -30.0, 30.0}},
    {12, {0.0, 500.0, 0.0, 5.0, -12.5, 12.5, -18.0, 18.0, -30.0, 30.0, -30.0, 30.0}},
    {13, {0.0, 500.0, 0.0, 5.0, -12.5, 12.5, -18.0, 18.0, -30.0, 30.0, -30.0, 30.0}},
    {14, {0.0, 500.0, 0.0, 5.0, -12.5, 12.5, -18.0, 18.0, -30.0, 30.0, -30.0, 30.0}},
    {15, {0.0, 500.0, 0.0, 5.0, -12.5, 12.5, -18.0, 18.0, -30.0, 30.0, -30.0, 30.0}},
    {16, {0.0, 500.0, 0.0, 5.0, -12.5, 12.5, -18.0, 18.0, -30.0, 30.0, -30.0, 30.0}},
    {17, {0.0, 500.0, 0.0, 5.0, -12.5, 12.5, -18.0, 18.0, -30.0, 30.0, -30.0, 30.0}},
    {18, {0.0, 500.0, 0.0, 5.0, -12.5, 12.5, -18.0, 18.0, -30.0, 30.0, -30.0, 30.0}},
    {19, {0.0, 500.0, 0.0, 5.0, -12.5, 12.5, -18.0, 18.0, -30.0, 30.0, -30.0, 30.0}},
    {20, {0.0, 500.0, 0.0, 5.0, -12.5, 12.5, -18.0, 18.0, -30.0, 30.0, -30.0, 30.0}},
    {21, {0.0, 500.0, 0.0, 5.0, -12.5, 12.5, -18.0, 18.0, -30.0, 30.0, -30.0, 30.0}},
    {22, {0.0, 500.0, 0.0, 5.0, -12.5, 12.5, -18.0, 18.0, -30.0, 30.0, -30.0, 30.0}},
    {23, {0.0, 500.0, 0.0, 5.0, -12.5, 12.5, -18.0, 18.0, -30.0, 30.0, -30.0, 30.0}},
};

MotorControl::MotorControl(int motor_num)
    : motor_ins_type(MotorInsType::INS_NONE),
      motor_param_type(MotorParamType::PARAM_NONE),
      motor_id_check(0),
      motor_num_(motor_num)
{
    memset(&motor_comm_fbd, 0, sizeof(motor_comm_fbd));
    rv_motor_msg.resize(motor_num_);
}

// 获取电机ID
void MotorControl::get_motor_id(EtherCAT_Msg *TxMessage)
{
    motor_ins_type = MotorInsType::INS_GET_ID;

    TxMessage->can_ide = 0;
    TxMessage->motor[0].rtr = 0;
    TxMessage->motor[0].id = 0x7FF;
    TxMessage->motor[0].dlc = 4;

    TxMessage->motor[0].data[0] = 0xFF;
    TxMessage->motor[0].data[1] = 0xFF;
    TxMessage->motor[0].data[2] = 0x00;
    TxMessage->motor[0].data[3] = 0x82;
}

// 设置电机ID
void MotorControl::set_motor_id(EtherCAT_Msg *TxMessage, uint16_t motor_id, uint16_t motor_id_new)
{
    motor_ins_type = MotorInsType::INS_SET_ID;

    TxMessage->can_ide = 0;
    TxMessage->motor[0].id = 0x7FF;
    TxMessage->motor[0].dlc = 6;
    TxMessage->motor[0].rtr = 0;

    TxMessage->motor[0].data[0] = motor_id >> 8;
    TxMessage->motor[0].data[1] = motor_id & 0xFF;
    TxMessage->motor[0].data[2] = 0x00;
    TxMessage->motor[0].data[3] = 0x04;
    TxMessage->motor[0].data[4] = motor_id_new >> 8;
    TxMessage->motor[0].data[5] = motor_id_new & 0xFF;
}

// 重置电机ID
void MotorControl::reset_motor_id(EtherCAT_Msg *TxMessage)
{
    motor_ins_type = MotorInsType::INS_RESET_ID;

    TxMessage->can_ide = 0;
    TxMessage->motor[0].id = 0x7FF;
    TxMessage->motor[0].dlc = 6;
    TxMessage->motor[0].rtr = 0;

    TxMessage->motor[0].data[0] = 0x7F;
    TxMessage->motor[0].data[1] = 0x7F;
    TxMessage->motor[0].data[2] = 0x00;
    TxMessage->motor[0].data[3] = 0x05;
    TxMessage->motor[0].data[4] = 0x7F;
    TxMessage->motor[0].data[5] = 0x7F;
}

// 设置电机零点
void MotorControl::set_motor_zero(EtherCAT_Msg *TxMessage, uint8_t passage, uint16_t motor_id)
{
    TxMessage->can_ide = 0;
    TxMessage->motor[passage - 1].id = 0x7FF;
    TxMessage->motor[passage - 1].rtr = 0;
    TxMessage->motor[passage - 1].dlc = 4;

    TxMessage->motor[passage - 1].data[0] = motor_id >> 8;
    TxMessage->motor[passage - 1].data[1] = motor_id & 0xFF;
    TxMessage->motor[passage - 1].data[2] = 0x00;
    TxMessage->motor[passage - 1].data[3] = 0x03;
}

// 清除电机发送缓冲区
void MotorControl::clear_motor_buff(EtherCAT_Msg *TxMessage, uint8_t passage)
{
    TxMessage->can_ide = 0;
    TxMessage->motor[passage - 1].rtr = 0;
    TxMessage->motor[passage - 1].id = 0;
    TxMessage->motor[passage - 1].dlc = 0;

    TxMessage->motor[passage - 1].data[0] = 0x00;
    TxMessage->motor[passage - 1].data[1] = 0x00;
    TxMessage->motor[passage - 1].data[2] = 0x00;
    TxMessage->motor[passage - 1].data[3] = 0x00;
    TxMessage->motor[passage - 1].data[4] = 0x00;
    TxMessage->motor[passage - 1].data[5] = 0x00;
    TxMessage->motor[passage - 1].data[6] = 0x00;
    TxMessage->motor[passage - 1].data[7] = 0x00;
}

// 查询电机参数
void MotorControl::get_motor_param(EtherCAT_Msg *TxMessage, uint8_t passage, uint16_t motor_id, uint8_t code)
{
    motor_ins_type = MotorInsType::INS_GET_PARAM;
    motor_param_type = static_cast<MotorParamType>(code);

    TxMessage->can_ide = 0;
    TxMessage->motor[passage - 1].rtr = 0;
    TxMessage->motor[passage - 1].id = motor_id;
    TxMessage->motor[passage - 1].dlc = 2;

    TxMessage->motor[passage - 1].data[0] = 0xE0;
    TxMessage->motor[passage - 1].data[1] = code;
}

// 设置电机加速度
void MotorControl::set_motor_acc(EtherCAT_Msg *TxMessage, uint8_t passage, uint16_t motor_id, float acc, uint8_t ack_status)
{
    motor_ins_type = MotorInsType::INS_SET_PARAM;

    TxMessage->can_ide = 0;
    TxMessage->motor[passage - 1].rtr = 0;
    TxMessage->motor[passage - 1].id = motor_id;
    TxMessage->motor[passage - 1].dlc = 4;

    if (ack_status > 2)
    {
        return;
    }

    uint16_t acc_int = acc * 100;
    if (acc_int > 2000)
    {
        acc_int = 2000;
    }
    TxMessage->motor[passage - 1].data[0] = 0xC0 | ack_status;
    TxMessage->motor[passage - 1].data[1] = 0x01;
    TxMessage->motor[passage - 1].data[2] = acc_int >> 8;
    TxMessage->motor[passage - 1].data[3] = acc_int & 0xFF;
}

// 设置电机力位混控协议KP配置
void MotorControl::set_motor_tor_pos_config_kp(EtherCAT_Msg *TxMessage, uint8_t passage, uint16_t motor_id, uint16_t kp_min, uint16_t kp_max)
{
    motor_ins_type = MotorInsType::INS_SET_PARAM;

    TxMessage->can_ide = 0;
    TxMessage->motor[passage - 1].rtr = 0;
    TxMessage->motor[passage - 1].id = motor_id;
    TxMessage->motor[passage - 1].dlc = 6;

    TxMessage->motor[passage - 1].data[0] = 0xC0;
    TxMessage->motor[passage - 1].data[1] = 0x05;
    TxMessage->motor[passage - 1].data[2] = kp_min >> 8;
    TxMessage->motor[passage - 1].data[3] = kp_min & 0xFF;
    TxMessage->motor[passage - 1].data[4] = kp_max >> 8;
    TxMessage->motor[passage - 1].data[5] = kp_max & 0xFF;
}

// 设置电机力位混控协议KD配置
void MotorControl::set_motor_tor_pos_config_kd(EtherCAT_Msg *TxMessage, uint8_t passage, uint16_t motor_id, uint16_t kd_min, uint16_t kd_max)
{
    motor_ins_type = MotorInsType::INS_SET_PARAM;

    TxMessage->can_ide = 0;
    TxMessage->motor[passage - 1].rtr = 0;
    TxMessage->motor[passage - 1].id = motor_id;
    TxMessage->motor[passage - 1].dlc = 6;

    TxMessage->motor[passage - 1].data[0] = 0xC0;
    TxMessage->motor[passage - 1].data[1] = 0x06;
    TxMessage->motor[passage - 1].data[2] = kd_min >> 8;
    TxMessage->motor[passage - 1].data[3] = kd_min & 0xFF;
    TxMessage->motor[passage - 1].data[4] = kd_max >> 8;
    TxMessage->motor[passage - 1].data[5] = kd_max & 0xFF;
}

// 设置电机力位混控协议位置配置
void MotorControl::set_motor_tor_pos_config_pos(EtherCAT_Msg *TxMessage, uint8_t passage, uint16_t motor_id, float pos_min, float pos_max)
{
    motor_ins_type = MotorInsType::INS_SET_PARAM;

    TxMessage->can_ide = 0;
    TxMessage->motor[passage - 1].rtr = 0;
    TxMessage->motor[passage - 1].id = motor_id;
    TxMessage->motor[passage - 1].dlc = 6;

    int16_t pos_min_init = pos_min * 100;
    int16_t pos_max_init = pos_max * 100;

    TxMessage->motor[passage - 1].data[0] = 0xC0;
    TxMessage->motor[passage - 1].data[1] = 0x07;
    TxMessage->motor[passage - 1].data[2] = pos_min_init >> 8;
    TxMessage->motor[passage - 1].data[3] = pos_min_init & 0xFF;
    TxMessage->motor[passage - 1].data[4] = pos_max_init >> 8;
    TxMessage->motor[passage - 1].data[5] = pos_max_init & 0xFF;
}

// 设置电机力位混控协议速度配置
void MotorControl::set_motor_tor_pos_config_spd(EtherCAT_Msg *TxMessage, uint8_t passage, uint16_t motor_id, float spd_min, float spd_max)
{
    motor_ins_type = MotorInsType::INS_SET_PARAM;

    TxMessage->can_ide = 0;
    TxMessage->motor[passage - 1].rtr = 0;
    TxMessage->motor[passage - 1].id = motor_id;
    TxMessage->motor[passage - 1].dlc = 6;

    int16_t spd_min_init = spd_min * 100;
    int16_t spd_max_init = spd_max * 100;

    TxMessage->motor[passage - 1].data[0] = 0xC0;
    TxMessage->motor[passage - 1].data[1] = 0x08;
    TxMessage->motor[passage - 1].data[2] = spd_min_init >> 8;
    TxMessage->motor[passage - 1].data[3] = spd_min_init & 0xFF;
    TxMessage->motor[passage - 1].data[4] = spd_max_init >> 8;
    TxMessage->motor[passage - 1].data[5] = spd_max_init & 0xFF;
}
// 设置电机力位混控协议扭矩配置
void MotorControl::set_motor_tor_pos_config_tor(EtherCAT_Msg *TxMessage, uint8_t passage, uint16_t motor_id, float tor_min, float tor_max)
{
    motor_ins_type = MotorInsType::INS_SET_PARAM;

    TxMessage->can_ide = 0;
    TxMessage->motor[passage - 1].rtr = 0;
    TxMessage->motor[passage - 1].id = motor_id;
    TxMessage->motor[passage - 1].dlc = 6;

    int16_t tor_min_init = tor_min * 10;
    int16_t tor_max_init = tor_max * 10;

    TxMessage->motor[passage - 1].data[0] = 0xC0;
    TxMessage->motor[passage - 1].data[1] = 0x09;
    TxMessage->motor[passage - 1].data[2] = tor_min_init >> 8;
    TxMessage->motor[passage - 1].data[3] = tor_min_init & 0xFF;
    TxMessage->motor[passage - 1].data[4] = tor_max_init >> 8;
    TxMessage->motor[passage - 1].data[5] = tor_max_init & 0xFF;
}

// 设置电机力位混控协议电流配置
void MotorControl::set_motor_tor_pos_config_cur(EtherCAT_Msg *TxMessage, uint8_t passage, uint16_t motor_id, float cur_min, float cur_max)
{
    motor_ins_type = MotorInsType::INS_SET_PARAM;

    TxMessage->can_ide = 0;
    TxMessage->motor[passage - 1].rtr = 0;
    TxMessage->motor[passage - 1].id = motor_id;
    TxMessage->motor[passage - 1].dlc = 6;

    int16_t cur_min_init = cur_min * 10;
    int16_t cur_max_init = cur_max * 10;

    TxMessage->motor[passage - 1].data[0] = 0xC0;
    TxMessage->motor[passage - 1].data[1] = 0x0A;
    TxMessage->motor[passage - 1].data[2] = cur_min_init >> 8;
    TxMessage->motor[passage - 1].data[3] = cur_min_init & 0xFF;
    TxMessage->motor[passage - 1].data[4] = cur_max_init >> 8;
    TxMessage->motor[passage - 1].data[5] = cur_max_init & 0xFF;
}

// 设置电机CAN超时时间
void MotorControl::set_motor_can_timeout(EtherCAT_Msg *TxMessage, uint8_t passage, uint16_t motor_id, uint16_t timeout)
{
    motor_ins_type = MotorInsType::INS_SET_PARAM;

    TxMessage->can_ide = 0;
    TxMessage->motor[passage - 1].rtr = 0;
    TxMessage->motor[passage - 1].id = motor_id;
    TxMessage->motor[passage - 1].dlc = 4;

    if (timeout > 500)
    {
        timeout = 500;
    }
    TxMessage->motor[passage - 1].data[0] = 0xC0;
    TxMessage->motor[passage - 1].data[1] = 0x0B;
    TxMessage->motor[passage - 1].data[2] = timeout >> 8;
    TxMessage->motor[passage - 1].data[3] = timeout & 0xFF;
}

// 设置电流环PI
void MotorControl::set_motor_current_loop_pi(EtherCAT_Msg *TxMessage, uint8_t passage, uint16_t motor_id, float kp, float ki)
{
    motor_ins_type = MotorInsType::INS_SET_PARAM;

    TxMessage->can_ide = 0;
    TxMessage->motor[passage - 1].rtr = 0;
    TxMessage->motor[passage - 1].id = motor_id;
    TxMessage->motor[passage - 1].dlc = 6;

    uint16_t kp_int = kp * 10000;
    uint16_t ki_int = ki * 10;

    TxMessage->motor[passage - 1].data[0] = 0xC0;
    TxMessage->motor[passage - 1].data[1] = 0x0C;
    TxMessage->motor[passage - 1].data[2] = kp_int >> 8;
    TxMessage->motor[passage - 1].data[3] = kp_int & 0xFF;
    TxMessage->motor[passage - 1].data[4] = ki_int >> 8;
    TxMessage->motor[passage - 1].data[5] = ki_int & 0xFF;
}

// 设置速度环PI
void MotorControl::set_motor_speed_loop_pi(EtherCAT_Msg *TxMessage, uint8_t passage, uint16_t motor_id, float kp, float ki)
{
    motor_ins_type = MotorInsType::INS_SET_PARAM;

    TxMessage->can_ide = 0;
    TxMessage->motor[passage - 1].rtr = 0;
    TxMessage->motor[passage - 1].id = motor_id;
    TxMessage->motor[passage - 1].dlc = 6;

    uint16_t kp_int = kp * 100000;
    uint16_t ki_int = ki * 100000;

    TxMessage->motor[passage - 1].data[0] = 0xC0;
    TxMessage->motor[passage - 1].data[1] = 0x0D;
    TxMessage->motor[passage - 1].data[2] = kp_int >> 8;
    TxMessage->motor[passage - 1].data[3] = kp_int & 0xFF;
    TxMessage->motor[passage - 1].data[4] = ki_int >> 8;
    TxMessage->motor[passage - 1].data[5] = ki_int & 0xFF;
}

// 设置位置环PD
void MotorControl::set_motor_position_loop_pd(EtherCAT_Msg *TxMessage, uint8_t passage, uint16_t motor_id, float kp, float kd)
{
    motor_ins_type = MotorInsType::INS_SET_PARAM;

    TxMessage->can_ide = 0;
    TxMessage->motor[passage - 1].rtr = 0;
    TxMessage->motor[passage - 1].id = motor_id;
    TxMessage->motor[passage - 1].dlc = 6;

    uint16_t kp_int = kp * 100000;
    uint16_t kd_int = kd * 100000;

    TxMessage->motor[passage - 1].data[0] = 0xC0;
    TxMessage->motor[passage - 1].data[1] = 0x0E;
    TxMessage->motor[passage - 1].data[2] = kp_int >> 8;
    TxMessage->motor[passage - 1].data[3] = kp_int & 0xFF;
    TxMessage->motor[passage - 1].data[4] = kd_int >> 8;
    TxMessage->motor[passage - 1].data[5] = kd_int & 0xFF;
}

// 设置电机扭矩系数 Kt 校准使能
void MotorControl::set_motor_kt_enable(EtherCAT_Msg *TxMessage, uint8_t passage, uint16_t motor_id, bool is_enable)
{
    motor_ins_type = MotorInsType::INS_SET_PARAM;

    TxMessage->can_ide = 0;
    TxMessage->motor[passage - 1].rtr = 0;
    TxMessage->motor[passage - 1].id = motor_id;
    TxMessage->motor[passage - 1].dlc = 3;

    TxMessage->motor[passage - 1].data[0] = 0xC0;
    TxMessage->motor[passage - 1].data[1] = 0x0F;
    TxMessage->motor[passage - 1].data[2] = is_enable ? 0x01 : 0x00;
}

// 设置力位混合模式
void MotorControl::set_motor_tor_pos(EtherCAT_Msg *TxMessage, uint8_t passage, uint16_t motor_id, float kp, float kd, float pos, float spd, float tor)
{
    motor_ins_type = MotorInsType::INS_SET_TOR_POS;

    if (g_param_map.find(motor_id) == g_param_map.end())
    {
        printf("motor_id:%d not found in g_param_map\n", motor_id);
        return;
    }

    std::vector<float> params = g_param_map[motor_id];
    if (params.size() < PARAM_COUNT)
    {
        return;
    }

    int kp_int;
    int kd_int;
    int pos_int;
    int spd_int;
    int tor_int;

    TxMessage->can_ide = 0;
    TxMessage->motor[passage - 1].rtr = 0;
    TxMessage->motor[passage - 1].id = motor_id;
    TxMessage->motor[passage - 1].dlc = 8;

    if (kp > params[KP_MAX])
    {
        kp = params[KP_MAX];
    }
    else if (kp < params[KP_MIN])
    {
        kp = params[KP_MIN];
    }
    if (kd > params[KD_MAX])
    {
        kd = params[KD_MAX];
    }
    else if (kd < params[KD_MIN])
    {
        kd = params[KD_MIN];
    }
    if (pos > params[POS_MAX])
    {
        pos = params[POS_MAX];
    }
    else if (pos < params[POS_MIN])
    {
        pos = params[POS_MIN];
    }
    if (spd > params[SPD_MAX])
    {
        spd = params[SPD_MAX];
    }
    else if (spd < params[SPD_MIN])
    {
        spd = params[SPD_MIN];
    }
    if (tor > params[TOR_MAX])
    {
        tor = params[TOR_MAX];
    }
    else if (tor < params[TOR_MIN])
    {
        tor = params[TOR_MIN];
    }

    kp_int = math_ops_.float_to_uint(kp, params[KP_MIN], params[KP_MAX], 12);
    kd_int = math_ops_.float_to_uint(kd, params[KD_MIN], params[KD_MAX], 9);
    pos_int = math_ops_.float_to_uint(pos, params[POS_MIN], params[POS_MAX], 16);
    spd_int = math_ops_.float_to_uint(spd, params[SPD_MIN], params[SPD_MAX], 12);
    tor_int = math_ops_.float_to_uint(tor, params[TOR_MIN], params[TOR_MAX], 12);

    // printf("kp_int:%d, kd_int:%d, pos_int:%d, spd_int:%d, tor_int:%d\n", kp_int, kd_int, pos_int, spd_int, tor_int);

    TxMessage->motor[passage - 1].data[0] = 0x00 | (kp_int >> 7);
    TxMessage->motor[passage - 1].data[1] = ((kp_int & 0x7F) << 1) | ((kd_int & 0x100) >> 8);
    TxMessage->motor[passage - 1].data[2] = kd_int & 0xFF;
    TxMessage->motor[passage - 1].data[3] = pos_int >> 8;
    TxMessage->motor[passage - 1].data[4] = pos_int & 0xFF;
    TxMessage->motor[passage - 1].data[5] = spd_int >> 4;
    TxMessage->motor[passage - 1].data[6] = (spd_int & 0x0F) << 4 | (tor_int >> 8);
    TxMessage->motor[passage - 1].data[7] = tor_int & 0xFF;

    // 更新目标
    rv_motor_msg[motor_id - 1].kp_target = kp;
    rv_motor_msg[motor_id - 1].kd_target = kd;
    rv_motor_msg[motor_id - 1].angle_target = pos;
    rv_motor_msg[motor_id - 1].speed_target = spd;
    rv_motor_msg[motor_id - 1].torque_target = tor;
}

// 设置伺服位置控制模式
void MotorControl::set_motor_pos(EtherCAT_Msg *TxMessage, uint8_t passage, uint16_t motor_id, float pos, float spd, float cur, uint8_t ack_status)
{
    motor_ins_type = MotorInsType::INS_SET_POS;

    int spd_int;
    int cur_int;

    TxMessage->can_ide = 0;
    TxMessage->motor[passage - 1].rtr = 0;
    TxMessage->motor[passage - 1].id = motor_id;
    TxMessage->motor[passage - 1].dlc = 8;

    if (ack_status > 3)
    {
        return;
    }

    if (spd > 3276.7)
    {
        spd = 3276.7;
    }
    else if (spd < -3276.7)
    {
        spd = -3276.7;
    }
    if (cur > 409.5)
    {
        cur = 409.5;
    }
    else if (cur < 0.0)
    {
        cur = 0.0;
    }

    spd_int = math_ops_.float_to_uint(spd, 0.0, 3276.7, 15);
    cur_int = math_ops_.float_to_uint(cur, 0.0, 409.5, 12);

    rv_type_convert.to_float = pos;
    TxMessage->motor[passage - 1].data[0] = 0x20 | (rv_type_convert.buf[3] >> 3);
    TxMessage->motor[passage - 1].data[1] = (rv_type_convert.buf[3] << 5) | (rv_type_convert.buf[2] >> 3);
    TxMessage->motor[passage - 1].data[2] = (rv_type_convert.buf[2] << 5) | (rv_type_convert.buf[1] >> 3);
    TxMessage->motor[passage - 1].data[3] = (rv_type_convert.buf[1] << 5) | (rv_type_convert.buf[0] >> 3);
    TxMessage->motor[passage - 1].data[4] = (rv_type_convert.buf[0] << 5) | (spd_int >> 10);
    TxMessage->motor[passage - 1].data[5] = (spd_int & 0x3FC) >> 2;
    TxMessage->motor[passage - 1].data[6] = (spd_int & 0x03) << 6 | (cur_int >> 6);
    TxMessage->motor[passage - 1].data[7] = (cur_int & 0x3F) << 2 | ack_status;

    // 更新目标
    rv_motor_msg[motor_id - 1].angle_target = pos;
    rv_motor_msg[motor_id - 1].speed_target = spd;
    rv_motor_msg[motor_id - 1].current_target = cur;
}
// 设置伺服速度控制模式
void MotorControl::set_motor_speed(EtherCAT_Msg *TxMessage, uint8_t passage, uint16_t motor_id, float spd, float cur, uint8_t ack_status)
{
    motor_ins_type = MotorInsType::INS_SET_SPD;

    int cur_int;

    TxMessage->can_ide = 0;
    TxMessage->motor[passage - 1].rtr = 0;
    TxMessage->motor[passage - 1].id = motor_id;
    TxMessage->motor[passage - 1].dlc = 7;

    cur_int = cur * 10;

    rv_type_convert.to_float = spd;
    TxMessage->motor[passage - 1].data[0] = 0x40 | ack_status;
    TxMessage->motor[passage - 1].data[1] = rv_type_convert.buf[3];
    TxMessage->motor[passage - 1].data[2] = rv_type_convert.buf[2];
    TxMessage->motor[passage - 1].data[3] = rv_type_convert.buf[1];
    TxMessage->motor[passage - 1].data[4] = rv_type_convert.buf[0];
    TxMessage->motor[passage - 1].data[5] = cur_int >> 8;
    TxMessage->motor[passage - 1].data[6] = cur_int & 0xFF;

    // 更新目标
    rv_motor_msg[motor_id - 1].speed_target = spd;
    rv_motor_msg[motor_id - 1].current_target = cur;
}

// 设置电流力矩控制模式
void MotorControl::set_motor_tor_cur(EtherCAT_Msg *TxMessage, uint8_t passage, uint16_t motor_id, int16_t cur_tor, uint8_t ctrl_status, uint8_t ack_status)
{
    motor_ins_type = MotorInsType::INS_SET_TOR_CUR;

    TxMessage->can_ide = 0;
    TxMessage->motor[passage - 1].rtr = 0;
    TxMessage->motor[passage - 1].id = motor_id;
    TxMessage->motor[passage - 1].dlc = 3;

    if (ack_status > 3)
    {
        return;
    }
    if (ctrl_status > 7)
    {
        return;
    }
    if (ctrl_status) // enter torque control mode or brake mode
    {
        if (cur_tor > 3000)
        {
            cur_tor = 3000;
        }
        else if (cur_tor < -3000)
        {
            cur_tor = -3000;
        }
    }
    else
    {
        if (cur_tor > 2000)
        {
            cur_tor = 2000;
        }
        else if (cur_tor < -2000)
        {
            cur_tor = -2000;
        }
    }
    TxMessage->motor[passage - 1].data[0] = 0x60 | ctrl_status << 2 | ack_status;
    TxMessage->motor[passage - 1].data[1] = cur_tor >> 8;
    TxMessage->motor[passage - 1].data[2] = cur_tor & 0xFF;
}

// 解析CAN数据
uint8_t MotorControl::rv_can_data_repack(EtherCAT_Msg *RxMessage, uint8_t comm_mode, uint8_t slave_id)
{
    uint8_t motor_id = 0;
    uint8_t ack_status = 0;
    int pos_int = 0;
    int spd_int = 0;
    int cur_int = 0;

    for (int i = 0; i < 6; ++i)
    {
        if (RxMessage->motor[i].dlc == 0)
        {
            continue;
        }

        // if (RxMessage->motor[i].data[0] != 0x20)
        // {
        //     int id = RxMessage->motor[i].id;
        //     int dlc = RxMessage->motor[i].dlc;
        //     printf("id:0x%02X, dlc:%d, ", id, dlc);
        //     for (int j = 0; j < dlc; j++)
        //     {
        //         printf("0x%02X ", RxMessage->motor[i].data[j]);
        //     }
        //     printf("\n");
        // }

        if (RxMessage->motor[i].id == 0x7FF)
        {
            if (RxMessage->motor[i].data[2] != 0x01) // 检测是否为电机反馈指令
            {
                return 0; // 不是电机反馈指令
            }
            if (motor_ins_type == MotorInsType::INS_GET_ID && RxMessage->motor[i].dlc == 5 && RxMessage->motor[i].data[0] == 0xFF && RxMessage->motor[i].data[1] == 0xFF) // 查询ID成功
            {
                motor_comm_fbd.motor_id = RxMessage->motor[i].data[3] << 8 | RxMessage->motor[i].data[4];
                motor_comm_fbd.motor_fbd = 0x06;
            }
            else if (motor_ins_type == MotorInsType::INS_GET_ID && RxMessage->motor[i].dlc == 4 && RxMessage->motor[i].data[0] == 0x80 && RxMessage->motor[i].data[1] == 0x80) // 查询ID失败
            {
                motor_comm_fbd.motor_id = 0;
                motor_comm_fbd.motor_fbd = 0x80;
            }
            else if (motor_ins_type == MotorInsType::INS_RESET_ID && RxMessage->motor[i].dlc == 6 && RxMessage->motor[i].data[0] == 0x7F && RxMessage->motor[i].data[1] == 0x7F) // 重置ID成功
            {
                motor_comm_fbd.motor_id = 1;
                motor_comm_fbd.motor_fbd = 0x05;
            }
            else if (motor_ins_type == MotorInsType::INS_SET_ID && RxMessage->motor[i].dlc == 4) // 设置新ID
            {
                motor_comm_fbd.motor_id = RxMessage->motor[i].data[0] << 8 | RxMessage->motor[i].data[1];
                motor_comm_fbd.motor_fbd = RxMessage->motor[i].data[3];
            }
            else if (motor_ins_type == MotorInsType::INS_SET_ZERO && RxMessage->motor[i].dlc == 4) // 设置电机零点
            {
                motor_comm_fbd.motor_id = RxMessage->motor[i].data[0] << 8 | RxMessage->motor[i].data[1];
                motor_comm_fbd.motor_fbd = RxMessage->motor[i].data[3];
            }
            else
            {
                return 0;
            }
            return 100 + i;
        }
        else if (comm_mode == 0x00 && RxMessage->motor[i].dlc != 0) // 应答模式
        {
            ack_status = RxMessage->motor[i].data[0] >> 5;
            motor_id = RxMessage->motor[i].id - 1;
            motor_id_check = RxMessage->motor[i].id;
            rv_motor_msg[motor_id].motor_id = motor_id_check;
            rv_motor_msg[motor_id].error = RxMessage->motor[i].data[0] & 0x1F;
            if ((motor_ins_type == MotorInsType::INS_SET_TOR_POS || motor_ins_type == MotorInsType::INS_SET_POS || motor_ins_type == MotorInsType::INS_SET_SPD || motor_ins_type == MotorInsType::INS_SET_TOR_CUR) && ack_status == 1 && RxMessage->motor[i].dlc == 8) // 返回报文类型1
            {
                if (g_param_map.find(motor_id_check) == g_param_map.end())
                {
                    printf("motor_id:%d not found in g_param_map\n", motor_id_check);
                    return 0;
                }

                std::vector<float> params = g_param_map[motor_id_check];
                if (params.size() < PARAM_COUNT)
                {
                    return 0;
                }
                pos_int = RxMessage->motor[i].data[1] << 8 | RxMessage->motor[i].data[2];
                spd_int = RxMessage->motor[i].data[3] << 4 | (RxMessage->motor[i].data[4] & 0xF0) >> 4;
                cur_int = (RxMessage->motor[i].data[4] & 0x0F) << 8 | RxMessage->motor[i].data[5];
                rv_motor_msg[motor_id].angle_actual_rad = math_ops_.uint_to_float(pos_int, params[POS_MIN], params[POS_MAX], 16);
                rv_motor_msg[motor_id].speed_actual_rad = math_ops_.uint_to_float(spd_int, params[SPD_MIN], params[SPD_MAX], 12);
                rv_motor_msg[motor_id].current_actual = math_ops_.uint_to_float(cur_int, params[CUR_MIN], params[CUR_MAX], 12);
                rv_motor_msg[motor_id].temp_motor = (RxMessage->motor[i].data[6] - 50) / 2;
                rv_motor_msg[motor_id].temp_mos = (RxMessage->motor[i].data[7] - 50) / 2;
            }
            else if ((motor_ins_type == MotorInsType::INS_SET_POS || motor_ins_type == MotorInsType::INS_SET_SPD || motor_ins_type == MotorInsType::INS_SET_TOR_CUR) && ack_status == 2 && RxMessage->motor[i].dlc == 8) // 返回报文类型2
            {
                rv_type_convert.buf[0] = RxMessage->motor[i].data[4];
                rv_type_convert.buf[1] = RxMessage->motor[i].data[3];
                rv_type_convert.buf[2] = RxMessage->motor[i].data[2];
                rv_type_convert.buf[3] = RxMessage->motor[i].data[1];
                rv_motor_msg[motor_id].angle_actual = rv_type_convert.to_float;
                rv_motor_msg[motor_id].current_actual_int = RxMessage->motor[i].data[5] << 8 | RxMessage->motor[i].data[6];
                rv_motor_msg[motor_id].current_actual = rv_motor_msg[motor_id].current_actual_int / 100.0f;
                rv_motor_msg[motor_id].temp_motor = (RxMessage->motor[i].data[7] - 50) / 2;
            }
            else if ((motor_ins_type == MotorInsType::INS_SET_POS || motor_ins_type == MotorInsType::INS_SET_SPD || motor_ins_type == MotorInsType::INS_SET_TOR_CUR) && ack_status == 3 && RxMessage->motor[i].dlc == 8) // 返回报文类型3
            {
                rv_type_convert.buf[0] = RxMessage->motor[i].data[4];
                rv_type_convert.buf[1] = RxMessage->motor[i].data[3];
                rv_type_convert.buf[2] = RxMessage->motor[i].data[2];
                rv_type_convert.buf[3] = RxMessage->motor[i].data[1];
                rv_motor_msg[motor_id].speed_actual = rv_type_convert.to_float;
                rv_motor_msg[motor_id].current_actual_int = RxMessage->motor[i].data[5] << 8 | RxMessage->motor[i].data[6];
                rv_motor_msg[motor_id].current_actual = rv_motor_msg[motor_id].current_actual_int / 100.0f;
                rv_motor_msg[motor_id].temp_motor = (RxMessage->motor[i].data[7] - 50) / 2;
            }
            else if (ack_status == 4 && RxMessage->motor[i].dlc == 4) // 返回报文类型4
            {
                motor_comm_fbd.ins_code = RxMessage->motor[i].data[1];
                motor_comm_fbd.motor_fbd = RxMessage->motor[i].data[2];
            }
            else if (motor_ins_type == MotorInsType::INS_GET_PARAM && ack_status == 5) // 返回报文类型5
            {
                motor_comm_fbd.ins_code = RxMessage->motor[i].data[1];

                // 获取当前位置（°）
                if (motor_param_type == MotorParamType::PARAM_POS && motor_comm_fbd.ins_code == static_cast<uint8_t>(MotorParamType::PARAM_POS) && RxMessage->motor[i].dlc == 6)
                {
                    rv_type_convert.buf[0] = RxMessage->motor[i].data[5];
                    rv_type_convert.buf[1] = RxMessage->motor[i].data[4];
                    rv_type_convert.buf[2] = RxMessage->motor[i].data[3];
                    rv_type_convert.buf[3] = RxMessage->motor[i].data[2];
                    rv_motor_msg[motor_id].angle_actual = rv_type_convert.to_float;
                }
                // 获取当前速度（rpm）
                else if (motor_param_type == MotorParamType::PARAM_SPD && motor_comm_fbd.ins_code == static_cast<uint8_t>(MotorParamType::PARAM_SPD) && RxMessage->motor[i].dlc == 6)
                {
                    rv_type_convert.buf[0] = RxMessage->motor[i].data[5];
                    rv_type_convert.buf[1] = RxMessage->motor[i].data[4];
                    rv_type_convert.buf[2] = RxMessage->motor[i].data[3];
                    rv_type_convert.buf[3] = RxMessage->motor[i].data[2];
                    rv_motor_msg[motor_id].speed_actual = rv_type_convert.to_float;
                }
                // 获取当前电流（A）
                else if (motor_param_type == MotorParamType::PARAM_CUR && motor_comm_fbd.ins_code == static_cast<uint8_t>(MotorParamType::PARAM_CUR) && RxMessage->motor[i].dlc == 6)
                {
                    rv_type_convert.buf[0] = RxMessage->motor[i].data[5];
                    rv_type_convert.buf[1] = RxMessage->motor[i].data[4];
                    rv_type_convert.buf[2] = RxMessage->motor[i].data[3];
                    rv_type_convert.buf[3] = RxMessage->motor[i].data[2];
                    rv_motor_msg[motor_id].current_actual = rv_type_convert.to_float;
                }
                // 获取当前功率（W）
                else if (motor_param_type == MotorParamType::PARAM_PWR && motor_comm_fbd.ins_code == static_cast<uint8_t>(MotorParamType::PARAM_PWR) && RxMessage->motor[i].dlc == 6)
                {
                    rv_type_convert.buf[0] = RxMessage->motor[i].data[5];
                    rv_type_convert.buf[1] = RxMessage->motor[i].data[4];
                    rv_type_convert.buf[2] = RxMessage->motor[i].data[3];
                    rv_type_convert.buf[3] = RxMessage->motor[i].data[2];
                    rv_motor_msg[motor_id].power = rv_type_convert.to_float;
                }
                // 获取加速度 (rad/s)
                else if (motor_param_type == MotorParamType::PARAM_ACC && motor_comm_fbd.ins_code == static_cast<uint8_t>(MotorParamType::PARAM_ACC) && RxMessage->motor[i].dlc == 4)
                {
                    uint16_t acc_int = RxMessage->motor[i].data[2] << 8 | RxMessage->motor[i].data[3];
                    rv_motor_msg[motor_id].acc = acc_int / 100.0;
                }
                // 获取磁链观测增益
                else if (motor_param_type == MotorParamType::PARAM_FLUX_GAIN && motor_comm_fbd.ins_code == static_cast<uint8_t>(MotorParamType::PARAM_FLUX_GAIN) && RxMessage->motor[i].dlc == 4)
                {
                    rv_motor_msg[motor_id].flux_gain = RxMessage->motor[i].data[2] << 8 | RxMessage->motor[i].data[3];
                }
                // 获取扰动补偿系数
                else if (motor_param_type == MotorParamType::PARAM_DC_COE && motor_comm_fbd.ins_code == static_cast<uint8_t>(MotorParamType::PARAM_DC_COE) && RxMessage->motor[i].dlc == 4)
                {
                    rv_motor_msg[motor_id].dc_coe = RxMessage->motor[i].data[2] << 8 | RxMessage->motor[i].data[3];
                }
                // 获取反馈补偿增益
                else if (motor_param_type == MotorParamType::PARAM_FEEDBACK_GAIN && motor_comm_fbd.ins_code == static_cast<uint8_t>(MotorParamType::PARAM_FEEDBACK_GAIN) && RxMessage->motor[i].dlc == 4)
                {
                    rv_motor_msg[motor_id].feedback_gain = RxMessage->motor[i].data[2] << 8 | RxMessage->motor[i].data[3];
                }
                // 获取阻尼系数
                else if (motor_param_type == MotorParamType::PARAM_DAMP_COE && motor_comm_fbd.ins_code == static_cast<uint8_t>(MotorParamType::PARAM_DAMP_COE) && RxMessage->motor[i].dlc == 4)
                {
                    rv_motor_msg[motor_id].damp_coe = RxMessage->motor[i].data[2] << 8 | RxMessage->motor[i].data[3];
                }
                // 获取扭矩系数
                else if (motor_param_type == MotorParamType::PARAM_TOR_COE && motor_comm_fbd.ins_code == static_cast<uint8_t>(MotorParamType::PARAM_TOR_COE) && RxMessage->motor[i].dlc == 4)
                {
                    uint16_t tor_coe_int = RxMessage->motor[i].data[2] << 8 | RxMessage->motor[i].data[3];
                    rv_motor_msg[motor_id].tor_coe = tor_coe_int / 100.0;
                }
                // 获取力位混控协议 KP 范围
                else if (motor_param_type == MotorParamType::PARAM_TOR_POS_KP && motor_comm_fbd.ins_code == static_cast<uint8_t>(MotorParamType::PARAM_TOR_POS_KP) && RxMessage->motor[i].dlc == 6)
                {
                    uint16_t kp_min_int = RxMessage->motor[i].data[2] << 8 | RxMessage->motor[i].data[3];
                    uint16_t kp_max_int = RxMessage->motor[i].data[4] << 8 | RxMessage->motor[i].data[5];
                    rv_motor_msg[motor_id].kp_min = kp_min_int / 1.0;
                    rv_motor_msg[motor_id].kp_max = kp_max_int / 1.0;
                }
                // 获取力位混控协议 KD 范围
                else if (motor_param_type == MotorParamType::PARAM_TOR_POS_KD && motor_comm_fbd.ins_code == static_cast<uint8_t>(MotorParamType::PARAM_TOR_POS_KD) && RxMessage->motor[i].dlc == 6)
                {
                    uint16_t kd_min_int = RxMessage->motor[i].data[2] << 8 | RxMessage->motor[i].data[3];
                    uint16_t kd_max_int = RxMessage->motor[i].data[4] << 8 | RxMessage->motor[i].data[5];
                    rv_motor_msg[motor_id].kd_min = kd_min_int / 1.0;
                    rv_motor_msg[motor_id].kd_max = kd_max_int / 1.0;
                }
                // 获取力位混控协议 POS 范围
                else if (motor_param_type == MotorParamType::PARAM_TOR_POS_POS && motor_comm_fbd.ins_code == static_cast<uint8_t>(MotorParamType::PARAM_TOR_POS_POS) && RxMessage->motor[i].dlc == 6)
                {
                    int16_t pos_min_int = RxMessage->motor[i].data[2] << 8 | RxMessage->motor[i].data[3];
                    int16_t pos_max_int = RxMessage->motor[i].data[4] << 8 | RxMessage->motor[i].data[5];
                    rv_motor_msg[motor_id].pos_min = pos_min_int / 100.0;
                    rv_motor_msg[motor_id].pos_max = pos_max_int / 100.0;
                }
                // 获取力位混控协议 SPD 范围
                else if (motor_param_type == MotorParamType::PARAM_TOR_POS_SPD && motor_comm_fbd.ins_code == static_cast<uint8_t>(MotorParamType::PARAM_TOR_POS_SPD) && RxMessage->motor[i].dlc == 6)
                {
                    int16_t spd_min_int = RxMessage->motor[i].data[2] << 8 | RxMessage->motor[i].data[3];
                    int16_t spd_max_int = RxMessage->motor[i].data[4] << 8 | RxMessage->motor[i].data[5];
                    rv_motor_msg[motor_id].spd_min = spd_min_int / 100.0;
                    rv_motor_msg[motor_id].spd_max = spd_max_int / 100.0;
                }
                // 获取力位混控协议 TOR 范围
                else if (motor_param_type == MotorParamType::PARAM_TOR_POS_TOR && motor_comm_fbd.ins_code == static_cast<uint8_t>(MotorParamType::PARAM_TOR_POS_TOR) && RxMessage->motor[i].dlc == 6)
                {
                    int16_t tor_min_int = RxMessage->motor[i].data[2] << 8 | RxMessage->motor[i].data[3];
                    int16_t tor_max_int = RxMessage->motor[i].data[4] << 8 | RxMessage->motor[i].data[5];
                    rv_motor_msg[motor_id].tor_min = tor_min_int / 10.0;
                    rv_motor_msg[motor_id].tor_max = tor_max_int / 10.0;
                }
                // 获取力位混控协议 CUR 范围
                else if (motor_param_type == MotorParamType::PARAM_TOR_POS_CUR && motor_comm_fbd.ins_code == static_cast<uint8_t>(MotorParamType::PARAM_TOR_POS_CUR) && RxMessage->motor[i].dlc == 6)
                {
                    int16_t cur_min_int = RxMessage->motor[i].data[2] << 8 | RxMessage->motor[i].data[3];
                    int16_t cur_max_int = RxMessage->motor[i].data[4] << 8 | RxMessage->motor[i].data[5];
                    rv_motor_msg[motor_id].cur_min = cur_min_int / 10.0;
                    rv_motor_msg[motor_id].cur_max = cur_max_int / 10.0;
                }
                // CAN超时时间
                else if (motor_param_type == MotorParamType::PARAM_CAN_TIMEOUT && motor_comm_fbd.ins_code == static_cast<uint8_t>(MotorParamType::PARAM_CAN_TIMEOUT) && RxMessage->motor[i].dlc == 4)
                {
                    uint16_t can_timeout_int = RxMessage->motor[i].data[2] << 8 | RxMessage->motor[i].data[3];
                    rv_motor_msg[motor_id].can_timeout = can_timeout_int / 1.0;
                }
                // 电流环 KP 和 KI
                else if (motor_param_type == MotorParamType::PARAM_CUR_LOOP_KP_KI && motor_comm_fbd.ins_code == static_cast<uint8_t>(MotorParamType::PARAM_CUR_LOOP_KP_KI) && RxMessage->motor[i].dlc == 6)
                {
                    uint16_t cur_loop_kp_int = RxMessage->motor[i].data[2] << 8 | RxMessage->motor[i].data[3];
                    uint16_t cur_loop_ki_int = RxMessage->motor[i].data[4] << 8 | RxMessage->motor[i].data[5];
                    rv_motor_msg[motor_id].cur_loop_kp = cur_loop_kp_int / 10000.0;
                    rv_motor_msg[motor_id].cur_loop_ki = cur_loop_ki_int / 10.0;
                }
                // 速度环 KP 和 KI
                else if (motor_param_type == MotorParamType::PARAM_SPD_LOOP_KP_KI && motor_comm_fbd.ins_code == static_cast<uint8_t>(MotorParamType::PARAM_SPD_LOOP_KP_KI) && RxMessage->motor[i].dlc == 6)
                {
                    uint16_t spd_loop_kp_int = RxMessage->motor[i].data[2] << 8 | RxMessage->motor[i].data[3];
                    uint16_t spd_loop_ki_int = RxMessage->motor[i].data[4] << 8 | RxMessage->motor[i].data[5];
                    rv_motor_msg[motor_id].spd_loop_kp = spd_loop_kp_int / 100000.0;
                    rv_motor_msg[motor_id].spd_loop_ki = spd_loop_ki_int / 100000.0;
                }
                // 位置环 KP 和 KD
                else if (motor_param_type == MotorParamType::PARAM_POS_LOOP_KP_KD && motor_comm_fbd.ins_code == static_cast<uint8_t>(MotorParamType::PARAM_POS_LOOP_KP_KD) && RxMessage->motor[i].dlc == 6)
                {
                    uint16_t pos_loop_kp_int = RxMessage->motor[i].data[2] << 8 | RxMessage->motor[i].data[3];
                    uint16_t pos_loop_kd_int = RxMessage->motor[i].data[4] << 8 | RxMessage->motor[i].data[5];
                    rv_motor_msg[motor_id].pos_loop_kp = pos_loop_kp_int / 100000.0;
                    rv_motor_msg[motor_id].pos_loop_kd = pos_loop_kd_int / 100000.0;
                }
                // 扭矩系数 Kt 校准使能状态
                else if (motor_param_type == MotorParamType::PARAM_TOR_COE_KT && motor_comm_fbd.ins_code == static_cast<uint8_t>(MotorParamType::PARAM_TOR_COE_KT) && RxMessage->motor[i].dlc == 3)
                {
                    rv_motor_msg[motor_id].tor_coe_kt_enable = RxMessage->motor[i].data[2];
                }
                else
                {
                    return 0;
                }
            }
            else
            {
                return 0;
            }
            // return ack_status;
        }
        // else if (comm_mode == 0x01 && RxMessage->motor[i].dlc != 0) // 自动反馈模式
        // {
        //     motor_id = RxMessage->motor[i].id - 0x205;
        //     rv_motor_msg[motor_id].motor_id = RxMessage->motor[i].id;
        //     rv_motor_msg[motor_id].angle_actual_int = (uint16_t)(RxMessage->motor[i].data[0] << 8 | RxMessage->motor[i].data[1]);
        //     rv_motor_msg[motor_id].speed_actual_int = (int16_t)(RxMessage->motor[i].data[2] << 8 | RxMessage->motor[i].data[3]);
        //     rv_motor_msg[motor_id].current_actual_int = (RxMessage->motor[i].data[4] << 8 | RxMessage->motor[i].data[5]);
        //     rv_motor_msg[motor_id].temp_motor = RxMessage->motor[i].data[6];
        //     rv_motor_msg[motor_id].error = RxMessage->motor[i].data[7];
        //     ack_status = 6;
        // }
    }
    return ack_status;
}

// 打印CAN数据
void MotorControl::rv_message_print(uint8_t ack_status)
{
    if (ack_status >= 100)
    {
        if (motor_comm_fbd.motor_fbd == 0x01)
        {
            printf("自动模式\n");
        }
        else if (motor_comm_fbd.motor_fbd == 0x02)
        {
            printf("问答模式\n");
        }
        else if (motor_comm_fbd.motor_fbd == 0x03)
        {
            printf("零点设置成功\n");
        }
        else if (motor_comm_fbd.motor_fbd == 0x04)
        {
            printf("新设置的Id为: %d\n", motor_comm_fbd.motor_id);
        }
        else if (motor_comm_fbd.motor_fbd == 0x05)
        {
            printf("重置Id成功\n");
        }
        else if (motor_comm_fbd.motor_fbd == 0x06)
        {
            printf("当前电机Id: %d\n", motor_comm_fbd.motor_id);
        }
        else if (motor_comm_fbd.motor_fbd == 0x80)
        {
            printf("查询失败\n");
        }
    }
    else
    {
        for (int i = 0; i < motor_num_; ++i)
        {
            if (rv_motor_msg[i].motor_id == 0)
            {
                continue;
            }
            switch (ack_status)
            {
            case 1:
                printf("motor id: %d\n", rv_motor_msg[i].motor_id);
                printf("angle_actual_rad: %f\n", rv_motor_msg[i].angle_actual_rad);
                printf("speed_actual_rad: %f\n", rv_motor_msg[i].speed_actual_rad);
                printf("current_actual: %f\n", rv_motor_msg[i].current_actual);
                printf("temperature: %d\n", rv_motor_msg[i].temp_motor);
                break;
            case 2:
                printf("motor id: %d\n", rv_motor_msg[i].motor_id);
                printf("angle_actual_deg: %f\n", rv_motor_msg[i].angle_actual);
                printf("current_actual: %f\n", rv_motor_msg[i].current_actual);
                printf("temperature: %d\n", rv_motor_msg[i].temp_motor);
                break;
            case 3:
                printf("motor id: %d\n", rv_motor_msg[i].motor_id);
                printf("speed_actual_rpm: %f\n", rv_motor_msg[i].speed_actual);
                printf("current_actual: %f\n", rv_motor_msg[i].current_actual);
                printf("temperature: %d\n", rv_motor_msg[i].temp_motor);
                break;
            case 4:
                if (motor_comm_fbd.motor_fbd == 1)
                {
                    printf("配置成功\n");
                }
                else
                {
                    printf("配置失败\n");
                }
                break;
            case 5:
                printf("motor id: %d\n", rv_motor_msg[i].motor_id);
                if (motor_comm_fbd.ins_code == static_cast<uint8_t>(MotorParamType::PARAM_POS))
                {
                    printf("angle_actual_deg: %f\n", rv_motor_msg[i].angle_actual);
                }
                else if (motor_comm_fbd.ins_code == static_cast<uint8_t>(MotorParamType::PARAM_SPD))
                {
                    printf("speed_actual_rpm: %f\n", rv_motor_msg[i].speed_actual);
                }
                else if (motor_comm_fbd.ins_code == static_cast<uint8_t>(MotorParamType::PARAM_CUR))
                {
                    printf("current_actual: %f\n", rv_motor_msg[i].current_actual);
                }
                else if (motor_comm_fbd.ins_code == static_cast<uint8_t>(MotorParamType::PARAM_PWR))
                {
                    printf("power: %f\n", rv_motor_msg[i].power);
                }
                else if (motor_comm_fbd.ins_code == static_cast<uint8_t>(MotorParamType::PARAM_ACC))
                {
                    printf("acceleration: %f\n", rv_motor_msg[i].acc);
                }
                else if (motor_comm_fbd.ins_code == static_cast<uint8_t>(MotorParamType::PARAM_FLUX_GAIN))
                {
                    printf("flux_gain: %d\n", rv_motor_msg[i].flux_gain);
                }
                else if (motor_comm_fbd.ins_code == static_cast<uint8_t>(MotorParamType::PARAM_DC_COE))
                {
                    printf("disturbance_compensation_coe: %d\n", rv_motor_msg[i].dc_coe);
                }
                else if (motor_comm_fbd.ins_code == static_cast<uint8_t>(MotorParamType::PARAM_FEEDBACK_GAIN))
                {
                    printf("feedback_gain: %d\n", rv_motor_msg[i].feedback_gain);
                }
                else if (motor_comm_fbd.ins_code == static_cast<uint8_t>(MotorParamType::PARAM_DAMP_COE))
                {
                    printf("damp_coe: %d\n", rv_motor_msg[i].damp_coe);
                }
                else if (motor_comm_fbd.ins_code == static_cast<uint8_t>(MotorParamType::PARAM_TOR_COE))
                {
                    printf("tor_coe: %f\n", rv_motor_msg[i].tor_coe);
                }
                else if (motor_comm_fbd.ins_code == static_cast<uint8_t>(MotorParamType::PARAM_TOR_POS_KP))
                {
                    printf("tor_pos_kp_range - kp_min: %f, kp_max: %f\n", rv_motor_msg[i].kp_min, rv_motor_msg[i].kp_max);
                }
                else if (motor_comm_fbd.ins_code == static_cast<uint8_t>(MotorParamType::PARAM_TOR_POS_KD))
                {
                    printf("tor_pos_kd_range - kd_min: %f, kd_max: %f\n", rv_motor_msg[i].kd_min, rv_motor_msg[i].kd_max);
                }
                else if (motor_comm_fbd.ins_code == static_cast<uint8_t>(MotorParamType::PARAM_TOR_POS_POS))
                {
                    printf("tor_pos_pos_range - pos_min: %f, pos_max: %f\n", rv_motor_msg[i].pos_min, rv_motor_msg[i].pos_max);
                }
                else if (motor_comm_fbd.ins_code == static_cast<uint8_t>(MotorParamType::PARAM_TOR_POS_SPD))
                {
                    printf("tor_pos_spd_range - spd_min: %f, spd_max: %f\n", rv_motor_msg[i].spd_min, rv_motor_msg[i].spd_max);
                }
                else if (motor_comm_fbd.ins_code == static_cast<uint8_t>(MotorParamType::PARAM_TOR_POS_TOR))
                {
                    printf("tor_pos_tor_range - tor_min: %f, tor_max: %f\n", rv_motor_msg[i].tor_min, rv_motor_msg[i].tor_max);
                }
                else if (motor_comm_fbd.ins_code == static_cast<uint8_t>(MotorParamType::PARAM_TOR_POS_CUR))
                {
                    printf("tor_pos_cur_range - cur_min: %f, cur_max: %f\n", rv_motor_msg[i].cur_min, rv_motor_msg[i].cur_max);
                }
                else if (motor_comm_fbd.ins_code == static_cast<uint8_t>(MotorParamType::PARAM_CAN_TIMEOUT))
                {
                    printf("can_timeout: %f\n", rv_motor_msg[i].can_timeout);
                }
                else if (motor_comm_fbd.ins_code == static_cast<uint8_t>(MotorParamType::PARAM_CUR_LOOP_KP_KI))
                {
                    printf("cur_loop: kp: %f, ki:%f\n", rv_motor_msg[i].cur_loop_kp, rv_motor_msg[i].cur_loop_ki);
                }
                else if (motor_comm_fbd.ins_code == static_cast<uint8_t>(MotorParamType::PARAM_SPD_LOOP_KP_KI))
                {
                    printf("spd_loop: kp: %f, ki:%f\n", rv_motor_msg[i].spd_loop_kp, rv_motor_msg[i].spd_loop_ki);
                }
                else if (motor_comm_fbd.ins_code == static_cast<uint8_t>(MotorParamType::PARAM_POS_LOOP_KP_KD))
                {
                    printf("pos_loop: kp: %f, kd:%f\n", rv_motor_msg[i].pos_loop_kp, rv_motor_msg[i].pos_loop_kd);
                }
                else if (motor_comm_fbd.ins_code == static_cast<uint8_t>(MotorParamType::PARAM_TOR_COE_KT))
                {
                    printf("tor_coe_kt_enable: %d\n", rv_motor_msg[i].tor_coe_kt_enable);
                }
                break;
            case 6:
                printf("motor id: %d\n", rv_motor_msg[i].motor_id);
                printf("angle_actual_int: %d\n", rv_motor_msg[i].angle_actual_int);
                printf("speed_actual_int: %d\n", rv_motor_msg[i].speed_actual_int);
                printf("current_actual_int: %d\n", rv_motor_msg[i].current_actual_int);
                printf("temperature: %d\n", rv_motor_msg[i].temp_motor);
                break;
            default:
                break;
            }
        }
    }
}
