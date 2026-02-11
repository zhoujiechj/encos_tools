#ifndef __EC_SERVER_H
#define __EC_SERVER_H

#include <string>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <iostream>
#include <sys/time.h>
#include <thread>
#include <unistd.h>
#include <map>

#include "encos_driver/transmit.h"
#include "rclcpp/rclcpp.hpp"

#include "encos_driver/msg/motor_get_id.hpp"
#include "encos_driver/msg/motor_set_id.hpp"
#include "encos_driver/msg/motor_reset_id.hpp"
#include "encos_driver/msg/motor_get_param.hpp"
#include "encos_driver/msg/motor_set_param.hpp"
#include "encos_driver/msg/motor_set_zero.hpp"
#include "encos_driver/msg/motor_set_speed.hpp"
#include "encos_driver/msg/motor_set_pos.hpp"
#include "encos_driver/msg/motor_set_tor_pos.hpp"

class ec_info
{
public:
    ec_info() {}
    ec_info(int ec_id, std::string net_name)
    {
        this->ec_id_ = ec_id;
        this->net_name_ = net_name;
    }

public:
    int ec_id_;                                  // ethercat网口id
    std::string net_name_;                       // 网口名称
    int slave_num_;                              // ethercat板子数
    int motor_num_;                              // 电机数量
    std::shared_ptr<EtherCATTransmit> ethercat_; // ethercat通信对象
    std::vector<EtherCAT_Msg_ptr> ec_msgs_;      // ethercat消息队列
};

class ec_server : public rclcpp::Node
{
public:
    ec_server();
    ~ec_server();

private:
    // 电机ID获取回调函数
    void motor_get_id_callback(const encos_driver::msg::MotorGetId::SharedPtr msg);
    // 电机ID设置回调函数
    void motor_set_id_callback(const encos_driver::msg::MotorSetId::SharedPtr msg);
    // 电机ID重置回调函数
    void motor_reset_id_callback(const encos_driver::msg::MotorResetId::SharedPtr msg);
    // 电机参数获取回调函数
    void motor_get_param_callback(const encos_driver::msg::MotorGetParam::SharedPtr msg);
    // 电机参数设置回调函数
    void motor_set_param_callback(const encos_driver::msg::MotorSetParam::SharedPtr msg);
    // 电机零位设置回调函数
    void motor_set_zero_callback(const encos_driver::msg::MotorSetZero::SharedPtr msg);
    // 电机速度回调函数
    void motor_set_speed_callback(const encos_driver::msg::MotorSetSpeed::SharedPtr msg);
    // 电机位置回调函数
    void motor_set_pos_callback(const encos_driver::msg::MotorSetPos::SharedPtr msg);
    // 电机力位混控回调函数
    void motor_set_tor_pos_callback(const encos_driver::msg::MotorSetTorPos::SharedPtr msg);

    // 推送至发送队列
    void send_to_queue(int ec_id, int slave_id, const EtherCAT_Msg_ptr &msg);

private:
    std::map<int, ec_info> ec_map; // ethercat网口id到ec_info的映射

    rclcpp::Subscription<encos_driver::msg::MotorGetId>::SharedPtr sub_motor_get_id_;       // 订阅获取电机ID消息
    rclcpp::Subscription<encos_driver::msg::MotorSetId>::SharedPtr sub_motor_set_id_;       // 订阅设置电机ID消息
    rclcpp::Subscription<encos_driver::msg::MotorResetId>::SharedPtr sub_motor_reset_id_;   // 订阅重置电机ID消息
    rclcpp::Subscription<encos_driver::msg::MotorGetParam>::SharedPtr sub_motor_get_param_; // 订阅获取电机参数消息
    rclcpp::Subscription<encos_driver::msg::MotorSetParam>::SharedPtr sub_motor_set_param_; // 订阅设置电机参数消息
    rclcpp::Subscription<encos_driver::msg::MotorSetZero>::SharedPtr sub_motor_set_zero_;   // 订阅设置电机零位消息
    rclcpp::Subscription<encos_driver::msg::MotorSetSpeed>::SharedPtr sub_motor_set_speed_; // 订阅设置电机速度消息
    rclcpp::Subscription<encos_driver::msg::MotorSetPos>::SharedPtr sub_motor_set_pos_;     // 订阅设置电机位置消息
    rclcpp::Subscription<encos_driver::msg::MotorSetTorPos>::SharedPtr sub_motor_set_tor_pos_; // 订阅设置电机力位混控消息
};

#endif
