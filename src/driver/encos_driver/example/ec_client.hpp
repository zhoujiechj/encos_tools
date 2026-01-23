#ifndef __EC_CLIENT_H
#define __EC_CLIENT_H

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
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include <rclcpp/executors/single_threaded_executor.hpp>

#include "encos_driver/msg/motor_get_id.hpp"
#include "encos_driver/msg/motor_set_id.hpp"
#include "encos_driver/msg/motor_reset_id.hpp"
#include "encos_driver/msg/motor_get_param.hpp"
#include "encos_driver/msg/motor_set_param.hpp"
#include "encos_driver/msg/motor_set_zero.hpp"
#include "encos_driver/msg/motor_set_speed.hpp"
#include "encos_driver/msg/motor_set_pos.hpp"


class ec_client : public rclcpp::Node
{
public:
    ec_client();
    ~ec_client();

public:
    rclcpp::Publisher<encos_driver::msg::MotorGetId>::SharedPtr pub_motor_get_id_;       // 发布电机ID获取请求
    rclcpp::Publisher<encos_driver::msg::MotorSetId>::SharedPtr pub_motor_set_id_;       // 发布电机ID设置请求
    rclcpp::Publisher<encos_driver::msg::MotorResetId>::SharedPtr pub_motor_reset_id_;   // 发布电机ID重置请求
    rclcpp::Publisher<encos_driver::msg::MotorGetParam>::SharedPtr pub_motor_get_param_; // 发布电机参数获取请求
    rclcpp::Publisher<encos_driver::msg::MotorSetParam>::SharedPtr pub_motor_set_param_; // 发布电机参数设置请求
    rclcpp::Publisher<encos_driver::msg::MotorSetZero>::SharedPtr pub_motor_set_zero_;   // 发布电机零位设置请求
    rclcpp::Publisher<encos_driver::msg::MotorSetSpeed>::SharedPtr pub_motor_set_speed_; // 发布电机速度设置请求
    rclcpp::Publisher<encos_driver::msg::MotorSetPos>::SharedPtr pub_motor_set_pos_;     // 发布电机位置设置请求
};

#endif
