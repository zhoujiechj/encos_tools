#include "ec_server.hpp"

ec_server::ec_server()
    : Node("ec_server")
{
    this->declare_parameter<std::string>("net_name", "ens33");
    auto net_name = get_parameter("net_name").as_string();
    ec_map[0] = ec_info(0, net_name);
    
    // 初始化ec_map
    // ec_map[0] = ec_info(0, "ens33");
    // ec_map[1] = ec_info(1, "enx00e04c436188");
    for (std::map<int, ec_info>::iterator it = ec_map.begin(); it != ec_map.end(); ++it)
    {
        ec_info &ec = it->second;
        ec.slave_num_ = 1;
        ec.motor_num_ = 100;
        ec.ec_msgs_.resize(ec.slave_num_);
        for (int i = 0; i < ec.slave_num_; i++)
        {
            ec.ec_msgs_[i] = std::make_shared<EtherCAT_Msg>();
        }
        ec.ethercat_ = std::make_shared<EtherCATTransmit>(ec.slave_num_, ec.motor_num_, true);
        if (!ec.ethercat_->initialize(ec.net_name_))
        {
            RCLCPP_ERROR(this->get_logger(), "EtherCAT %d-%s not initialized", ec.ec_id_, ec.net_name_.c_str());
            continue;
            ;
        }
        RCLCPP_INFO(this->get_logger(), "EtherCAT %d-%s initialize success", ec.ec_id_, ec.net_name_.c_str());
    }

    rclcpp::QoS qos(10);
    qos.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
    qos.durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);

    // 初始化消息订阅
    sub_motor_get_id_ = this->create_subscription<encos_driver::msg::MotorGetId>(
        "/motor_get_id", qos, std::bind(&ec_server::motor_get_id_callback, this, std::placeholders::_1));
    sub_motor_set_id_ = this->create_subscription<encos_driver::msg::MotorSetId>(
        "/motor_set_id", qos, std::bind(&ec_server::motor_set_id_callback, this, std::placeholders::_1));
    sub_motor_reset_id_ = this->create_subscription<encos_driver::msg::MotorResetId>(
        "/motor_reset_id", qos, std::bind(&ec_server::motor_reset_id_callback, this, std::placeholders::_1));
    sub_motor_get_param_ = this->create_subscription<encos_driver::msg::MotorGetParam>(
        "/motor_get_param", qos, std::bind(&ec_server::motor_get_param_callback, this, std::placeholders::_1));
    sub_motor_set_param_ = this->create_subscription<encos_driver::msg::MotorSetParam>(
        "/motor_set_param", qos, std::bind(&ec_server::motor_set_param_callback, this, std::placeholders::_1));
    sub_motor_set_zero_ = this->create_subscription<encos_driver::msg::MotorSetZero>(
        "/motor_set_zero", qos, std::bind(&ec_server::motor_set_zero_callback, this, std::placeholders::_1));
    sub_motor_set_speed_ = this->create_subscription<encos_driver::msg::MotorSetSpeed>(
        "/motor_set_speed", qos, std::bind(&ec_server::motor_set_speed_callback, this, std::placeholders::_1));
    sub_motor_set_pos_ = this->create_subscription<encos_driver::msg::MotorSetPos>(
        "/motor_set_pos", qos, std::bind(&ec_server::motor_set_pos_callback, this, std::placeholders::_1));
    sub_motor_set_tor_pos_ = this->create_subscription<encos_driver::msg::MotorSetTorPos>(
        "/motor_set_tor_pos", qos, std::bind(&ec_server::motor_set_tor_pos_callback, this, std::placeholders::_1));
}

ec_server::~ec_server()
{
}

// 电机ID获取回调函数
void ec_server::motor_get_id_callback(const encos_driver::msg::MotorGetId::SharedPtr msg)
{
    RCLCPP_INFO(this->get_logger(), "Received motor get ID request: ec_id: %d, slave_id: %d, passage: %d",
                msg->ec_id, msg->slave_id, msg->passage);

    auto it = ec_map.find(msg->ec_id);
    if (it == ec_map.end())
    {
        RCLCPP_ERROR(this->get_logger(), "Invalid ec_id: %d", msg->ec_id);
        return;
    }

    ec_info &ec = it->second;
    if (ec.ethercat_ != nullptr)
    {
        ec.ethercat_->motor_control_->get_motor_id(ec.ec_msgs_[msg->slave_id].get(), msg->passage);
        send_to_queue(msg->ec_id, msg->slave_id, ec.ec_msgs_[msg->slave_id]);
    }
}

// 电机ID设置回调函数
void ec_server::motor_set_id_callback(const encos_driver::msg::MotorSetId::SharedPtr msg)
{
    RCLCPP_INFO(this->get_logger(), "Received motor set ID request: ec_id: %d, slave_id: %d, passage: %d, motor_id_old: %d, motor_id_new: %d",
                msg->ec_id, msg->slave_id, msg->passage, msg->motor_id_old, msg->motor_id_new);

    auto it = ec_map.find(msg->ec_id);
    if (it == ec_map.end())
    {
        RCLCPP_ERROR(this->get_logger(), "Invalid ec_id: %d", msg->ec_id);
        return;
    }

    ec_info &ec = it->second;
    if (ec.ethercat_ != nullptr)
    {
        ec.ethercat_->motor_control_->set_motor_id(ec.ec_msgs_[msg->slave_id].get(), msg->passage, msg->motor_id_old, msg->motor_id_new);
        send_to_queue(msg->ec_id, msg->slave_id, ec.ec_msgs_[msg->slave_id]);
    }
}

// 电机ID重置回调函数
void ec_server::motor_reset_id_callback(const encos_driver::msg::MotorResetId::SharedPtr msg)
{
    RCLCPP_INFO(this->get_logger(), "Received motor reset ID request: ec_id: %d, slave_id: %d, passage: %d",
                msg->ec_id, msg->slave_id, msg->passage);

    auto it = ec_map.find(msg->ec_id);
    if (it == ec_map.end())
    {
        RCLCPP_ERROR(this->get_logger(), "Invalid ec_id: %d", msg->ec_id);
        return;
    }

    ec_info &ec = it->second;
    if (ec.ethercat_ != nullptr)
    {
        ec.ethercat_->motor_control_->reset_motor_id(ec.ec_msgs_[msg->slave_id].get(), msg->passage);
        send_to_queue(msg->ec_id, msg->slave_id, ec.ec_msgs_[msg->slave_id]);
    }
}

// 电机参数获取回调函数
void ec_server::motor_get_param_callback(const encos_driver::msg::MotorGetParam::SharedPtr msg)
{
    RCLCPP_INFO(this->get_logger(), "Received motor get param request: ec_id: %d, slave_id: %d, passage: %d, motor_id: %d, query_code: %d",
                msg->ec_id, msg->slave_id, msg->passage, msg->motor_id, msg->query_code);

    auto it = ec_map.find(msg->ec_id);
    if (it == ec_map.end())
    {
        RCLCPP_ERROR(this->get_logger(), "Invalid ec_id: %d", msg->ec_id);
        return;
    }

    ec_info &ec = it->second;
    if (ec.ethercat_ != nullptr)
    {
        ec.ethercat_->motor_control_->get_motor_param(ec.ec_msgs_[msg->slave_id].get(), msg->passage, msg->motor_id, msg->query_code);
        send_to_queue(msg->ec_id, msg->slave_id, ec.ec_msgs_[msg->slave_id]);
    }
}

// 电机参数设置回调函数
void ec_server::motor_set_param_callback(const encos_driver::msg::MotorSetParam::SharedPtr msg)
{
    RCLCPP_INFO(this->get_logger(), "Received motor set param request: ec_id: %d, slave_id: %d, passage: %d, motor_id: %d, config_code: %d, param_a: %f, param_b: %f",
                msg->ec_id, msg->slave_id, msg->passage, msg->motor_id, msg->config_code, msg->param_a, msg->param_b);

    auto it = ec_map.find(msg->ec_id);
    if (it == ec_map.end())
    {
        RCLCPP_ERROR(this->get_logger(), "Invalid ec_id: %d", msg->ec_id);
        return;
    }

    ec_info &ec = it->second;
    if (ec.ethercat_ != nullptr)
    {
        // 配置力位混控参数 - KP的最小值和最大值
        if (msg->config_code == 0x05)
        {
            float kp_min = msg->param_a;
            float kp_max = msg->param_b;
            ec.ethercat_->motor_control_->set_motor_tor_pos_config_kp(ec.ec_msgs_[msg->slave_id].get(), msg->passage, msg->motor_id, kp_min, kp_max);
            RCLCPP_INFO(this->get_logger(), "Config tor pos kp - slave_id: %d, passage: %d, motor_id: %d, config_code: %d, kp_min: %f, kp_max: %f",
                        msg->slave_id, msg->passage, msg->motor_id, msg->config_code, kp_min, kp_max);
        }
        // 配置力位混控参数 - KD的最小值和最大值
        else if (msg->config_code == 0x06)
        {
            float kd_min = msg->param_a;
            float kd_max = msg->param_b;
            ec.ethercat_->motor_control_->set_motor_tor_pos_config_kd(ec.ec_msgs_[msg->slave_id].get(), msg->passage, msg->motor_id, kd_min, kd_max);
            RCLCPP_INFO(this->get_logger(), "Config tor pos kd - slave_id: %d, passage: %d, motor_id: %d, config_code: %d, kd_min: %f, kd_max: %f",
                        msg->slave_id, msg->passage, msg->motor_id, msg->config_code, kd_min, kd_max);
        }
        // 配置力位混控参数 - 位置的最小值和最大值
        else if (msg->config_code == 0x07)
        {
            float pos_min = msg->param_a;
            float pos_max = msg->param_b;
            ec.ethercat_->motor_control_->set_motor_tor_pos_config_pos(ec.ec_msgs_[msg->slave_id].get(), msg->passage, msg->motor_id, pos_min, pos_max);
            RCLCPP_INFO(this->get_logger(), "Config tor pos pos - slave_id: %d, passage: %d, motor_id: %d, config_code: %d, pos_min: %f, pos_max: %f",
                        msg->slave_id, msg->passage, msg->motor_id, msg->config_code, pos_min, pos_max);
        }
        // 配置力位混控参数 - 速度的最小值和最大值
        else if (msg->config_code == 0x08)
        {
            float spd_min = msg->param_a;
            float spd_max = msg->param_b;
            ec.ethercat_->motor_control_->set_motor_tor_pos_config_spd(ec.ec_msgs_[msg->slave_id].get(), msg->passage, msg->motor_id, spd_min, spd_max);
            RCLCPP_INFO(this->get_logger(), "Config tor pos spd - slave_id: %d, passage: %d, motor_id: %d, config_code: %d, spd_min: %f, spd_max: %f",
                        msg->slave_id, msg->passage, msg->motor_id, msg->config_code, spd_min, spd_max);
        }
        // 配置力位混控参数 - 扭矩的最小值和最大值
        else if (msg->config_code == 0x09)
        {
            float tor_min = msg->param_a;
            float tor_max = msg->param_b;
            ec.ethercat_->motor_control_->set_motor_tor_pos_config_tor(ec.ec_msgs_[msg->slave_id].get(), msg->passage, msg->motor_id, tor_min, tor_max);
            RCLCPP_INFO(this->get_logger(), "Config tor pos tor - slave_id: %d, passage: %d, motor_id: %d, config_code: %d, tor_min: %f, tor_max: %f",
                        msg->slave_id, msg->passage, msg->motor_id, msg->config_code, tor_min, tor_max);
        }
        // 配置力位混控参数 - 电流的最小值和最大值
        else if (msg->config_code == 0x0A)
        {
            float cur_min = msg->param_a;
            float cur_max = msg->param_b;
            ec.ethercat_->motor_control_->set_motor_tor_pos_config_cur(ec.ec_msgs_[msg->slave_id].get(), msg->passage, msg->motor_id, cur_min, cur_max);
            RCLCPP_INFO(this->get_logger(), "Config tor pos cur - slave_id: %d, passage: %d, motor_id: %d, config_code: %d, cur_min: %f, cur_max: %f",
                        msg->slave_id, msg->passage, msg->motor_id, msg->config_code, cur_min, cur_max);
        }
        // 配置CAN超时时间
        else if (msg->config_code == 0x0B)
        {
            uint timeout = msg->param_a;
            ec.ethercat_->motor_control_->set_motor_can_timeout(ec.ec_msgs_[msg->slave_id].get(), msg->passage, msg->motor_id, timeout);
            RCLCPP_INFO(this->get_logger(), "Config can timeout - slave_id: %d, passage: %d, motor_id: %d, config_code: %d, timeout: %d",
                        msg->slave_id, msg->passage, msg->motor_id, msg->config_code, timeout);
        }
        // 配置电流环PI参数
        else if (msg->config_code == 0x0C)
        {
            float kp = msg->param_a;
            float ki = msg->param_b;
            ec.ethercat_->motor_control_->set_motor_current_loop_pi(ec.ec_msgs_[msg->slave_id].get(), msg->passage, msg->motor_id, kp, ki);
            RCLCPP_INFO(this->get_logger(), "Config current loop pi - slave_id: %d, passage: %d, motor_id: %d, config_code: %d, kp: %f, ki: %f",
                        msg->slave_id, msg->passage, msg->motor_id, msg->config_code, kp, ki);
        }
        // 配置速度环PI参数
        else if (msg->config_code == 0x0D)
        {
            float kp = msg->param_a;
            float ki = msg->param_b;
            ec.ethercat_->motor_control_->set_motor_speed_loop_pi(ec.ec_msgs_[msg->slave_id].get(), msg->passage, msg->motor_id, kp, ki);
            RCLCPP_INFO(this->get_logger(), "Config speed loop pi - slave_id: %d, passage: %d, motor_id: %d, config_code: %d, kp: %f, ki: %f",
                        msg->slave_id, msg->passage, msg->motor_id, msg->config_code, kp, ki);
        }
        // 配置位置环PD参数
        else if (msg->config_code == 0x0E)
        {
            float kp = msg->param_a;
            float kd = msg->param_b;
            ec.ethercat_->motor_control_->set_motor_position_loop_pd(ec.ec_msgs_[msg->slave_id].get(), msg->passage, msg->motor_id, kp, kd);
            RCLCPP_INFO(this->get_logger(), "Config position loop pd - slave_id: %d, passage: %d, motor_id: %d, config_code: %d, kp: %f, kd: %f",
                        msg->slave_id, msg->passage, msg->motor_id, msg->config_code, kp, kd);
        }

        send_to_queue(msg->ec_id, msg->slave_id, ec.ec_msgs_[msg->slave_id]);
    }
}

// 电机零位设置回调函数
void ec_server::motor_set_zero_callback(const encos_driver::msg::MotorSetZero::SharedPtr msg)
{
    RCLCPP_INFO(this->get_logger(), "Received motor set zero request: ec_id:%d, slave_id: %d, passage: %d, motor_id: %d",
                msg->ec_id, msg->slave_id, msg->passage, msg->motor_id);

    auto it = ec_map.find(msg->ec_id);
    if (it == ec_map.end())
    {
        RCLCPP_ERROR(this->get_logger(), "Invalid ec_id: %d", msg->ec_id);
        return;
    }

    ec_info &ec = it->second;
    if (ec.ethercat_ != nullptr)
    {
        ec.ethercat_->motor_control_->set_motor_zero(ec.ec_msgs_[msg->slave_id].get(), msg->passage, msg->motor_id);
        send_to_queue(msg->ec_id, msg->slave_id, ec.ec_msgs_[msg->slave_id]);
    }
}

// 电机速度回调函数
void ec_server::motor_set_speed_callback(const encos_driver::msg::MotorSetSpeed::SharedPtr msg)
{
    RCLCPP_INFO(this->get_logger(), "Received motor set speed request: ec_id:%d, slave_id: %d, passage: %d, motor_id: %d, speed: %f, current: %f",
                msg->ec_id, msg->slave_id, msg->passage, msg->motor_id, msg->speed, msg->current);

    auto it = ec_map.find(msg->ec_id);
    if (it == ec_map.end())
    {
        RCLCPP_ERROR(this->get_logger(), "Invalid ec_id: %d", msg->ec_id);
        return;
    }

    ec_info &ec = it->second;
    if (ec.ethercat_ != nullptr)
    {
        ec.ethercat_->motor_control_->set_motor_speed(ec.ec_msgs_[msg->slave_id].get(), msg->passage, msg->motor_id, msg->speed, msg->current, 1);
        send_to_queue(msg->ec_id, msg->slave_id, ec.ec_msgs_[msg->slave_id]);
    }
}

// 电机位置回调函数
void ec_server::motor_set_pos_callback(const encos_driver::msg::MotorSetPos::SharedPtr msg)
{
    RCLCPP_INFO(this->get_logger(), "Received motor set speed request: ec_id:%d, slave_id: %d, passage: %d, motor_id: %d, position: %f, speed: %f, current: %f",
                msg->ec_id, msg->slave_id, msg->passage, msg->motor_id, msg->position, msg->speed, msg->current);

    auto it = ec_map.find(msg->ec_id);
    if (it == ec_map.end())
    {
        RCLCPP_ERROR(this->get_logger(), "Invalid ec_id: %d", msg->ec_id);
        return;
    }

    ec_info &ec = it->second;
    if (ec.ethercat_ != nullptr)
    {
        ec.ethercat_->motor_control_->set_motor_pos(ec.ec_msgs_[msg->slave_id].get(), msg->passage, msg->motor_id, msg->position, msg->speed, msg->current, 1);
        send_to_queue(msg->ec_id, msg->slave_id, ec.ec_msgs_[msg->slave_id]);
    }
}

// 电机力位混控回调函数
void ec_server::motor_set_tor_pos_callback(const encos_driver::msg::MotorSetTorPos::SharedPtr msg)
{
    RCLCPP_INFO(this->get_logger(), "Received motor set tor pos request: ec_id:%d, slave_id: %d, passage: %d, motor_id: %d, kp: %f, kd: %f, position: %f, speed: %f, torque: %f",
                msg->ec_id, msg->slave_id, msg->passage, msg->motor_id, msg->kp, msg->kd, msg->position, msg->speed, msg->torque);

    auto it = ec_map.find(msg->ec_id);
    if (it == ec_map.end())
    {
        RCLCPP_ERROR(this->get_logger(), "Invalid ec_id: %d", msg->ec_id);
        return;
    }

    ec_info &ec = it->second;
    if (ec.ethercat_ != nullptr)
    {
        ec.ethercat_->motor_control_->set_motor_tor_pos(ec.ec_msgs_[msg->slave_id].get(), msg->passage, msg->motor_id, msg->kp, msg->kd, msg->position, msg->speed, msg->torque);
        send_to_queue(msg->ec_id, msg->slave_id, ec.ec_msgs_[msg->slave_id]);
    }
}

// 推送至发送队列
void ec_server::send_to_queue(int ec_id, int slave_id, const EtherCAT_Msg_ptr &msg)
{
    auto it = ec_map.find(ec_id);
    if (it == ec_map.end())
    {
        RCLCPP_ERROR(this->get_logger(), "Invalid ec_id: %d", ec_id);
        return;
    }

    ec_info &ec = it->second;

    // constexpr size_t MAX_QUEUE_SIZE = 10;
    // if (ec.ethercat_->messages_[slave_id].size_approx() >= MAX_QUEUE_SIZE)
    // {
    //     RCLCPP_WARN(this->get_logger(), "Queue for slave %d is full, waiting...", slave_id);
    //     std::this_thread::sleep_for(std::chrono::milliseconds(10));
    //     return;
    // }

    // while (!ec.ethercat_->messages_[slave_id].try_enqueue(msg))
    // {
    //     // 理论上不会失败，除非内存不足
    //     RCLCPP_ERROR(this->get_logger(), "Failed to enqueue message for slave %d", slave_id);
    //     std::this_thread::sleep_for(std::chrono::milliseconds(10));
    // }

    if (ec.ethercat_->messages_[slave_id]->write_available())
    {
        ec.ethercat_->messages_[slave_id]->push(msg);
    }
    else
    {
        RCLCPP_WARN(this->get_logger(), "Queue Fulled, Waiting For Command Executing.");
        while (ec.ethercat_->messages_[slave_id]->push(msg))
        {
            sleep(1);
        }
    }
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<ec_server>();

    rclcpp::spin(node);

    return 0;
}
