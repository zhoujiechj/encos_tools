#include "ec_client.h"
#include <iomanip>

enum ParamIndex
{
    KP_RANGE = 0,
    KD_RANGE,
    POS_RANGE,
    SPD_RANGE,
    TOR_RANGE,
    CUR_RANGE,
    PARAM_COUNT
};

std::vector<int> g_param_vec = {23, 24, 25, 26, 27, 28};

std::map<int, std::vector<int>> g_param_map{
    {1, {0, 1, 13}},
    {2, {0, 2, 14}},
    {3, {0, 3, 15}},
    {4, {0, 4, 16}},
	{5, {0, 5, 17}},
    {6, {0, 6, 23}},
    {7, {1, 1, 18}},
    {8, {1, 2, 19}},
    {9, {1, 3, 20}},
    {10, {1, 4, 21}},
	{11, {1, 5, 22}},
    {12, {2, 1, 1}},
    {13, {2, 2, 2}},
    {14, {2, 3, 3}},
    {15, {2, 4, 4}},
    {16, {2, 5, 5}},
    {17, {2, 6, 6}},
    {18, {3, 1, 7}},
    {19, {3, 2, 8}},
    {20, {3, 3, 9}},
    {21, {3, 4, 10}},
    {22, {3, 5, 11}},
    {23, {3, 6, 12}},
};

ec_client::ec_client()
    : Node("ec_client")
{
    rclcpp::QoS qos(10);
    qos.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
    qos.durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);

    // 发布电机ID获取请求
    pub_motor_get_id_ = this->create_publisher<encos_driver::msg::MotorGetId>("/motor_get_id", qos);
    // 发布电机ID设置请求
    pub_motor_set_id_ = this->create_publisher<encos_driver::msg::MotorSetId>("/motor_set_id", qos);
    // 发布电机ID重置请求
    pub_motor_reset_id_ = this->create_publisher<encos_driver::msg::MotorResetId>("/motor_reset_id", qos);
    // 发布电机参数获取请求
    pub_motor_get_param_ = this->create_publisher<encos_driver::msg::MotorGetParam>("/motor_get_param", qos);
    // 发布电机参数设置请求
    pub_motor_set_param_ = this->create_publisher<encos_driver::msg::MotorSetParam>("/motor_set_param", qos);
    // 发布电机零位设置请求
    pub_motor_set_zero_ = this->create_publisher<encos_driver::msg::MotorSetZero>("/motor_set_zero", qos);
    // 发布电机速度设置请求
    pub_motor_set_speed_ = this->create_publisher<encos_driver::msg::MotorSetSpeed>("/motor_set_speed", qos);
    // 发布电机位置设置请求
    pub_motor_set_pos_ = this->create_publisher<encos_driver::msg::MotorSetPos>("/motor_set_pos", qos);
}

ec_client::~ec_client()
{
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<ec_client>();

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);

    int opt;
    std::string type;
    int ec_id = 0;
    int slave_id = 0;
    int passage = 1;
    int motor_id = 1;
    int motor_id_old = 1;
    int motor_id_new = 1;
    int code = 0;
    float param_a = 0.0;
    float param_b = 0.0;
    double kp = 0.0;
    double kd = 0.0;
    double position = 0.0;
    double speed = 0.0;
    double torque = 0.0;
    double current = 0.0;
    double angle_positive = 0.0;  // 正转的角度（度）
    double angle_negative = 0.0;  // 负转的角度（度）
    int swing_interval = 1000;    // 正负旋转之间的间隔时间（毫秒）
    int cycle_interval = 1000;    // 每次正负旋转完成后的间隔时间（毫秒）
    int cycle_count = 1;          // 循环次数

    // 短选项
    while ((opt = getopt(argc, argv, "t:e:s:p:m:o:n:c:k:d:l:v:u:i:A:B:P:N:w:y:r:")) != -1)
    {
        switch (opt)
        {
        case 't':
            type = optarg;
            break;
        case 'e':
            ec_id = std::stoi(optarg);
            break;
        case 's':
            slave_id = std::stoi(optarg);
            break;
        case 'p':
            passage = std::stoi(optarg);
            break;
        case 'm':
            motor_id = std::stoi(optarg);
            break;
        case 'o':
            motor_id_old = std::stoi(optarg);
            break;
        case 'n':
            motor_id_new = std::stoi(optarg);
            break;
        case 'c':
            code = std::stoi(optarg);
            break;
        case 'k':
            kp = std::stod(optarg);
            break;
        case 'd':
            kd = std::stod(optarg);
            break;
        case 'l':
            position = std::stod(optarg);
            break;
        case 'v':
            speed = std::stod(optarg);
            break;
        case 'u':
            torque = std::stod(optarg);
            break;
        case 'i':
            current = std::stod(optarg);
            break;
        case 'A':
            param_a = std::stod(optarg);
            break;
        case 'B':
            param_b = std::stod(optarg);
            break;
        case 'P':
            angle_positive = std::stod(optarg);
            break;
        case 'N':
            angle_negative = std::stod(optarg);
            break;
        case 'w':
            swing_interval = std::stoi(optarg);
            break;
        case 'y':
            cycle_interval = std::stoi(optarg);
            break;
        case 'r':
            cycle_count = std::stoi(optarg);
            break;
        default:
            std::cerr
                << "Usage:\n"
                << " : -t get_id -e [ec_id] -s <slave_id>\n"
                << " : -t set_id -e [ec_id] -s <slave_id> -o <motor_id_old> -n <motor_id_new>\n"
                << " : -t reset_id -e [ec_id] -s <slave_id>\n"
                << " : -t set_zero -e [ec_id] -s <slave_id> -m <motor_id>\n"
                << " : -t get_param -e [ec_id] -s <slave_id> -p <passage> -m <motor_id> -c  <query_code>\n"
                << " : -t set_param -e [ec_id] -s <slave_id> -p <passage> -m <motor_id> -c  <config_code> -A <param_1> -B <param_2>\n"
                << " : -t set_tor_pos -e [ec_id] -s <slave_id> -p <passage> -m <motor_id> -k <kp> -d <kd> -l <pos> -v <speed> -u <torque>\n"
                << " : -t set_pos -e [ec_id] -s <slave_id> -p <passage> -m <motor_id> -l <pos> -v <speed> -i <current>\n"
                << " : -t set_spd -e [ec_id] -s <slave_id> -p <passage> -m <motor_id> -v <speed> -i <current>\n"
                << " : -t stop_all -e [ec_id] -s <slave_id>\n"
                << " : -t clear_buff -e [ec_id] -s <slave_id> -p <passage> -m <motor_id>\n"
                << " : -t leg_joint_cmd -e [ec_id] -k <kp> -d <kd> -l <pos> -v <speed> -u <torque>\n"
                << " : -t loop_param\n"
                << " : -t swing_test -e [ec_id] -s <slave_id> -p <passage> -m <motor_id> -l <init_pos> -v <speed> -i <current> -P <angle_positive> -N <angle_negative> -w <swing_interval_ms> -y <cycle_interval_ms> -r <cycle_count>\n";
            return 1;
        }
    }

    // 获取电机ID
    if (type == "get_id")
    {
        encos_driver::msg::MotorGetId msg;
        msg.ec_id = ec_id;
        msg.slave_id = slave_id;
        node->pub_motor_get_id_->publish(msg);
    }
    // 设置电机ID
    else if (type == "set_id")
    {
        encos_driver::msg::MotorSetId msg;
        msg.ec_id = ec_id;
        msg.slave_id = slave_id;
        msg.motor_id_old = motor_id_old;
        msg.motor_id_new = motor_id_new;
        node->pub_motor_set_id_->publish(msg);
    }
    // 重置电机ID
    else if (type == "reset_id")
    {
        encos_driver::msg::MotorResetId msg;
        msg.ec_id = ec_id;
        msg.slave_id = slave_id;
        node->pub_motor_reset_id_->publish(msg);
    }
    // 设置电机零点
    else if (type == "set_zero")
    {
        encos_driver::msg::MotorSetZero msg;
        msg.ec_id = ec_id;
        msg.slave_id = slave_id;
        msg.passage = passage;
        msg.motor_id = motor_id;
        node->pub_motor_set_zero_->publish(msg);
    }
    // 获取电机参数
    else if (type == "get_param")
    {
        encos_driver::msg::MotorGetParam msg;
        msg.ec_id = ec_id;
        msg.slave_id = slave_id;
        msg.passage = passage;
        msg.motor_id = motor_id;
        msg.query_code = code;
        node->pub_motor_get_param_->publish(msg);
    }
    // 设置电机参数
    else if (type == "set_param")
    {
        encos_driver::msg::MotorSetParam msg;
        msg.ec_id = ec_id;
        msg.slave_id = slave_id;
        msg.passage = passage;
        msg.motor_id = motor_id;
        msg.config_code = code;
        msg.param_a = param_a;
        msg.param_b = param_b;
        node->pub_motor_set_param_->publish(msg);
    }
    // 遍历电机参数
    else if (type == "loop_param")
    {
        for (int param : g_param_vec)
        {
            for (const auto &kv : g_param_map)
            {
                int key = kv.first;
                const std::vector<int> &vec = kv.second;
                int slave_id = vec[0];
                int passage = vec[1];
                int motor_id = vec[2];

                encos_driver::msg::MotorGetParam msg;
                msg.ec_id = ec_id;
                msg.slave_id = slave_id;
                msg.passage = passage;
                msg.motor_id = motor_id;
                msg.query_code = param;
                node->pub_motor_get_param_->publish(msg);

                std::this_thread::sleep_for(std::chrono::milliseconds(200));
            }
        }
    }
    else if (type == "set_spd")
    {
        encos_driver::msg::MotorSetSpeed msg;
        msg.ec_id = ec_id;
        msg.slave_id = slave_id;
        msg.passage = passage;
        msg.motor_id = motor_id;
        msg.speed = speed;
        msg.current = current;
        msg.ack_status = 1;
        node->pub_motor_set_speed_->publish(msg);
    }
    else if (type == "set_pos")
    {
        encos_driver::msg::MotorSetPos msg;
        msg.ec_id = ec_id;
        msg.slave_id = slave_id;
        msg.passage = passage;
        msg.motor_id = motor_id;
        msg.position = position;
        msg.speed = speed;
        msg.current = current;
        msg.ack_status = 1;
        node->pub_motor_set_pos_->publish(msg);
    }
    // 电机正负旋转测试
    else if (type == "swing_test")
    {
        std::cout << "========================================" << std::endl;
        std::cout << "电机正负旋转测试开始" << std::endl;
        std::cout << "========================================" << std::endl;
        std::cout << "测试参数:" << std::endl;
        std::cout << "  EC ID: " << ec_id << std::endl;
        std::cout << "  Slave ID: " << slave_id << std::endl;
        std::cout << "  通道: " << passage << std::endl;
        std::cout << "  电机ID: " << motor_id << std::endl;
        std::cout << "  初始位置: " << position << " 度" << std::endl;
        std::cout << "  速度: " << speed << " rpm" << std::endl;
        std::cout << "  电流: " << current << " A" << std::endl;
        std::cout << "  正转角度: +" << angle_positive << " 度" << std::endl;
        std::cout << "  负转角度: -" << angle_negative << " 度" << std::endl;
        std::cout << "  正负旋转间隔: " << swing_interval << " ms" << std::endl;
        std::cout << "  每次循环间隔: " << cycle_interval << " ms" << std::endl;
        std::cout << "  循环次数: " << cycle_count << std::endl;
        std::cout << "========================================" << std::endl;
        std::cout << std::endl;

        // 记录整体开始时间
        auto test_start = std::chrono::system_clock::now();
        auto test_start_t = std::chrono::system_clock::to_time_t(test_start);
        auto test_start_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
            test_start.time_since_epoch()) % 1000;
        std::cout << "测试开始时间: " << std::put_time(std::localtime(&test_start_t), "%Y-%m-%d %H:%M:%S") 
                 << "." << std::setfill('0') << std::setw(3) << test_start_ms.count() << std::endl;
        std::cout << std::endl;

        // 计算目标位置
        double target_pos_positive = position + angle_positive;
        double target_pos_negative = position - angle_negative;

        // 循环执行往复运动
        for (int i = 0; i < cycle_count; i++)
        {
            encos_driver::msg::MotorSetPos msg_pos;
            msg_pos.ec_id = ec_id;
            msg_pos.slave_id = slave_id;
            msg_pos.passage = passage;
            msg_pos.motor_id = motor_id;
            msg_pos.position = target_pos_positive;
            msg_pos.speed = speed;
            msg_pos.current = current;
            msg_pos.ack_status = 1;
            node->pub_motor_set_pos_->publish(msg_pos);

            // 处理ROS消息
            // for (int j = 0; j < 10; j++)
            // {
            //     executor.spin_once(std::chrono::milliseconds(10));
            // }

            // 等待正负旋转间隔时间
            std::cout << "  [等待] " << swing_interval << " ms" << std::endl;
            std::this_thread::sleep_for(std::chrono::milliseconds(swing_interval));

            // 第二步：负向旋转
            // std::cout << "  [负转] 目标位置: " << target_pos_negative << " 度" << std::endl;
            
            encos_driver::msg::MotorSetPos msg_neg;
            msg_neg.ec_id = ec_id;
            msg_neg.slave_id = slave_id;
            msg_neg.passage = passage;
            msg_neg.motor_id = motor_id;
            msg_neg.position = target_pos_negative;
            msg_neg.speed = speed;
            msg_neg.current = current;
            msg_neg.ack_status = 1;
            node->pub_motor_set_pos_->publish(msg_neg);

            // 处理ROS消息
            // for (int j = 0; j < 10; j++)
            // {
            //     executor.spin_once(std::chrono::milliseconds(10));
            // }
            std::cout << "  [等待下次循环] " << cycle_interval << " ms" << std::endl;
            std::cout << std::endl;
            std::this_thread::sleep_for(std::chrono::milliseconds(cycle_interval));
            // }
        }
    }
    // 其他
    else
    {
        std::cerr << "Unknown type: " << type << "\n";
        return 1;
    }

    auto start_time = std::chrono::steady_clock::now();
    while (rclcpp::ok() && std::chrono::steady_clock::now() - start_time < std::chrono::milliseconds(200))
    {
        executor.spin_once(std::chrono::milliseconds(100));
    }

    rclcpp::shutdown();

    return 0;
}