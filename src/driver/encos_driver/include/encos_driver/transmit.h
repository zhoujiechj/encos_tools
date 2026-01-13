#ifndef __TRANSMIT_H
#define __TRANSMIT_H

#include <inttypes.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "sys/time.h"
#include <boost/lockfree/spsc_queue.hpp>
#include <memory>
#include <atomic>
#include <thread>
#include <mutex>
#include <shared_mutex>
#include <vector>
#include "encos_driver/config.h"
#include "encos_driver/motor_control.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include "concurrentqueue/concurrentqueue.h"

extern "C"
{
#include "ethercat.h"
}

#define K_ETHERCAT_ERR_PERIOD 100
#define K_ETHERCAT_ERR_MAX 20
#define EC_TIMEOUTMON 500

class EtherCATTransmit
{
public:
    EtherCATTransmit(int salve_num, int motor_num, bool show_print=false);
    ~EtherCATTransmit();

    EtherCATTransmit(const EtherCATTransmit &) = delete;
    EtherCATTransmit &operator=(const EtherCATTransmit &) = delete;

    // 连接状态检查
    bool isConnected() const;
    // 设置连接状态回调
    void setConnectionCallback(std::function<void(bool)> callback);
    // 初始化
    bool initialize(const std::string &ifname);
    // 重新初始化
    bool reinitialize();

private:
    // 初始化
    bool ethercat_init(char *ifname);
    // 反初始化
    void ethercat_uninit();
    // 单次初始化
    int ethercat_init_once(const char *ifname);
    // 从站故障检查
    static OSAL_THREAD_FUNC ethercat_check_thread(void *ptr);
    // 从站故障检查（主体）
    void ethercat_check();

    // 处理线程
    void ethercat_run();
    // 指令设置
    void ethercat_command_set();
    // 数据获取
    void ethercat_data_get();

    // 启动线程
    void start_ethercat_run();
    // 停止线程
    void stop_ethercat_run();

    // 启动连接状态监控线程
    void start_connection_monitor();
    // 停止连接状态监控线程
    void stop_connection_monitor();
    // 连接状态监控循环
    void connection_monitor_loop();
    // 尝试重新连接EtherCAT
    bool attempt_reconnect();

public:
    // std::vector<moodycamel::ConcurrentQueue<EtherCAT_Msg_ptr>> messages_;
	std::vector<std::unique_ptr<boost::lockfree::spsc_queue<EtherCAT_Msg_ptr, boost::lockfree::capacity<10>>>> messages_;
    std::shared_ptr<MotorControl> motor_control_;
    std::vector<EtherCAT_Msg_ptr> ec_msgs_; // ethercat消息指针数组
    std::vector<std::unique_ptr<std::shared_mutex>> msg_mutexes_;

    int slave_num_;
    int motor_num_;

private:
    rclcpp::Logger logger_; // logger对象

    std::atomic<bool> is_running{false};
    std::thread runThread;

    char IOmap[4096];
    OSAL_THREAD_HANDLE checkThread;
    std::atomic<bool> is_checking{true};
    int check_thread_id = 0;
    int expectedWKC;
    bool needlf;
    volatile int wkc;
    bool inOP;
    uint8_t currentgroup = 0;
    uint64_t num;
    std::vector<bool> isConfig;

    int wkc_err_count = 0;
    int wkc_err_iteration_count = 0;
    int err_count = 0;
    int err_iteration_count = 0;

    std::vector<EtherCAT_Msg> Rx_Message;
    std::vector<EtherCAT_Msg> Tx_Message;

    std::atomic<bool> connected_{false};
    std::function<void(bool)> connection_callback_;
    std::string ifname_;
    // std::mutex reinit_mutex_;
    std::recursive_mutex reinit_mutex_;

    std::atomic<bool> initialized_{false};
    std::thread connection_monitor_thread_;            // 连接状态监控线程
    std::atomic<bool> stop_connection_monitor_{false}; // 停止连接状态监控线程
    int reconnect_attempts_{0};                        // 重连尝试次数
    const int max_reconnect_attempts_{5};              // 最大重连尝试次数
    const std::chrono::seconds reconnect_interval_{3}; // 重连间隔

    bool show_print_;
};

#endif