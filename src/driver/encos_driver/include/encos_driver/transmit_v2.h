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

#include "soem/soem.h"

#define EC_TIMEOUTMON 500

#define NSEC_PER_SEC 1000000000

class EtherCATTransmit
{
public:
    EtherCATTransmit(int salve_num, int motor_num, bool show_print = false);
    ~EtherCATTransmit();

    EtherCATTransmit(const EtherCATTransmit &) = delete;
    EtherCATTransmit &operator=(const EtherCATTransmit &) = delete;

    // 初始化
    bool initialize(const std::string &ifname);

private:
    /* add ns to ec_timet */
    void add_time_ns(ec_timet *ts, int64 addtime);
    /* PI calculation to get linux time synced to DC time */
    void ec_sync(int64 reftime, int64 cycletime, int64 *offsettime);
    // Cyclic RT
    static OSAL_THREAD_FUNC ethercat_rt_thread(void *ptr);
    // Cyclic RT
    void ethercat_rt();
    // 从站故障检查
    static OSAL_THREAD_FUNC ethercat_check_thread(void *ptr);
    // 从站故障检查（主体）
    void ethercat_check();
    // 反初始化
    void ethercat_uninit();
    // 初始化
    bool ethercat_init(char *ifname);
    // 单次初始化
    int ethercat_init_once(const char *ifname);

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

public:
    std::vector<std::unique_ptr<boost::lockfree::spsc_queue<EtherCAT_Msg_ptr, boost::lockfree::capacity<10>>>> messages_;
    std::shared_ptr<MotorControl> motor_control_;
    std::vector<EtherCAT_Msg_ptr> ec_msgs_; // ethercat消息指针数组
    std::vector<std::unique_ptr<std::shared_mutex>> msg_mutexes_;

    int slave_num_;
    int motor_num_;

private:
    rclcpp::Logger logger_; // logger对象

    uint8 IOmap[4096];
    OSAL_THREAD_HANDLE thread_rt_, thread_check_;
    int expectedWKC;
    volatile int wkc;
    int mappingdone, dorun, inOP, dowkccheck;
    uint8_t currentgroup = 0;
    int cycle = 0;
    int64_t cycletime = 1000000;

    ecx_contextt ctx;

    float pgain = 0.01f;
    float igain = 0.00002f;
    /* set linux sync point 500us later than DC sync, just as example */
    int64 syncoffset = 500000;
    int64 timeerror;

    std::string ifname_;

    std::vector<EtherCAT_Msg> Rx_Message;
    std::vector<EtherCAT_Msg> Tx_Message;

    std::atomic<bool> is_check{true};
    std::atomic<bool> is_rt{true};

    std::vector<bool> isConfig;
    std::atomic<bool> is_running{false};
    std::thread thread_run;

    bool show_print_;
};

#endif