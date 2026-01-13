#include "encos_driver/transmit.h"

EtherCATTransmit::EtherCATTransmit(int slave_num, int motor_num, bool show_print)
    : logger_(rclcpp::get_logger("encos_driver")),
      connected_(false),
      initialized_(false),
      slave_num_(slave_num),
      motor_num_(motor_num),
      show_print_(show_print)
{
    // 初始化电机控制
    motor_control_ = std::make_shared<MotorControl>(motor_num_);

    // 初始化消息队列
    messages_.resize(slave_num_);
    for (size_t i = 0; i < slave_num_; ++i) 
    {
        messages_[i] = std::make_unique<boost::lockfree::spsc_queue<EtherCAT_Msg_ptr, boost::lockfree::capacity<10>>>();
    }

    // 初始化指令数组
    ec_msgs_.resize(slave_num_);
    msg_mutexes_.resize(slave_num_);
    for (int i = 0; i < slave_num_; i++)
    {
        ec_msgs_[i] = std::make_shared<EtherCAT_Msg>();
        msg_mutexes_[i] = std::make_unique<std::shared_mutex>();
    }

    // 初始化配置状态
    isConfig.resize(slave_num_, false);

    // 初始化接收和发送消息
    Rx_Message.resize(slave_num_);
    Tx_Message.resize(slave_num_);
}

EtherCATTransmit::~EtherCATTransmit()
{
    // 停止线程
    stop_ethercat_run();

    // 反初始化
    ethercat_uninit();

    // 停止连接监控
    stop_connection_monitor();
}

// 检查连接状态
bool EtherCATTransmit::isConnected() const
{
    return connected_ && inOP;
}

// 设置连接状态回调
void EtherCATTransmit::setConnectionCallback(std::function<void(bool)> callback)
{
    connection_callback_ = callback;
}

// 初始化
bool EtherCATTransmit::initialize(const std::string &ifname)
{
    ifname_ = ifname;

    if (isConnected())
    {
        return true;
    }

    // EtherCAT 初始化代码
    if (ethercat_init((char *)(ifname.c_str())))
    {
        is_checking = true;
        connected_ = inOP;

        // 通知连接状态变化
        if (connection_callback_)
        {
            connection_callback_(connected_);
        }

        // 启动线程
        start_ethercat_run();

        if (!initialized_)
        {
            // 启动连接监控
            // start_connection_monitor();
        }

        initialized_ = true;

        return true;
    }

    return false;
}

// 重新初始化
bool EtherCATTransmit::reinitialize()
{
    // std::lock_guard<std::mutex> lock(reinit_mutex_);
    std::lock_guard<std::recursive_mutex> lock(reinit_mutex_);

    RCLCPP_WARN(logger_, "[EtherCAT] Attempting to reinitialize...");

    // 停止当前连接
    stop_ethercat_run();
    // 反初始化
    ethercat_uninit();

    connected_ = false;
    is_checking = true;

    if (ifname_.empty())
    {
        RCLCPP_ERROR(logger_, "[EtherCAT] No network device name saved for reinitialization");
        return false;
    }

    return initialize(ifname_);
}

// 启动连接监控
void EtherCATTransmit::start_connection_monitor()
{
    stop_connection_monitor_ = false;
    connection_monitor_thread_ = std::thread([this]()
                                             { connection_monitor_loop(); });
}

// 停止连接监控
void EtherCATTransmit::stop_connection_monitor()
{
    stop_connection_monitor_ = true;
    if (connection_monitor_thread_.joinable())
    {
        connection_monitor_thread_.join();
    }
}

// 连接监控循环
void EtherCATTransmit::connection_monitor_loop()
{
    while (!stop_connection_monitor_)
    {
        // 检查连接状态
        if (!connected_)
        {
            RCLCPP_WARN(logger_, "EtherCAT disconnected, attempting to reconnect...");

            if (attempt_reconnect())
            {
                RCLCPP_INFO(logger_, "EtherCAT reconnected successfully");
            }
            else
            {
                RCLCPP_ERROR(logger_, "EtherCAT reconnection failed");
            }
        }

        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
}

// 尝试重新连接
bool EtherCATTransmit::attempt_reconnect()
{
    // if (reconnect_attempts_ >= max_reconnect_attempts_)
    // {
    //     RCLCPP_ERROR(logger_, "Max reconnection attempts reached");
    //     return false;
    // }

    reconnect_attempts_++;

    // RCLCPP_INFO(logger_, "Reconnection attempt %d/%d", reconnect_attempts_, max_reconnect_attempts_);
    RCLCPP_INFO(logger_, "Reconnection attempt %d", reconnect_attempts_);

    // // 添加延时，确保资源完全释放
    // std::this_thread::sleep_for(std::chrono::milliseconds(100));

    bool success = reinitialize();
    if (success)
    {
        reconnect_attempts_ = 0; // 成功时重置计数
    }
    else
    {
        RCLCPP_WARN(logger_, "Reconnection failed, waiting before next attempt...");
        std::this_thread::sleep_for(reconnect_interval_);
    }

    return success;
}

// 初始化
bool EtherCATTransmit::ethercat_init(char *ifname)
{
    int cnt_index = 0;
    int rc = 0;
    bool result = true;

    RCLCPP_INFO(logger_, "[EtherCAT] Initializing EtherCAT");
    check_thread_id += 1;
    osal_thread_create((void *)&checkThread, 128000, (void *)&EtherCATTransmit::ethercat_check_thread, this);
    for (cnt_index = 1; cnt_index < 3; cnt_index++)
    {
        RCLCPP_INFO(logger_, "[EtherCAT] Attempting to start EtherCAT, try %d of 10.", cnt_index);
        rc = ethercat_init_once(ifname);
        if (rc)
        {
            break;
        }

        osal_usleep(1000000);
    }
    if (rc)
    {
        RCLCPP_INFO(logger_, "[EtherCAT] EtherCAT successfully initialized on attempt %d", cnt_index);
    }
    else
    {
        RCLCPP_ERROR(logger_, "[EtherCAT Error] Failed to initialize EtherCAT after 10 tries.");
    }

    if (ec_slavecount <= 0)
    {
        RCLCPP_INFO(logger_, "Cannot find any slave!");
        result = false;
    }
    else
    {
        RCLCPP_INFO(logger_, "Slave count: %d", ec_slavecount);
    }

    return result;
}

// 反初始化
void EtherCATTransmit::ethercat_uninit()
{
    is_checking = false;

    if (checkThread)
    {
        pthread_join((pthread_t)checkThread, nullptr); // 等待 pthread 结束
        checkThread = nullptr;
    }

    // 反初始化SOEM
    ec_close(); // 重要：关闭SOEM连接
}

// 单次初始化
int EtherCATTransmit::ethercat_init_once(const char *ifname)
{
    int oloop, iloop, chk;
    needlf = FALSE;
    inOP = FALSE;

    /* initialise SOEM, bind socket to ifname */
    if (ec_init(ifname))
    {
        RCLCPP_INFO(logger_, "[EtherCAT Init] Initialization on device %s succeeded.", ifname);
        /* find and auto-config slaves */

        if (ec_config_init(FALSE) > 0)
        {
            RCLCPP_INFO(logger_, "[EtherCAT Init] %d slaves found and configured.", ec_slavecount);
            if (ec_slavecount < slave_num_)
            {
                RCLCPP_INFO(logger_, "[RT EtherCAT] Warning: Expected %d slaves, found %d.", slave_num_, ec_slavecount);
            }

            for (int slave_idx = 0; slave_idx < ec_slavecount; slave_idx++)
                ec_slave[slave_idx + 1].CoEdetails &= ~ECT_COEDET_SDOCA;

            ec_config_map(&IOmap);
            ec_configdc();

            RCLCPP_INFO(logger_, "[EtherCAT Init] Mapped slaves.");
            /* wait for all slaves to reach SAFE_OP state */
            ec_statecheck(0, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE * slave_num_);

            for (int slave_idx = 0; slave_idx < ec_slavecount; slave_idx++)
            {
                RCLCPP_INFO(logger_, "[SLAVE %d]", slave_idx);
                RCLCPP_INFO(logger_, "  IN  %d bytes, %d bits", ec_slave[slave_idx].Ibytes, ec_slave[slave_idx].Ibits);
                RCLCPP_INFO(logger_, "  OUT %d bytes, %d bits", ec_slave[slave_idx].Obytes, ec_slave[slave_idx].Obits);
                RCLCPP_INFO(logger_, "");
            }

            oloop = ec_slave[0].Obytes;
            if ((oloop == 0) && (ec_slave[0].Obits > 0))
                oloop = 1;
            if (oloop > 8)
                oloop = 8;
            iloop = ec_slave[0].Ibytes;
            if ((iloop == 0) && (ec_slave[0].Ibits > 0))
                iloop = 1;
            if (iloop > 8)
                iloop = 8;

            RCLCPP_INFO(logger_, "[EtherCAT Init] segments : %d : %d %d %d %d",
                        ec_group[0].nsegments, ec_group[0].IOsegment[0],
                        ec_group[0].IOsegment[1], ec_group[0].IOsegment[2],
                        ec_group[0].IOsegment[3]);

            RCLCPP_INFO(logger_, "[EtherCAT Init] Requesting operational state for all slaves...");
            expectedWKC = (ec_group[0].outputsWKC * 2) + ec_group[0].inputsWKC;
            RCLCPP_INFO(logger_, "[EtherCAT Init] Calculated workcounter %d", expectedWKC);
            ec_slave[0].state = EC_STATE_OPERATIONAL;
            /* send one valid process data to make outputs in slaves happy*/
            ec_send_processdata();
            ec_receive_processdata(EC_TIMEOUTRET);
            /* request OP state for all slaves */
            ec_writestate(0);
            chk = 40;
            /* wait for all slaves to reach OP state */
            do
            {
                ec_send_processdata();
                ec_receive_processdata(EC_TIMEOUTRET);
                ec_statecheck(0, EC_STATE_OPERATIONAL, 50000);
            } while (chk-- && (ec_slave[0].state != EC_STATE_OPERATIONAL));

            if (ec_slave[0].state == EC_STATE_OPERATIONAL)
            {
                RCLCPP_INFO(logger_, "[EtherCAT Init] Operational state reached for all slaves.");
                inOP = TRUE;
                return 1;
            }
            else
            {
                RCLCPP_ERROR(logger_, "[EtherCAT Error] Not all slaves reached operational state.");
                ec_readstate();
                for (int i = 1; i <= ec_slavecount; i++)
                {
                    if (ec_slave[i].state != EC_STATE_OPERATIONAL)
                    {
                        RCLCPP_ERROR(logger_, "[EtherCAT Error] Slave %d State=0x%2.2x StatusCode=0x%4.4x : %s",
                                     i, ec_slave[i].state, ec_slave[i].ALstatuscode,
                                     ec_ALstatuscode2string(ec_slave[i].ALstatuscode));
                    }
                }
            }
        }
        else
        {
            RCLCPP_ERROR(logger_, "[EtherCAT Error] No slaves found!");
        }
    }
    else
    {
        RCLCPP_ERROR(logger_, "[EtherCAT Error] No socket connection on %s - are you is_running run.sh?", ifname);
    }
    return 0;
}

OSAL_THREAD_FUNC EtherCATTransmit::ethercat_check_thread(void *ptr)
{
    // 将传入的 void* 上下文转换回 EtherCATTransmit* 类型
    EtherCATTransmit *instance = static_cast<EtherCATTransmit *>(ptr);

    // 调用真正的成员函数
    if (instance)
    {
        instance->ethercat_check();
    }
}

// 从站故障检查
void EtherCATTransmit::ethercat_check()
{
    int slave = 0;

    RCLCPP_INFO(logger_, "[EtherCAT Check] Start checking...");

    int connect_count = 0;

    while (is_checking)
    {
        bool current_connection = inOP && (wkc_err_count <= K_ETHERCAT_ERR_MAX);
        if (!current_connection)
        {
            connect_count = 0;
        }
        else
        {
            connect_count++;
        }
        current_connection = current_connection && (connect_count >= 3);

        // 连接状态变化
        if (current_connection != connected_)
        {
            RCLCPP_WARN(logger_, "[EtherCAT Check %d] !!! - %d %d", check_thread_id, inOP, wkc_err_count <= K_ETHERCAT_ERR_MAX);

            connected_ = current_connection;
            RCLCPP_WARN(logger_, "[EtherCAT] Connection status changed: %s", connected_ ? "CONNECTED" : "DISCONNECTED");

            // 通知连接状态变化
            if (connection_callback_)
            {
                connection_callback_(connected_);
            }
        }

        // count errors
        if (err_iteration_count > K_ETHERCAT_ERR_PERIOD)
        {
            err_iteration_count = 0;
            err_count = 0;
        }

        if (err_count > K_ETHERCAT_ERR_MAX)
        {
            // possibly shut down
            RCLCPP_ERROR(logger_, "[EtherCAT Error] EtherCAT connection degraded.");
            RCLCPP_INFO(logger_, "[Simulink-Linux] Shutting down....");
            // degraded_handler();
            is_checking = false; // 通知自己退出
            return;              // 线程函数直接返回
        }
        err_iteration_count++;

        if (inOP && ((wkc < expectedWKC) || ec_group[currentgroup].docheckstate))
        {
            if (needlf)
            {
                needlf = FALSE;
                // printf("\n");
            }
            /* one ore more slaves are not responding */
            ec_group[currentgroup].docheckstate = FALSE;
            ec_readstate();
            for (slave = 1; slave <= ec_slavecount; slave++)
            {
                if ((ec_slave[slave].group == currentgroup) && (ec_slave[slave].state != EC_STATE_OPERATIONAL))
                {
                    ec_group[currentgroup].docheckstate = TRUE;
                    if (ec_slave[slave].state == (EC_STATE_SAFE_OP + EC_STATE_ERROR))
                    {
                        RCLCPP_ERROR(logger_, "[EtherCAT Error] Slave %d is in SAFE_OP + ERROR, attempting ack.", slave);
                        ec_slave[slave].state = (EC_STATE_SAFE_OP + EC_STATE_ACK);
                        ec_writestate(slave);
                        err_count++;
                    }
                    else if (ec_slave[slave].state == EC_STATE_SAFE_OP)
                    {
                        RCLCPP_ERROR(logger_, "[EtherCAT Error] Slave %d is in SAFE_OP, change to OPERATIONAL.", slave);
                        ec_slave[slave].state = EC_STATE_OPERATIONAL;
                        ec_writestate(slave);
                        err_count++;
                    }
                    else if (ec_slave[slave].state > 0)
                    {
                        if (ec_reconfig_slave(slave, EC_TIMEOUTMON))
                        {
                            ec_slave[slave].islost = FALSE;
                            RCLCPP_INFO(logger_, "[EtherCAT Status] Slave %d reconfigured.", slave);
                        }
                    }
                    else if (!ec_slave[slave].islost)
                    {
                        /* re-check state */
                        ec_statecheck(slave, EC_STATE_OPERATIONAL, EC_TIMEOUTRET);
                        if (!ec_slave[slave].state)
                        {
                            ec_slave[slave].islost = TRUE;
                            RCLCPP_ERROR(logger_, "[EtherCAT Error] Slave %d lost.", slave);
                            err_count++;
                        }
                    }
                }
                if (ec_slave[slave].islost)
                {
                    if (!ec_slave[slave].state)
                    {
                        if (ec_recover_slave(slave, EC_TIMEOUTMON))
                        {
                            ec_slave[slave].islost = FALSE;
                            RCLCPP_INFO(logger_, "[EtherCAT Status] Slave %d recovered.", slave);
                        }
                    }
                    else
                    {
                        ec_slave[slave].islost = FALSE;
                        RCLCPP_INFO(logger_, "[EtherCAT Status] Slave %d found", slave);
                    }
                }
            }
            if (!ec_group[currentgroup].docheckstate)
            {
                // RCLCPP_INFO(logger_, "[EtherCAT Status] All slaves resumed OPERATIONAL.");
            }
        }

        osal_usleep(50000);
    }

    RCLCPP_INFO(logger_, "[EtherCAT Check] Check thread exiting...");
}

// 处理线程
void EtherCATTransmit::ethercat_run()
{
    if (wkc_err_iteration_count > K_ETHERCAT_ERR_PERIOD)
    {
        wkc_err_count = 0;
        wkc_err_iteration_count = 0;
    }
    if (wkc_err_count > K_ETHERCAT_ERR_MAX)
    {
        // RCLCPP_ERROR(logger_, "[EtherCAT Error] Error count too high!");
        // degraded_handler();
    }

    // send
    ethercat_command_set();
    ec_send_processdata();

    // receive
    wkc = ec_receive_processdata(EC_TIMEOUTRET);
    ethercat_data_get();

    //  check for dropped packet
    if (wkc < expectedWKC)
    {
        // RCLCPP_ERROR(logger_, "[EtherCAT Error] Dropped packet (Bad WKC!).");
        wkc_err_count++;
    }
    else
    {
        needlf = TRUE;
    }
    wkc_err_iteration_count++;
}

// 指令设置
void EtherCATTransmit::ethercat_command_set()
{
    for (int slave = 0; slave < ec_slavecount; ++slave)
    {
        EtherCAT_Msg_ptr msg;

        // 从队列取出用户命令
        if (messages_[slave]->pop(msg))
        {
            if (show_print_)
            {
                RCLCPP_INFO(logger_, "[DEBUG] Slave %d: Popped command from queue", slave);
            }

            Tx_Message[slave] = *msg;
            isConfig[slave] = true;

            // 写入 slave 输出缓冲区
            EtherCAT_Msg *slave_dest = (EtherCAT_Msg *)(ec_slave[slave + 1].outputs);
            if (slave_dest)
            {
                if (show_print_)
                {
                    RCLCPP_INFO(logger_, "[DEBUG] Slave %d: write command to dest.", slave);
                }

                *(slave_dest) = Tx_Message[slave];
            }
        }
    }
}

// 数据获取
void EtherCATTransmit::ethercat_data_get()
{
    static uint64_t cycle = 0;

    for (int slave = 0; slave < ec_slavecount; ++slave)
    {
        EtherCAT_Msg *slave_src = (EtherCAT_Msg *)(ec_slave[slave + 1].inputs);
        if (slave_src)
        {
            Rx_Message[slave] = *(EtherCAT_Msg *)(ec_slave[slave + 1].inputs);

            uint8_t ack_status = motor_control_->rv_can_data_repack(&Rx_Message[slave], 0x00, slave);
            if (isConfig[slave] && ack_status != 0)
            {
                isConfig[slave] = false;
                if (show_print_)
                {
                    RCLCPP_INFO(logger_, "slave %d, ack_status %d, msg:", slave, ack_status);
                    motor_control_->rv_message_print(ack_status);
                }
            }
        }
    }
}

// 启动线程
void EtherCATTransmit::start_ethercat_run()
{
    is_running = true;

    runThread = std::thread([this]()
                            {
    while (is_running)
    {
        ethercat_run();
        usleep(1000);
    } });
}

// 停止线程
void EtherCATTransmit::stop_ethercat_run()
{
    is_running = false;
    if (runThread.joinable())
    {
        runThread.join();
    }
}
