#include "encos_driver/transmit.h"

EtherCATTransmit::EtherCATTransmit(int slave_num, int motor_num, bool show_print)
    : logger_(rclcpp::get_logger("encos_driver")),
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
}

// 初始化
bool EtherCATTransmit::initialize(const std::string &ifname)
{
    ifname_ = ifname;

    // EtherCAT 初始化代码
    if (ethercat_init((char *)(ifname.c_str())))
    {
        // 启动线程
        start_ethercat_run();

        return true;
    }

    return false;
}

/* add ns to ec_timet */
void EtherCATTransmit::add_time_ns(ec_timet *ts, int64 addtime)
{
    ec_timet addts;

    addts.tv_nsec = addtime % NSEC_PER_SEC;
    addts.tv_sec = (addtime - addts.tv_nsec) / NSEC_PER_SEC;
    osal_timespecadd(ts, &addts, ts);
}

/* PI calculation to get linux time synced to DC time */
void EtherCATTransmit::ec_sync(int64 reftime, int64 cycletime, int64 *offsettime)
{
    static int64 integral = 0;
    int64 delta;
    delta = (reftime - syncoffset) % cycletime;
    if (delta > (cycletime / 2))
    {
        delta = delta - cycletime;
    }
    timeerror = -delta;
    integral += timeerror;
    *offsettime = (int64)((timeerror * pgain) + (integral * igain));
}

OSAL_THREAD_FUNC EtherCATTransmit::ethercat_rt_thread(void *ptr)
{
    // 将传入的 void* 上下文转换回 EtherCATTransmit* 类型
    EtherCATTransmit *instance = static_cast<EtherCATTransmit *>(ptr);

    // 调用真正的成员函数
    if (instance)
    {
        instance->ethercat_rt();
    }
}

/* Cyclic RT EtherCAT thread */
void EtherCATTransmit::ethercat_rt(void)
{
    ec_timet ts;
    int ht;
    static int64_t toff = 0;

    dorun = 0;
    while (!mappingdone)
    {
        osal_usleep(100);
    }
    osal_get_monotonic_time(&ts);
    ht = (ts.tv_nsec / 1000000) + 1; /* round to nearest ms */
    ts.tv_nsec = ht * 1000000;
    ecx_send_processdata(&ctx);

    while (is_rt)
    {
        /* calculate next cycle start */
        add_time_ns(&ts, cycletime + toff);
        /* wait to cycle start */
        osal_monotonic_sleep(&ts);
        if (dorun > 0)
        {
            cycle++;
            wkc = ecx_receive_processdata(&ctx, EC_TIMEOUTRET);
            if (wkc != expectedWKC)
                dowkccheck++;
            else
                dowkccheck = 0;

            if (ctx.slavelist[0].hasdc && (wkc > 0))
            {
                /* calculate toff to get linux time and DC synced */
                ec_sync(ctx.DCtime, cycletime, &toff);
            }
            ecx_mbxhandler(&ctx, 0, 4);
            ecx_send_processdata(&ctx);
        }
    }
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

/* Slave error handler */
void EtherCATTransmit::ethercat_check(void)
{
    int slaveix;

    while (is_check)
    {
        if (inOP && ((dowkccheck > 2) || ctx.grouplist[currentgroup].docheckstate))
        {
            /* one or more slaves are not responding */
            ctx.grouplist[currentgroup].docheckstate = FALSE;
            ecx_readstate(&ctx);
            for (slaveix = 1; slaveix <= ctx.slavecount; slaveix++)
            {
                ec_slavet *slave = &ctx.slavelist[slaveix];

                if ((slave->group == currentgroup) && (slave->state != EC_STATE_OPERATIONAL))
                {
                    ctx.grouplist[currentgroup].docheckstate = TRUE;
                    if (slave->state == (EC_STATE_SAFE_OP + EC_STATE_ERROR))
                    {
                        RCLCPP_ERROR(logger_, "ERROR : slave %d is in SAFE_OP + ERROR, attempting ack.", slaveix);
                        slave->state = (EC_STATE_SAFE_OP + EC_STATE_ACK);
                        ecx_writestate(&ctx, slaveix);
                    }
                    else if (slave->state == EC_STATE_SAFE_OP)
                    {
                        RCLCPP_WARN(logger_, "WARNING : slave %d is in SAFE_OP, change to OPERATIONAL.", slaveix);
                        slave->state = EC_STATE_OPERATIONAL;
                        if (slave->mbxhandlerstate == ECT_MBXH_LOST)
                            slave->mbxhandlerstate = ECT_MBXH_CYCLIC;
                        ecx_writestate(&ctx, slaveix);
                    }
                    else if (slave->state > EC_STATE_NONE)
                    {
                        if (ecx_reconfig_slave(&ctx, slaveix, EC_TIMEOUTMON) >= EC_STATE_PRE_OP)
                        {
                            slave->islost = FALSE;
                            RCLCPP_INFO(logger_, "MESSAGE : slave %d reconfigured.", slaveix);
                        }
                    }
                    else if (!slave->islost)
                    {
                        /* re-check state */
                        ecx_statecheck(&ctx, slaveix, EC_STATE_OPERATIONAL, EC_TIMEOUTRET);
                        if (slave->state == EC_STATE_NONE)
                        {
                            slave->islost = TRUE;
                            slave->mbxhandlerstate = ECT_MBXH_LOST;
                            /* zero input data for this slave */
                            if (slave->Ibytes)
                            {
                                memset(slave->inputs, 0x00, slave->Ibytes);
                            }
                            RCLCPP_ERROR(logger_, "ERROR : %s - slave %d lost.", ifname_.c_str(), slaveix);
                        }
                    }
                }
                if (slave->islost)
                {
                    if (slave->state <= EC_STATE_INIT)
                    {
                        if (ecx_recover_slave(&ctx, slaveix, EC_TIMEOUTMON))
                        {
                            slave->islost = FALSE;
                            RCLCPP_INFO(logger_, "MESSAGE : %s - slave %d recovered.", ifname_.c_str(), slaveix);
                        }
                    }
                    else
                    {
                        slave->islost = FALSE;
                        RCLCPP_INFO(logger_, "MESSAGE : %s - slave %d found.", ifname_.c_str(), slaveix);
                    }
                }
            }
            if (!ctx.grouplist[currentgroup].docheckstate)
            {
                // RCLCPP_INFO(logger_, "OK : all slaves resumed OPERATIONAL.");
            }

            dowkccheck = 0;
        }
        osal_usleep(10000);
    }
}

// 反初始化
void EtherCATTransmit::ethercat_uninit()
{
    is_rt = false;
    is_check = false;

    if (thread_rt_)
    {
        pthread_join((pthread_t)thread_rt_, nullptr); // 等待 pthread 结束
        thread_rt_ = 0;
    }
    if (thread_check_)
    {
        pthread_join((pthread_t)thread_check_, nullptr); // 等待 pthread 结束
        thread_check_ = 0;
    }

    /* Go to SAFE_OP */
    RCLCPP_INFO(logger_, "EtherCAT to SAFE_OP.");
    ctx.slavelist[0].state = EC_STATE_SAFE_OP;
    ecx_writestate(&ctx, 0);
    ecx_statecheck(&ctx, 0, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE);

    /* Go to INIT state */
    RCLCPP_INFO(logger_, "EtherCAT to INIT.");
    ctx.slavelist[0].state = EC_STATE_INIT;
    ecx_writestate(&ctx, 0);
    ecx_statecheck(&ctx, 0, EC_STATE_INIT, EC_TIMEOUTSTATE);

    // 反初始化SOEM
    ecx_close(&ctx); // 重要：关闭SOEM连接
}

// 初始化
bool EtherCATTransmit::ethercat_init(char *ifname)
{
    int cnt_index = 0;
    int rc = 0;
    bool result = true;

    RCLCPP_INFO(logger_, "[EtherCAT] Initializing EtherCAT");
    /* create process data thread */
    osal_thread_create_rt(&thread_rt_, 128000, (void *)&EtherCATTransmit::ethercat_rt_thread, this);
    /* create thread to handle slave error handling in OP */
    osal_thread_create(&thread_check_, 128000, (void *)&EtherCATTransmit::ethercat_check_thread, this);

    for (cnt_index = 1; cnt_index < 10; cnt_index++)
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

    if (ctx.slavecount <= 0)
    {
        RCLCPP_INFO(logger_, "Cannot find any slave!");
        result = false;
    }
    else
    {
        RCLCPP_INFO(logger_, "Slave count: %d", ctx.slavecount);
    }

    return result;
}

// 单次初始化
int EtherCATTransmit::ethercat_init_once(const char *ifname)
{
    RCLCPP_INFO(logger_, "EtherCAT Startup.");
    if (ecx_init(&ctx, ifname))
    {
        RCLCPP_INFO(logger_, "ecx_init ok.");
        ecx_config_init(&ctx);
        if (ctx.slavecount > 0)
        {
            if (ctx.slavecount < slave_num_)
            {
                RCLCPP_INFO(logger_, "[RT EtherCAT] Warning: Expected %d slaves, found %d.", slave_num_, ctx.slavecount);
            }

            for (int slave_idx = 0; slave_idx < ctx.slavecount; slave_idx++)
                ctx.slavelist[slave_idx + 1].CoEdetails &= ~ECT_COEDET_SDOCA;

            ec_groupt *group = &ctx.grouplist[0];

            ecx_config_map_group(&ctx, IOmap, 0);
            expectedWKC = (group->outputsWKC * 2) + group->inputsWKC;
            RCLCPP_INFO(logger_, "%d slaves found and configured.", ctx.slavecount);

            RCLCPP_INFO(logger_, "segments : %d : %d %d %d %d.",
                   group->nsegments,
                   group->IOsegment[0],
                   group->IOsegment[1],
                   group->IOsegment[2],
                   group->IOsegment[3]);

            /* Configure distributed clocks */
            mappingdone = 1;
            ecx_configdc(&ctx);

            /* Add all CoE slaves to cyclic mailbox handler */
            int sdoslave = -1;
            for (int si = 1; si <= ctx.slavecount; si++)
            {
                ec_slavet *slave = &ctx.slavelist[si];
                if (slave->CoEdetails > 0)
                {
                    ecx_slavembxcyclic(&ctx, si);
                    sdoslave = si;
                    RCLCPP_INFO(logger_, " Slave %d added to cyclic mailbox handler.", si);
                }
            }

            /* Let network sync to clocks */
            dorun = 1;
            osal_usleep(1000000);

            /* Go to operational state */
            ctx.slavelist[0].state = EC_STATE_OPERATIONAL;
            ecx_writestate(&ctx, 0);
            ecx_statecheck(&ctx, 0, EC_STATE_OPERATIONAL, EC_TIMEOUTSTATE);

            if (ctx.slavelist[0].state != EC_STATE_OPERATIONAL)
            {
                ecx_readstate(&ctx);
                for (int si = 1; si <= ctx.slavecount; si++)
                {
                    ec_slavet *slave = &ctx.slavelist[si];
                    if (slave->state != EC_STATE_OPERATIONAL)
                    {
                        RCLCPP_INFO(logger_, "Slave %d State=0x%2.2x StatusCode=0x%4.4x : %s",
                               si,
                               slave->state,
                               slave->ALstatuscode,
                               ec_ALstatuscode2string(slave->ALstatuscode));
                    }
                }
            }
            else
            {
                RCLCPP_INFO(logger_, "EtherCAT OP.");
                inOP = TRUE;
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
}

// 处理线程
void EtherCATTransmit::ethercat_run()
{
    // send
    ethercat_command_set();
    ecx_send_processdata(&ctx);

    // receive
    wkc = ecx_receive_processdata(&ctx, EC_TIMEOUTRET);
    ethercat_data_get();
}

// 指令设置
void EtherCATTransmit::ethercat_command_set()
{
    for (int slave = 0; slave < ctx.slavecount; ++slave)
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

            // auto now = std::chrono::system_clock::now();
            // auto duration = now.time_since_epoch();
            // int64_t timestamp = std::chrono::duration_cast<std::chrono::microseconds>(duration).count();
            // RCLCPP_INFO(logger_, "timestamp: %ld", timestamp);

            // 写入 slave 输出缓冲区
            ec_slavet *ec_slave = &ctx.slavelist[slave + 1];
            EtherCAT_Msg *slave_dest = (EtherCAT_Msg *)(ec_slave->outputs);
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

    for (int slave = 0; slave < ctx.slavecount; ++slave)
    {
        ec_slavet *ec_slave = &ctx.slavelist[slave + 1];
        EtherCAT_Msg *slave_src = (EtherCAT_Msg *)(ec_slave->inputs);
        if (slave_src)
        {
            Rx_Message[slave] = *(EtherCAT_Msg *)(ec_slave->inputs);

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

    thread_run = std::thread([this]()
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
    if (thread_run.joinable())
    {
        thread_run.join();
    }
}
