#ifndef __CONFIG_H
#define __CONFIG_H

#include <inttypes.h>
#include <string.h>
#include <memory>
#include <atomic>
#include <thread>

//********************************************//
//***********EtherCAT Message*****************//
//********************************************//

#pragma pack(push, 1)

typedef struct
{
    uint32_t id;
    uint8_t rtr;
    uint8_t dlc;
    uint8_t data[8];
} Motor_Msg;

typedef struct
{
    uint8_t motor_num;
    uint8_t can_ide;
    Motor_Msg motor[6];

} EtherCAT_Msg;

#pragma pack(pop)

typedef std::shared_ptr<EtherCAT_Msg> EtherCAT_Msg_ptr;

#endif
