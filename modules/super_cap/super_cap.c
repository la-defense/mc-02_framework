/*
 * @Descripttion: 保存超级电容原始 CAN 反馈并发送固定八字节控制内容。
 * @version:
 * @Author: Chenfu
 * @Date: 2022-12-02 21:32:47
 * @LastEditTime: 2022-12-05 15:29:49
 */
#include "super_cap.h"
#include "memory.h"
#include "stdlib.h"

static SuperCapInstance *super_cap_instance = NULL; // 可以由app保存此指针

// 按高字节在前组合三项原始整数；此处不检查帧长，也不做设备比例换算。
static void SuperCapRxCallback(CANInstance *_instance)
{
    if (_instance == NULL)
        return;
    SuperCapInstance *owner = (SuperCapInstance *)_instance->id;
    if (owner == NULL)
        return;
    uint8_t *rxbuff;
    SuperCap_Msg_s *Msg;
    rxbuff = _instance->rx_buff;
    Msg = &owner->cap_msg;
    Msg->vol = (uint16_t)(rxbuff[0] << 8 | rxbuff[1]);
    Msg->current = (uint16_t)(rxbuff[2] << 8 | rxbuff[3]);
    Msg->power = (uint16_t)(rxbuff[4] << 8 | rxbuff[5]);
}

SuperCapInstance *SuperCapInit(SuperCap_Init_Config_s *supercap_config)
{
    if (supercap_config == NULL || super_cap_instance != NULL)
        return NULL;
    SuperCapInstance *instance = (SuperCapInstance *)malloc(sizeof(*instance));
    if (instance == NULL)
        return NULL;
    memset(instance, 0, sizeof(*instance));

    CAN_Init_Config_s can_config = supercap_config->can_config;
    can_config.id = instance;
    can_config.can_module_callback = SuperCapRxCallback;
    instance->can_ins = CANRegister(&can_config);
    if (instance->can_ins == NULL)
    {
        free(instance);
        return NULL;
    }
    super_cap_instance = instance;
    return super_cap_instance;
}

void SuperCapSend(SuperCapInstance *instance, uint8_t *data)
{
    memcpy(instance->can_ins->tx_buff, data, 8);
    CANTransmit(instance->can_ins,1);
}

/** @brief 按值返回最近保存的原始反馈，不检查在线或数据时间。
 * @param[in] instance 有效实例，不能为 NULL。
 * @return 电压、电流、功率原始整数，比例见设备协议。
 */
SuperCap_Msg_s SuperCapGet(SuperCapInstance *instance)
{
    return instance->cap_msg;
}
