/*
* @Descripttion: 保存超级电容原始 CAN 反馈并发送固定八字节控制内容。
 * @version:
 * @Author: Chenfu
 * @Date: 2022-12-02 21:32:47
 * @LastEditTime: 2022-12-05 15:29:49
 */
#include "super_cap.h"
#include <string.h>
#include <stdlib.h>
#include "FreeRTOS.h"
#include "task.h"
#include <limits.h>

struct SuperCapInstance {
    CANInstance *can_ins;
    SuperCap_Msg_s value;
    uint32_t source_rx_ms;
    uint8_t received;
};

static SuperCapInstance *super_cap_instance = NULL; // 可以由app保存此指针

// 长度足够时才按高字节在前组合原始整数，比例仍需实际设备协议确认。
static void SuperCapRxCallback(CANInstance *_instance)
{
    if (_instance == NULL)
        return;
    SuperCapInstance *owner = (SuperCapInstance *)_instance->id;
    if (owner == NULL || owner != super_cap_instance || _instance->rx_len < 6u || _instance->rx_len > 8u)
        return;
    const uint8_t *rxbuff = _instance->rx_buff;
    const SuperCap_Msg_s candidate = {
        (uint16_t)((uint16_t)rxbuff[0] << 8 | rxbuff[1]),
        (uint16_t)((uint16_t)rxbuff[2] << 8 | rxbuff[3]),
        (uint16_t)((uint16_t)rxbuff[4] << 8 | rxbuff[5])
    };
    // 三项数值与原始接收时间一起发布，避免读到不同帧的组合。
    taskENTER_CRITICAL();
    owner->value = candidate;
    owner->source_rx_ms = _instance->rx_timestamp_ms;
    owner->received = 1u;
    taskEXIT_CRITICAL();
}

SuperCapInstance *SuperCapInit(const SuperCap_Init_Config_s *supercap_config)
{
    if (supercap_config == NULL || supercap_config->can_config.can_handle == NULL || super_cap_instance != NULL)
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

CAN_TxSubmitStatus_t SuperCapSend(SuperCapInstance *instance, const uint8_t *data, size_t length)
{
    if (instance == NULL || instance != super_cap_instance || data == NULL || length != 8u || instance->can_ins == NULL)
        return CAN_TX_SUBMIT_INVALID;
    const CAN_TxFrame_t frame = {
        .identifier = instance->can_ins->tx_id,
        .identifier_type = CAN_TX_ID_STANDARD,
        .data = data,
        .length = 8u,
    };
    uint32_t request_id = 0u;
    return CANSubmitPeriodic(instance->can_ins->can_handle, instance->can_ins, 0u,
                             &frame, CAN_TX_PRIORITY_NORMAL, 20u, 10000u, &request_id);
}

SuperCapStatus_t SuperCapGet(const SuperCapInstance *instance, uint32_t now_ms,
                            uint32_t max_age_ms, SuperCapFeedback_t *output)
{
    if (instance == NULL || instance != super_cap_instance || output == NULL ||
        max_age_ms == 0u || max_age_ms > INT32_MAX)
        return SUPERCAP_ERROR_ARGUMENT;
    SuperCapFeedback_t candidate = {0};
    taskENTER_CRITICAL();
    candidate.value = instance->value;
    candidate.source_rx_ms = instance->source_rx_ms;
    const uint8_t received = instance->received;
    taskEXIT_CRITICAL();
    if (received != 0u) {
        candidate.age_ms = now_ms - candidate.source_rx_ms;
        candidate.state = candidate.age_ms <= max_age_ms ? SUPERCAP_FEEDBACK_FRESH : SUPERCAP_FEEDBACK_EXPIRED;
    }
    *output = candidate;
    return SUPERCAP_OK;
}
