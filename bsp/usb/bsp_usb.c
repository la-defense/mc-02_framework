/**
 * @file bsp_usb.c
* @brief USB CDC 单例收发；中断保存接收内容，任务跳过旧包并调用模块回调。
 * @version 0.1
 * @date 2023-02-09
 *
 * @copyright Copyright (c) 2023
 *
 */

#include "bsp_usb.h"
#include "message_freshness.h"
#include "bsp_log.h"
#include "bsp_dwt.h"
#include <string.h>

static uint8_t *bsp_usb_rx_buffer; // 接收到的数据会被放在这里,buffer size为2048
static USBCallback s_rx_cbk = NULL;

// USB 端点数组会被下一包复用，先复制到队列；中断负责写入，唯一派发任务负责取出。
#define VCP_RXQ_DEPTH 8u
#define VCP_RXQ_ITEM 128u /* USB FS 单次回调 ≤64 字节, 留一倍余量 */

static uint8_t vcp_rxq[VCP_RXQ_DEPTH][VCP_RXQ_ITEM] __attribute__((aligned(4)));
static volatile uint16_t vcp_rxq_len[VCP_RXQ_DEPTH];
static volatile uint32_t vcp_rxq_timestamp_ms[VCP_RXQ_DEPTH];
static const uint8_t *vcp_current_rx_data;
static uint32_t vcp_current_rx_timestamp_ms;
static volatile uint16_t vcp_head = 0u; /* 生产者(中断)写 */
static volatile uint16_t vcp_tail = 0u; /* 消费者(任务)写 */
static volatile uint32_t vcp_drop = 0u; /* 队列满丢弃帧数 */
static volatile uint16_t vcp_peak = 0u; /* 历史最大占用 */
static LogRateLimit_t vcp_log_rl = {0};

volatile DWT_Probe_t vcp_prof_rx_isr = {0};   /* 中断侧入队耗时 */
volatile DWT_Probe_t vcp_prof_dispatch = {0}; /* 任务侧派发耗时 */
volatile uint32_t vcp_rxq_drop_total = 0;

/**
 * @brief 把 USB 收到的一包数据压进队列(由 CDC_Receive_HS 在中断里调用)
 */
void VCPQueuePush(const uint8_t *buf, uint16_t len)
{
    uint32_t probe = DWT_ProbeStart();

    if (buf != NULL && len > 0u)
    {
        if (len > VCP_RXQ_ITEM)
            len = VCP_RXQ_ITEM;

        uint16_t next = (uint16_t)((vcp_head + 1u) % VCP_RXQ_DEPTH);
        if (next == vcp_tail)
        {
            vcp_drop++; /* 队列满: 丢新帧 + 计数 */
        }
        else
        {
            memcpy(vcp_rxq[vcp_head], buf, len);
            vcp_rxq_len[vcp_head] = len;
            vcp_rxq_timestamp_ms[vcp_head] = HAL_GetTick();
            __DMB();
            vcp_head = next;

            uint16_t used = (uint16_t)((uint16_t)((next - vcp_tail + VCP_RXQ_DEPTH) %
                                                  VCP_RXQ_DEPTH));
            if (used > vcp_peak)
                vcp_peak = used;
        }
    }

    DWT_ProbeDone(&vcp_prof_rx_isr, probe);
}

/**
 * @brief 逐包派发 VCP 收到的数据(在任务上下文调用)
 * @note  全工程只允许一个调用者(当前挂在 1kHz 的 MotorControlTask 里)。
 */
void VCPProcessRx(void)
{
    /* 双消费者保护(与 CANProcessRx 同一套做法) */
    static volatile uint8_t processing = 0;
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    if (processing)
    {
        __set_PRIMASK(primask);
        return;
    }
    processing = 1u;
    __set_PRIMASK(primask);

    uint32_t probe = DWT_ProbeStart();

    // 按入队顺序处理，每包超过 50 ms 就不再回调；即使跳过也推进到下一项。
    while (vcp_tail != vcp_head)
    {
        uint16_t len = vcp_rxq_len[vcp_tail];
        if (bsp_usb_rx_buffer != NULL && s_rx_cbk != NULL && len > 0u &&
            (uint32_t)(HAL_GetTick() - vcp_rxq_timestamp_ms[vcp_tail]) <= MESSAGE_DEFAULT_MAX_AGE_MS)
        {
            // 兼容旧回调，同时提供本包队列地址；回调返回后撤销该地址。
            memcpy(bsp_usb_rx_buffer, vcp_rxq[vcp_tail], len);
            vcp_current_rx_data = vcp_rxq[vcp_tail];
            vcp_current_rx_timestamp_ms = vcp_rxq_timestamp_ms[vcp_tail];
            s_rx_cbk(len);
            vcp_current_rx_data = NULL;
        }
        __DMB();
        vcp_tail = (uint16_t)((vcp_tail + 1u) % VCP_RXQ_DEPTH);
    }

    if (vcp_drop != 0u && LogRateLimitAllow(&vcp_log_rl, 1000u))
        LOGWARNING("[bsp_usb] VCP 接收队列满, 已丢帧 %lu 次", (unsigned long)vcp_drop);

    vcp_rxq_drop_total = vcp_drop;
    DWT_ProbeDone(&vcp_prof_dispatch, probe);
    processing = 0u;
}
// 注意usb单个数据包(Full speed模式下)最大为64byte,超出可能会出现丢包情况

const uint8_t *VCPGetCurrentRxData(void) { return vcp_current_rx_data; }
uint32_t VCPGetCurrentRxTimestamp(void) { return vcp_current_rx_data != NULL ? vcp_current_rx_timestamp_ms : 0u; }

uint8_t *USBInit(USB_Init_Config_s usb_conf)
{
    // 此处仅设置回调和接收地址；USB 硬件与枚举由设备初始化代码负责。
    s_rx_cbk = usb_conf.rx_cbk;
    bsp_usb_rx_buffer = CDCInitRxbufferNcallback(usb_conf.tx_cbk, usb_conf.rx_cbk); // 获取接收数据指针
    // usb的接收回调函数会在这里被设置,并将数据保存在bsp_usb_rx_buffer中
    LOGINFO("USB init success");
    return bsp_usb_rx_buffer;
}

uint8_t USBTransmit(const uint8_t *buffer, uint16_t len)
{
    if (buffer == NULL || len == 0u)
        return USBD_FAIL;

    return CDC_Transmit_HS((uint8_t *)buffer, len);
}
