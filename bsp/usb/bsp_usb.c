/**
 * @file bsp_usb.c
 * @author your name (you@domain.com)
 * @brief usb是单例bsp,因此不保存实例
 * @version 0.1
 * @date 2023-02-09
 *
 * @copyright Copyright (c) 2023
 *
 */

#include "bsp_usb.h"
#include "bsp_log.h"
#include "bsp_dwt.h"
#include <string.h>

static uint8_t *bsp_usb_rx_buffer; // 接收到的数据会被放在这里,buffer size为2048
static USBCallback s_rx_cbk = NULL;

/* ============================================================================
   VCP 接收队列(BSPUART-02 的 USB 侧, 2026-09-26)
   ----------------------------------------------------------------------------
   原来 CDC_Receive_HS()(USB 中断上下文)里直接调模块回调, 视觉要在这里做
   "滑动找帧 + CRC16 校验"。现在中断只把端点缓冲的数据拷进队列, 解析交给任务里的
   VCPProcessRx()。
   为什么必须拷贝: USB 端点缓冲是复用的 —— CDC_Receive_HS 返回前会重新武装接收,
   下一次数据随时可能盖进来, 只传指针给任务会读到半新半旧的数据。
   并发模型: 单生产者(USB 中断) / 单消费者(VCPProcessRx)。
   ========================================================================== */
#define VCP_RXQ_DEPTH 8u
#define VCP_RXQ_ITEM 128u /* USB FS 单次回调 ≤64 字节, 留一倍余量 */

static uint8_t vcp_rxq[VCP_RXQ_DEPTH][VCP_RXQ_ITEM] __attribute__((aligned(4)));
static volatile uint16_t vcp_rxq_len[VCP_RXQ_DEPTH];
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

    while (vcp_tail != vcp_head)
    {
        uint16_t len = vcp_rxq_len[vcp_tail];
        if (bsp_usb_rx_buffer != NULL && s_rx_cbk != NULL && len > 0u)
        {
            /* 拷回端点缓冲再回调: 模块回调(DecodeVision)仍按原约定从
               bsp_usb_rx_buffer 读数据, 模块侧零改动 */
            memcpy(bsp_usb_rx_buffer, vcp_rxq[vcp_tail], len);
            s_rx_cbk(len);
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

uint8_t *USBInit(USB_Init_Config_s usb_conf)
{
    // usb的软件复位(模拟拔插)在usbd_conf.c中的HAL_PCD_MspInit()中
    s_rx_cbk = usb_conf.rx_cbk;
    bsp_usb_rx_buffer = CDCInitRxbufferNcallback(usb_conf.tx_cbk, usb_conf.rx_cbk); // 获取接收数据指针
    // usb的接收回调函数会在这里被设置,并将数据保存在bsp_usb_rx_buffer中
    LOGINFO("USB init success");
    return bsp_usb_rx_buffer;
}

void USBTransmit(uint8_t *buffer, uint16_t len)
{
    CDC_Transmit_HS(buffer, len); // 发送
}
