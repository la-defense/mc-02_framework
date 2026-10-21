/**
 * @file master_process.c
 * @brief 视觉数据收发, SP+CRC16 协议(与上位机 sp_vision_25 对齐)
 *        支持 USB VCP(VISION_USE_VCP) 和 UART(VISION_USE_UART) 两种传输, 宏切换
 */
#include "master_process.h"
#include "crc.h"
#include "message_freshness.h"
#include "daemon.h"
#include "bsp_log.h"
#include "robot_def.h"
#include "string.h"
#include "FreeRTOS.h"
#include "task.h"

static Vision_Recv_s recv_data;
static Vision_Send_s send_data;
static DaemonInstance *vision_daemon_instance;
static uint32_t vision_rx_count = 0;
static uint32_t vision_tx_count = 0;
static uint32_t vision_tx_drop_count = 0;
static uint32_t vision_crc_error_count = 0;
static uint32_t vision_last_rx_ms = 0;
static uint8_t vision_has_valid_frame;

#ifdef VISION_USE_UART
#include "bsp_usart.h"
static USARTInstance *vision_usart_instance;
typedef struct {
    uint8_t bytes[sizeof(Vision_Recv_s)];
    uint32_t time[sizeof(Vision_Recv_s)];
    size_t used;
    uint32_t sequence;
    uint8_t have_sequence;
    volatile uint8_t reset_requested;
} VisionUartStream;
static VisionUartStream vision_stream;
#endif

#ifdef VISION_USE_VCP
#include "bsp_usb.h"
static uint8_t *vis_recv_buff;
#endif

/* 从接收缓冲中滑动寻找 SP 帧并解码 */
#ifdef VISION_USE_VCP
static void DecodeVision(const uint8_t *rx_data, uint16_t recv_len, uint32_t rx_timestamp_ms)
{
    // 按真正接收时刻检查这片数据，排队超过 50 ms 后不再采用。
    if (rx_data == NULL || (uint32_t)(HAL_GetTick() - rx_timestamp_ms) > MESSAGE_DEFAULT_MAX_AGE_MS)
        return;
    Vision_Recv_s packet;
    uint32_t crc_errors = 0u;
    const uint8_t found = VisionProtocolFindCommandFrame(
        rx_data, recv_len, &packet, &crc_errors);

    taskENTER_CRITICAL();
    vision_crc_error_count += crc_errors;
    if (found)
    {
        recv_data = packet;
        vision_rx_count++;
        vision_last_rx_ms = rx_timestamp_ms;
        vision_has_valid_frame = 1u;
        DaemonReload(vision_daemon_instance);
    }
    taskEXIT_CRITICAL();
}

#endif
#ifdef VISION_USE_UART
static void VisionShift(VisionUartStream *stream, size_t count)
{
    stream->used -= count;
    memmove(stream->bytes, stream->bytes + count, stream->used);
    memmove(stream->time, stream->time + count, stream->used * sizeof(stream->time[0]));
}

static void VisionUartRxCallback(void *context, const USART_RxBlock_t *block)
{
    VisionUartStream *stream = context;
    if (stream == NULL || block == NULL) return;
    taskENTER_CRITICAL();
    if (stream->reset_requested) { stream->used = 0u; stream->reset_requested = 0u; }
    taskEXIT_CRITICAL();
    if (block->data == NULL || block->length == 0u || block->length > 255u)
    { stream->used = 0u; return; }
    if ((stream->have_sequence && block->sequence != stream->sequence + 1u) ||
        (stream->used && (uint32_t)(HAL_GetTick() - stream->time[0]) > 50u))
        stream->used = 0u;
    stream->sequence = block->sequence;
    stream->have_sequence = 1u;
    if ((uint32_t)(HAL_GetTick() - block->received_ms) > 50u)
    { stream->used = 0u; return; }
    // 固定29字节候选；坏头或CRC失败后移动一字节，同一块内的完整帧按顺序全部处理。
    for (size_t i = 0u; i < block->length; ++i)
    {
        if (stream->reset_requested) return;
        stream->bytes[stream->used] = block->data[i];
        stream->time[stream->used++] = block->received_ms;
        while (stream->used)
        {
            if (stream->bytes[0] != 'S' || (stream->used >= 2u && stream->bytes[1] != 'P'))
            { VisionShift(stream, 1u); continue; }
            if (stream->used < sizeof(Vision_Recv_s)) break;
            if (!VisionProtocolCheckCRC16(stream->bytes, sizeof(Vision_Recv_s)))
            { ++vision_crc_error_count; VisionShift(stream, 1u); continue; }
            taskENTER_CRITICAL();
            if (!stream->reset_requested && (uint32_t)(HAL_GetTick() - stream->time[0]) <= 50u)
            {
                memcpy(&recv_data, stream->bytes, sizeof(recv_data));
                ++vision_rx_count;
                vision_last_rx_ms = stream->time[0];
                vision_has_valid_frame = 1u;
                DaemonReload(vision_daemon_instance);
            }
            taskEXIT_CRITICAL();
            VisionShift(stream, sizeof(Vision_Recv_s));
        }
    }
}
#endif

// 离线时清旧命令、接收时间和有效标记；UART 配置还会重新准备接收。
static void VisionOfflineCallback(void *id)
{
    (void)id;
    taskENTER_CRITICAL();
    memset(&recv_data, 0, sizeof(recv_data));
    vision_last_rx_ms = 0u;
    vision_has_valid_frame = 0u;
#ifdef VISION_USE_UART
    vision_stream.reset_requested = 1u;
#endif
    taskEXIT_CRITICAL();
#ifdef VISION_USE_UART
    USARTServiceInit(vision_usart_instance);
#endif
    BSP_LOG_EVENT_0(EV_MODULES_MASTER_MACHINE_MASTER_PROCESS_C_01);
}

void VisionUpdateTx(uint8_t mode,
                    float q0, float q1, float q2, float q3,
                    float yaw, float yaw_vel,
                    float pitch, float pitch_vel,
                    float bullet_speed, uint16_t bullet_count)
{
    send_data.head[0] = 'S';
    send_data.head[1] = 'P';
    send_data.mode = mode;

    send_data.q[0] = q0;
    send_data.q[1] = q1;
    send_data.q[2] = q2;
    send_data.q[3] = q3;

    send_data.yaw = yaw;
    send_data.yaw_vel = yaw_vel;
    send_data.pitch = pitch;
    send_data.pitch_vel = pitch_vel;
    send_data.bullet_speed = bullet_speed;
    send_data.bullet_count = bullet_count;
}

#ifdef VISION_USE_VCP
static void VisionVcpRxCallback(uint16_t recv_len)
{
    DecodeVision(VCPGetCurrentRxData(), recv_len, VCPGetCurrentRxTimestamp());
}
#endif

Vision_Recv_s *VisionInit(UART_HandleTypeDef *_handle)
{
    memset(&recv_data, 0, sizeof(recv_data));
    memset(&send_data, 0, sizeof(send_data));
    vision_last_rx_ms = 0u;
    vision_has_valid_frame = 0u;

    Daemon_Init_Config_s daemon_conf = {
        .callback = VisionOfflineCallback,
        .owner_id = NULL,
        .reload_count = 5, // 50ms
    };
    vision_daemon_instance = DaemonRegister(&daemon_conf);
    if (vision_daemon_instance == NULL)
        return NULL;

#ifdef VISION_USE_VCP
    UNUSED(_handle);
    USB_Init_Config_s conf = {.rx_cbk = VisionVcpRxCallback};
    vis_recv_buff = USBInit(conf);
    if (vis_recv_buff == NULL)
    {
        (void)DaemonUnregister(vision_daemon_instance);
        vision_daemon_instance = NULL;
        return NULL;
    }
#elif defined(VISION_USE_UART)
    if (_handle == NULL)
    {
        (void)DaemonUnregister(vision_daemon_instance);
        vision_daemon_instance = NULL;
        return NULL;
    }
    USART_Init_Config_s conf = {0};
    conf.context = &vision_stream;
    conf.module_callback = VisionUartRxCallback;
    conf.recv_buff_size = VISION_RECV_SIZE * 2;
    conf.usart_handle = _handle;
    vision_usart_instance = USARTRegister(&conf);
    if (vision_usart_instance == NULL)
    {
        (void)DaemonUnregister(vision_daemon_instance);
        vision_daemon_instance = NULL;
        return NULL;
    }
#else
    UNUSED(_handle);
    BSP_LOG_EVENT_0(EV_MODULES_MASTER_MACHINE_MASTER_PROCESS_C_02);
    (void)DaemonUnregister(vision_daemon_instance);
    vision_daemon_instance = NULL;
    return NULL;
#endif

    return &recv_data;
}

// 即使没有离线回调，读取或发送时也检查 50 ms 时限，避免旧目标继续使用。
static void VisionExpireIfStale(uint32_t now_ms)
{
    taskENTER_CRITICAL();
    if (vision_has_valid_frame != 0u &&
        (uint32_t)(now_ms - vision_last_rx_ms) > MESSAGE_DEFAULT_MAX_AGE_MS)
    {
        memset(&recv_data, 0, sizeof(recv_data));
        vision_has_valid_frame = 0u;
    }
    taskEXIT_CRITICAL();
}

#ifdef VISION_USE_UART
static USARTTxEvent_t vision_uart_last_tx;
static void VisionTxCompleted(void *context, const USARTTxEvent_t *event)
{
    (void)context;
    taskENTER_CRITICAL();vision_uart_last_tx=*event;
    if(event->result!=USART_TX_COMPLETE)++vision_tx_drop_count;
    taskEXIT_CRITICAL();
}
#endif
void VisionSend(void)
{
    VisionExpireIfStale(HAL_GetTick());
#ifdef VISION_USE_VCP
    // USB仍按原实现借用持久缓冲；本轮不改变默认USB链路的发送方式。
    static Vision_Send_s tx_pkt;
#else
    // UART在提交时复制完整报文，本次候选只需保留到提交返回。
    Vision_Send_s tx_pkt;
#endif

    tx_pkt = send_data;
    tx_pkt.head[0] = 'S';
    tx_pkt.head[1] = 'P';
    uint32_t crc;
    // CRC 失败时不提交本帧，旧缓冲区不能冒充本次有效状态。
    if (CRCCompute(CRC16_SP_REFEREE, (const uint8_t *)&tx_pkt,
                   sizeof(Vision_Send_s) - 2u, &crc) != CRC_OK)
        return;
    tx_pkt.crc16 = (uint16_t)crc;

#ifdef VISION_USE_VCP
    uint8_t usb_status = USBTransmit((const uint8_t *)&tx_pkt, sizeof(Vision_Send_s));
    if (usb_status == USBD_OK)
        vision_tx_count++;
    else if (usb_status == USBD_BUSY)
        vision_tx_drop_count++;
#elif defined(VISION_USE_UART)
    uint32_t id;
    USARTTxRequest_t request={(const uint8_t *)&tx_pkt,sizeof(tx_pkt),USART_TRANSFER_DMA,
        20u,VisionTxCompleted,NULL,NULL};
    if(USARTSubmitTx(vision_usart_instance,&request,&id)==USART_SEND_ACCEPTED)
        vision_tx_count++;
    else { taskENTER_CRITICAL();++vision_tx_drop_count;taskEXIT_CRITICAL(); }
#endif
}

void VisionGetStatus(Vision_Status_t *status)
{
    VisionGetSnapshot(NULL, status);
}

void VisionGetSnapshot(Vision_Recv_s *frame, Vision_Status_t *status)
{
    VisionExpireIfStale(HAL_GetTick());
    taskENTER_CRITICAL();
    // 两个输出可各自省略；需要哪份数据就复制哪份，整个复制过程放在一起。
    if (frame != NULL)
        *frame = recv_data;
    if (status != NULL)
    {
        status->has_valid_frame = vision_has_valid_frame;
        status->online = (vision_has_valid_frame != 0u && vision_daemon_instance != NULL &&
                          DaemonIsOnline(vision_daemon_instance) > 0) ? 1 : 0;
        status->mode = recv_data.mode;
        status->last_rx_ms = vision_last_rx_ms;
        status->rx_count = vision_rx_count;
        status->tx_count = vision_tx_count;
        status->tx_drop_count = vision_tx_drop_count;
        status->crc_error_count = vision_crc_error_count;
#ifdef VISION_USE_UART
        status->uart_last_tx = vision_uart_last_tx;
#else
        memset(&status->uart_last_tx,0,sizeof(status->uart_last_tx));
#endif
        status->bullet_speed = send_data.bullet_speed;
        status->bullet_count = send_data.bullet_count;
    }
    taskEXIT_CRITICAL();
}
