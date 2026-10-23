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
#include <math.h>

static Vision_Recv_s recv_data;
static Vision_Send_s send_data;
static DaemonInstance *vision_daemon_instance;
static uint32_t vision_rx_count = 0;
static uint32_t vision_tx_count = 0;
static uint32_t vision_tx_drop_count = 0;
static uint32_t vision_crc_error_count = 0;
static uint32_t vision_last_rx_ms = 0;
static uint8_t vision_has_valid_frame;
static VisionSPStream_t vision_stream;
static volatile uint8_t vision_stream_reset_requested;

#ifdef VISION_USE_UART
#include "bsp_usart.h"
static USARTInstance *vision_usart_instance;
#endif

#ifdef VISION_USE_VCP
#include "usb_transport.h"
static uint32_t vision_usb_generation;
#endif

// 线上CRC通过后继续检查实际字段；解析不授予使能，控制端仍检查范围和当前许可。
static void VisionFrameReceived(void *context, const Vision_Recv_s *packet,
                                uint32_t received_ms, uint32_t generation)
{
    (void)context;
    if (packet->mode > 2u || !isfinite(packet->yaw) || !isfinite(packet->yaw_vel) ||
        !isfinite(packet->yaw_acc) || !isfinite(packet->pitch) || !isfinite(packet->pitch_vel) ||
        !isfinite(packet->pitch_acc))
        return;
    taskENTER_CRITICAL();
    uint8_t current = (uint8_t)(vision_stream_reset_requested == 0u &&
        (uint32_t)(HAL_GetTick() - received_ms) <= MESSAGE_DEFAULT_MAX_AGE_MS);
#ifdef VISION_USE_VCP
    current = (uint8_t)(current && USBTransportGenerationIsCurrent(generation));
#else
    (void)generation;
#endif
    if (current)
    {
        recv_data = *packet;
        vision_rx_count++;
        vision_last_rx_ms = received_ms;
        vision_has_valid_frame = 1u;
#ifdef VISION_USE_VCP
        vision_usb_generation = generation;
        USBTransportReportValidCommand(generation, received_ms);
#endif
        DaemonReload(vision_daemon_instance);
    }
    taskEXIT_CRITICAL();
}

// 只有解析调用者清理候选数组；其他任务只置请求标志，避免清零与解析交错。
static void VisionFeedBlock(const uint8_t *data, size_t length, uint32_t received_ms,
                            uint32_t sequence, uint32_t generation)
{
    taskENTER_CRITICAL();
    uint8_t reset = vision_stream_reset_requested;
    vision_stream_reset_requested = 0u;
    taskEXIT_CRITICAL();
    if (reset) VisionProtocolStreamReset(&vision_stream);
    uint32_t errors = 0u;
    (void)VisionProtocolStreamFeed(&vision_stream, data, length, received_ms, sequence,
        generation, HAL_GetTick(), VisionFrameReceived, NULL, &errors);
    taskENTER_CRITICAL();
    vision_crc_error_count += errors;
    taskEXIT_CRITICAL();
}

#ifdef VISION_USE_UART
static void VisionUartRxCallback(void *context, const USART_RxBlock_t *block)
{
    (void)context;
    if (block == NULL) return;
    VisionFeedBlock(block->data, block->length, block->received_ms, block->sequence, 1u);
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
    vision_stream_reset_requested = 1u;
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
    taskENTER_CRITICAL();
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
    taskEXIT_CRITICAL();
}

#ifdef VISION_USE_VCP
static void VisionVcpRxCallback(void *context, const USBRxBlock_t *block)
{
    (void)context;
    if (block == NULL) return;
    VisionFeedBlock(block->data, block->length, block->received_ms, block->sequence, block->generation);
}
static void VisionVcpLinkChanged(void *context, uint32_t generation, USBLinkReason_t reason)
{
    (void)context; (void)generation; (void)reason;
    taskENTER_CRITICAL();
    // 新连接可能已先收到首帧；迟一点执行的连接通知不能再清掉这份新数据。
    if (!vision_has_valid_frame || vision_usb_generation != generation ||
        !USBTransportGenerationIsCurrent(generation))
    {
        memset(&recv_data, 0, sizeof(recv_data));
        vision_has_valid_frame = 0u;
        vision_last_rx_ms = 0u;
        vision_stream_reset_requested = 1u;
    }
    taskEXIT_CRITICAL();
}
static void VisionVcpTxCompleted(void *context, const USBTxEvent_t *event)
{
    (void)context;
    if (event->result != USB_TX_COMPLETE)
    { taskENTER_CRITICAL(); ++vision_tx_drop_count; taskEXIT_CRITICAL(); }
}
#endif

Vision_Recv_s *VisionInit(UART_HandleTypeDef *_handle)
{
    memset(&recv_data, 0, sizeof(recv_data));
    memset(&send_data, 0, sizeof(send_data));
    vision_last_rx_ms = 0u;
    vision_has_valid_frame = 0u;
    VisionProtocolStreamReset(&vision_stream);
    vision_stream_reset_requested = 0u;

    Daemon_Init_Config_s daemon_conf = {
        .callback = VisionOfflineCallback,
        .owner_id = NULL,
        .reload_count = 5, // 50ms
    };
    vision_daemon_instance = DaemonRegister(&daemon_conf);
    if (vision_daemon_instance == NULL)
        return NULL;

#ifdef VISION_USE_VCP
    (void)_handle;
    USBTransportConfig_t conf = {VisionVcpRxCallback, VisionVcpTxCompleted, VisionVcpLinkChanged, NULL};
    if (USBTransportInit(&conf) != USB_OK)
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
    (void)_handle;
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
    uint8_t expired = (uint8_t)((uint32_t)(now_ms - vision_last_rx_ms) > MESSAGE_DEFAULT_MAX_AGE_MS);
#ifdef VISION_USE_VCP
    expired = (uint8_t)(expired || !USBTransportGenerationIsCurrent(vision_usb_generation));
#endif
    if (vision_has_valid_frame != 0u && expired)
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
    // 两种传输都在提交时复制；组包使用本次独占的局部内容。
    Vision_Send_s tx_pkt;
    taskENTER_CRITICAL();
    tx_pkt = send_data;
    taskEXIT_CRITICAL();
    tx_pkt.head[0] = 'S';
    tx_pkt.head[1] = 'P';
    uint32_t crc;
    // CRC 失败时不提交本帧，旧缓冲区不能冒充本次有效状态。
    if (CRCCompute(CRC16_SP_REFEREE, (const uint8_t *)&tx_pkt,
                   sizeof(Vision_Send_s) - 2u, &crc) != CRC_OK)
        return;
    tx_pkt.crc16 = (uint16_t)crc;

#ifdef VISION_USE_VCP
    uint32_t id;
    USBStatus_t usb_status = USBTransportSubmit((const uint8_t *)&tx_pkt, sizeof(tx_pkt), &id);
    taskENTER_CRITICAL();
    if (usb_status == USB_ACCEPTED)
        vision_tx_count++;
    else
        vision_tx_drop_count++;
    taskEXIT_CRITICAL();
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
