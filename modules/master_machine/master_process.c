/**
 * @file master_process.c
 * @brief 视觉数据收发, SP+CRC16 协议(与上位机 sp_vision_25 对齐)
 *        支持 USB VCP(VISION_USE_VCP) 和 UART(VISION_USE_UART) 两种传输, 宏切换
 */
#include "master_process.h"
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

_Static_assert(sizeof(Vision_Recv_s) == 29u, "Vision RX frame must stay 29 bytes");
_Static_assert(sizeof(Vision_Send_s) == 43u, "Vision TX frame must stay 43 bytes");

#ifdef VISION_USE_UART
#include "bsp_usart.h"
static USARTInstance *vision_usart_instance;
static uint8_t *vis_recv_buff;
#endif

#ifdef VISION_USE_VCP
#include "bsp_usb.h"
static uint8_t *vis_recv_buff;
#endif

/* 和 sp_vision_25 一致的 CRC16/X25 */
static uint16_t VisionCRC16(const uint8_t *data, uint32_t len)
{
    uint16_t crc = 0xFFFF;

    while (len--)
    {
        crc ^= *data++;
        for (uint8_t i = 0; i < 8; i++)
        {
            if (crc & 0x0001)
                crc = (crc >> 1) ^ 0x8408;
            else
                crc >>= 1;
        }
    }

    return crc;
}

static uint8_t VisionCheckCRC16(const uint8_t *data, uint32_t len)
{
    uint16_t rx_crc = (uint16_t)data[len - 2] | ((uint16_t)data[len - 1] << 8);
    uint16_t calc_crc = VisionCRC16(data, len - 2);
    return (rx_crc == calc_crc);
}

/* 从接收缓冲中滑动寻找 SP 帧并解码 */
static void DecodeVision(uint16_t recv_len)
{
    if (recv_len < sizeof(Vision_Recv_s))
        return;

    for (uint16_t i = 0; i + sizeof(Vision_Recv_s) <= recv_len; i++)
    {
        Vision_Recv_s pkt;
        memcpy(&pkt, vis_recv_buff + i, sizeof(Vision_Recv_s));

        if (pkt.head[0] != 'S' || pkt.head[1] != 'P')
            continue;

        if (!VisionCheckCRC16((const uint8_t *)&pkt, sizeof(Vision_Recv_s)))
        {
            taskENTER_CRITICAL();
            vision_crc_error_count++;
            taskEXIT_CRITICAL();
            continue;
        }

        taskENTER_CRITICAL();
        recv_data = pkt;
        vision_rx_count++;
        vision_last_rx_ms = HAL_GetTick();
        DaemonReload(vision_daemon_instance);
        taskEXIT_CRITICAL();
        return;
    }
}

#ifdef VISION_USE_UART
/* bsp_usart 回调(无参): 数据到达后调用 DecodeVision */
static void VisionUartRxCallback(void)
{
    DaemonReload(vision_daemon_instance);
    DecodeVision(vision_usart_instance->last_recv_size);
}
#endif

static void VisionOfflineCallback(void *id)
{
    (void)id;
    taskENTER_CRITICAL();
    memset(&recv_data, 0, sizeof(recv_data));
    vision_last_rx_ms = 0u;
    taskEXIT_CRITICAL();
#ifdef VISION_USE_UART
    USARTServiceInit(vision_usart_instance);
#endif
    LOGWARNING("[vision] vision offline, restart communication.");
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

Vision_Recv_s *VisionInit(UART_HandleTypeDef *_handle)
{
    memset(&recv_data, 0, sizeof(recv_data));
    memset(&send_data, 0, sizeof(send_data));

    Daemon_Init_Config_s daemon_conf = {
        .callback = VisionOfflineCallback,
        .owner_id = NULL,
        .reload_count = 5, // 50ms
    };
    vision_daemon_instance = DaemonRegister(&daemon_conf);

#ifdef VISION_USE_VCP
    UNUSED(_handle);
    USB_Init_Config_s conf = {.rx_cbk = DecodeVision};
    vis_recv_buff = USBInit(conf);
#elif defined(VISION_USE_UART)
    USART_Init_Config_s conf;
    conf.module_callback = VisionUartRxCallback;
    conf.recv_buff_size = VISION_RECV_SIZE * 2;
    conf.usart_handle = _handle;
    vision_usart_instance = USARTRegister(&conf);
    vis_recv_buff = vision_usart_instance->recv_buff;
#else
    UNUSED(_handle);
    LOGWARNING("[vision] neither VISION_USE_VCP nor VISION_USE_UART defined");
#endif

    return &recv_data;
}

void VisionSend(void)
{
    /* DMA 异步发送需要持久缓冲区, 不能用栈局部变量(函数返回后帧数据会被覆盖) */
    static Vision_Send_s tx_pkt;

    tx_pkt = send_data;
    tx_pkt.head[0] = 'S';
    tx_pkt.head[1] = 'P';
    tx_pkt.crc16 = VisionCRC16((const uint8_t *)&tx_pkt, sizeof(Vision_Send_s) - 2);

#ifdef VISION_USE_VCP
    uint8_t usb_status = USBTransmit((const uint8_t *)&tx_pkt, sizeof(Vision_Send_s));
    if (usb_status == USBD_OK)
        vision_tx_count++;
    else if (usb_status == USBD_BUSY)
        vision_tx_drop_count++;
#elif defined(VISION_USE_UART)
    /* 发送串口忙时跳过本帧, 避免 HAL_UART_Transmit_DMA 返回 BUSY 丢帧 */
    if (!USARTIsReady(vision_usart_instance))
        return;
    USARTSend(vision_usart_instance, (uint8_t *)&tx_pkt, sizeof(Vision_Send_s), USART_TRANSFER_DMA);
    vision_tx_count++;
#endif
}

void VisionGetStatus(Vision_Status_t *status)
{
    if (status == NULL)
        return;

    taskENTER_CRITICAL();
    status->online = (vision_daemon_instance != NULL && DaemonIsOnline(vision_daemon_instance) > 0) ? 1 : 0;
    status->mode = recv_data.mode;
    status->last_rx_ms = vision_last_rx_ms;
    status->rx_count = vision_rx_count;
    status->tx_count = vision_tx_count;
    status->tx_drop_count = vision_tx_drop_count;
    status->crc_error_count = vision_crc_error_count;
    status->bullet_speed = send_data.bullet_speed;
    status->bullet_count = send_data.bullet_count;
    taskEXIT_CRITICAL();
}
