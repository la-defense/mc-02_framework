/** @file bsp_usart.h
 * @brief UART 注册与发送；接收中断复制数据入队，MotorControlTask 统一派发和恢复错误。
 * @note 先配置 UART、接收 DMA 及对应中断；每个硬件句柄只注册一次。
 */
#ifndef BSP_RC_H
#define BSP_RC_H

#include <stdint.h>
#include "main.h"

#define DEVICE_USART_CNT 8     // MC02串口数量8个，其中
#define USART_RXBUFF_LIMIT 256 // 如果协议需要更大的buff,请修改这里

// 模块回调函数,用于解析协议
typedef void (*usart_module_callback)();

/* 发送模式枚举 */
typedef enum
{
    USART_TRANSFER_NONE=0,
    USART_TRANSFER_BLOCKING,
    USART_TRANSFER_IT,
    USART_TRANSFER_DMA,
} USART_TRANSFER_MODE;

// 串口实例结构体,每个module都要包含一个实例.
// 由于串口是独占的点对点通信,所以不需要考虑多个module同时使用一个串口的情况,因此不用加入id;当然也可以选择加入,这样在bsp层可以访问到module的其他信息
typedef struct
{
    uint8_t recv_buff[USART_RXBUFF_LIMIT]; // 预先定义的最大buff大小,如果太小请修改USART_RXBUFF_LIMIT
    uint8_t recv_buff_size;                // 模块接收一包数据的大小
    uint16_t last_recv_size;               // 最近一次回调收到的实际字节数(HAL RxEvent 的 Size)
    UART_HandleTypeDef *usart_handle;      // 实例对应的usart_handle
    usart_module_callback module_callback; // 解析收到的数据的回调函数
    const uint8_t *current_rx_data; // 本次派发的数据，只在模块回调期间可读，不可保存地址到下一轮。
    uint32_t current_rx_timestamp_ms;
} USARTInstance;

/* usart 初始化配置结构体 */
typedef struct
{
    uint8_t recv_buff_size;                // 模块接收一包数据的大小
    UART_HandleTypeDef *usart_handle;      // 实例对应的usart_handle
    usart_module_callback module_callback; // 解析收到的数据的回调函数
} USART_Init_Config_s;

/** @brief 注册 UART、分配接收队列并启动 DMA 接收。
 * @param[in] init_config 有效句柄、非空模块回调及 1～255 字节接收容量，配置会复制。
 * @return 成功返回长期有效对象；参数、重复注册、容量或启动失败时返回 NULL。
 */
USARTInstance *USARTRegister(USART_Init_Config_s *init_config);

/** @brief 启动接收到空闲的 DMA 服务，并关闭半传输通知。
 * @param[in] _instance 已准备 UART 与接收 DMA 的实例；NULL 或信息不完整时失败。
 * @return 1 为启动成功，0 为参数不符或 HAL 拒绝；不会替调用方中止旧接收。
 */
uint8_t USARTServiceInit(USARTInstance *_instance);


/** @brief 按指定模式尝试发送，不返回 HAL 状态，也不提供发送队列。
 * @param[in] _instance 有效实例，不能为 NULL。
 * @param[in] send_buf 至少可读 send_size 字节；IT、DMA 完成前保持不变。
 * @param[in] send_size 字节数。
 * @param[in] mode BLOCKING、IT 或 DMA；NONE 或未知值持续等待。
 * @note 阻塞等待参数为 100 ms；异步发送前检查 USARTIsReady()，忙时 HAL 可能拒绝新发送。
 */
void USARTSend(USARTInstance *_instance, uint8_t *send_buf, uint16_t send_size,USART_TRANSFER_MODE mode);

/** @brief 检查 HAL 的发送状态是否为 READY。
 * @param[in] _instance 有效实例，不能为 NULL。
 * @return 1 为发送句柄空闲，0 为其他状态；不表示对端已接收上次数据。
 */
uint8_t USARTIsReady(USARTInstance *_instance);

/**
 * @brief 派发所有串口已接收的数据 + 处理错误恢复(必须在任务上下文调用)
 * @note  接收中断现在只把数据拷进队列, 协议解析在这里按到达顺序逐帧执行。
 *        全工程只允许一个调用者(当前挂在 1kHz 的 MotorControlTask 里),
 *        放在"用数据之前"调用即可, 解析延迟就是一个任务周期。
 */
void USARTProcessRx(void);

/** @brief 取得本次派发帧地址，仅在模块接收回调中使用。
 * @param[in] instance 对应实例，NULL 时返回 NULL。
 * @return 可读 last_recv_size 字节的地址，回调外为 NULL；不可改写或长期保存。
 */
const uint8_t *USARTGetCurrentRxData(const USARTInstance *instance);
/** @brief 读取当前派发帧入队时的 HAL 毫秒值。
 * @param[in] instance 对应实例。
 * @return 回调中的原接收时间，NULL 或回调外为 0，派发本身不刷新时间。
 */
uint32_t USARTGetCurrentRxTimestamp(const USARTInstance *instance);

#endif
