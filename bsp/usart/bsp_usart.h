/** @file bsp_usart.h
* @brief UART 注册与发送；接收中断复制数据入队，MotorControlTask 统一派发和恢复错误。
* @note 先配置 UART、接收 DMA 及对应中断；每个硬件句柄只注册一次。
 */
#ifndef BSP_RC_H
#define BSP_RC_H

#include <stdint.h>
#include "usart_rx.h"
#include "main.h"

#define DEVICE_USART_CNT 8     // MC02串口数量8个，其中
#define USART_RXBUFF_LIMIT 256 // DMA数组容量，注册容量受uint8_t限制为1～255字节。

/* 发送模式枚举 */
typedef enum
{
    USART_TRANSFER_NONE=0,
    USART_TRANSFER_BLOCKING,
    USART_TRANSFER_IT,
    USART_TRANSFER_DMA,
} USART_TRANSFER_MODE;

/** @brief 仅报告本机发送启动结果；ACCEPTED不表示传输完成或对端收到。 */
typedef enum {
    USART_SEND_ACCEPTED = 0, USART_SEND_BAD_ARGUMENT, USART_SEND_BUSY,
    USART_SEND_ERROR, USART_SEND_TIMEOUT
} USARTSendStatus_t;

// 串口实例结构体,每个module都要包含一个实例.
// 每个硬件句柄独占注册；业务状态通过context传回，接收数组只供HAL使用。
typedef struct
{
    uint8_t recv_buff[USART_RXBUFF_LIMIT]; // 仅供HAL/DMA使用，业务只能读回调收到的不可变块。
    uint8_t recv_buff_size;                // 单次接收容量1～255字节，不等于协议帧长。
    UART_HandleTypeDef *usart_handle;      // 实例对应的usart_handle
    usart_module_callback module_callback; // 解析收到的数据的回调函数
    void *context;                        // 调用者长期保存的上下文，不由BSP释放。
    volatile uint32_t rx_sequence;        // BSP内部连续性计数，不得由业务改写。
    volatile uint32_t rx_generation;      // 错误/重启后隔离此前已排队的数据，仅供BSP使用。
} USARTInstance;

/* usart 初始化配置结构体 */
typedef struct
{
    uint8_t recv_buff_size;                // 单次接收容量1～255字节。
    UART_HandleTypeDef *usart_handle;      // 实例对应的usart_handle
    usart_module_callback module_callback; // 解析收到的数据的回调函数
    void *context;                         // 原样传回回调，非空时须长期保存。
} USART_Init_Config_s;

/** @brief 注册 UART、分配接收队列并启动 DMA 接收。
* @param[in] init_config 有效句柄、非空模块回调及 1～255 字节接收容量，配置会复制。
* @return 成功返回长期有效对象；参数、重复注册、容量或启动失败时返回 NULL。
* @note 启动阶段串行注册，派发任务开始前完成。配置复制，context对象由调用者长期保存。
* 启动失败后若无法确认DMA中止，保留隔离对象及池空间，避免释放硬件仍借用的数组。
 */
USARTInstance *USARTRegister(USART_Init_Config_s *init_config);

/** @brief 显式重新准备DMA，打断接收连续性并隔离此前已排队的块。
* @param[in] _instance 已准备 UART 与接收 DMA 的实例；NULL 或信息不完整时失败。
* @return 1 为启动成功，0 为参数不符或 HAL 拒绝；不会替调用方中止旧接收。
* @note 仅任务上下文对已注册实例串行调用。HAL拒绝时请求下轮派发中有限重试；
* 正常RxEvent后的DMA重装使用内部入口，不打断协议连续性。
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

/** @brief 检查参数并返回HAL发送启动结果，无队列或内部数据复制。
 * @param[in] instance 非空、持有已初始化UART句柄的实例；同一发送者串行调用。
 * @param[in] data 非空，至少length个可读字节；IT/DMA终止前内容保持不变。
 * @param[in] length 字节数，1～65535。
 * @param[in] mode BLOCKING、IT或DMA；其他值返回BAD_ARGUMENT，不等待。
 * @return ACCEPTED、BAD_ARGUMENT、BUSY、ERROR或TIMEOUT；阻塞模式HAL时限100ms。
 * @note 仅任务上下文使用；异步错误和完成须由后续传输管理处理，本函数不证明完成。
 */
USARTSendStatus_t USARTSendChecked(USARTInstance *instance, const uint8_t *data,
                                 uint16_t length, USART_TRANSFER_MODE mode);

/** @brief 检查 HAL 的发送状态是否为 READY。
* @param[in] _instance 有效实例，不能为 NULL。
* @return 1 为发送句柄空闲，0 为其他状态；不表示对端已接收上次数据。
 */
uint8_t USARTIsReady(USARTInstance *_instance);

/**
 * @brief 按接收顺序派发入口时已排队的块，并执行有限错误恢复。
 * @note 全工程只有一个任务调用者（MotorControlTask），递归调用直接返回。
 * 每路四槽可用三槽，满时丢新块；每轮每路最多三块，新到块留到下轮。
 * 超过50ms或错误/重启前的块被丢弃。回调直接借用队列存储，返回前保持只读有效。
 * 不承诺物理耗时或派发延迟；协议自己承担拼帧、校验及来源时间检查。
 */
void USARTProcessRx(void);

#endif
