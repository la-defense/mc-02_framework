/** @file usart_tx.h
 * @brief UART发送请求与结果的纯数据声明；无HAL和RTOS依赖。
 * @note 数据生命周期、容量及通知条件见各字段；先注册，再调用bsp_usart.h入口。
 */
#ifndef MC02_USART_TX_H
#define MC02_USART_TX_H
#include <stddef.h>
#include <stdint.h>
/* 发送模式枚举 */
typedef enum
{
    USART_TRANSFER_NONE = 0,
    USART_TRANSFER_BLOCKING,
    USART_TRANSFER_IT,
    USART_TRANSFER_DMA,
} USART_TRANSFER_MODE;

/** @brief 提交结果；ACCEPTED仅表示已复制入队，未表示传输完成。 */
typedef enum
{
    USART_SEND_ACCEPTED = 0,
    USART_SEND_BAD_ARGUMENT,
    USART_SEND_BUSY,
    USART_SEND_ERROR,
    USART_SEND_TIMEOUT,
    USART_SEND_NOT_READY,
    USART_SEND_ISOLATED,
    USART_SEND_NOT_FOUND
} USARTSendStatus_t;

#define USART_TX_DEPTH 4u
#define USART_TX_MAX_BYTES 128u
/** @brief 每条已入队报文最终通知一次；隔离表示内部缓冲仍被保留。 */
typedef enum
{
    USART_TX_COMPLETE,
    USART_TX_FAILED,
    USART_TX_EXPIRED,
    USART_TX_CANCELLED,
    USART_TX_ISOLATED
} USARTTxResult_t;
typedef struct
{
    uint32_t id, submitted_ms, started_ms, completed_ms;
    USARTTxResult_t result;
    uint8_t started;
} USARTTxEvent_t;
/** @brief 仅发送任务调用；event只在回调期间有效，context须保存至通知结束。 */
typedef void (*USARTTxCallback_t)(void *context, const USARTTxEvent_t *event);
/** @brief 发送前检查业务身份；只在发送任务调用，返回0则取消本条，不得等待。
 * @note data只在本次检查期间可读，不可保留地址或修改队列数据。
 */
typedef uint8_t (*USARTTxGuard_t)(void *context, const uint8_t *data, size_t length);
typedef struct
{
    const uint8_t *data;        /**< 提交期间可读length字节，成功后可立即复用。 */
    size_t length;              /**< 字节数1～128。 */
    USART_TRANSFER_MODE mode;   /**< 只接受IT或DMA，所需外设及IRQ必须已配置。 */
    uint32_t timeout_ms;        /**< 从入队起的总期限1～INT32_MAX ms，达到即到期。 */
    USARTTxCallback_t complete; /**< 必须非NULL，短小、不等待，允许提交新报文。 */
    USARTTxGuard_t guard;       /**< 可为NULL；非NULL时实际启动前检查一次。 */
    void *context;              /**< 调用方长期保存，BSP不释放。 */
} USARTTxRequest_t;
typedef struct
{
    uint8_t used, active, isolated, service_ready;
    uint32_t accepted, completed, failed, rejected;
    USARTTxEvent_t last_event;
} USARTTxState_t;

#endif
