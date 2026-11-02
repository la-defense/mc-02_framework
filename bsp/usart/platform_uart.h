/** @file platform_uart.h
 * @brief 使用逻辑串口注册接收并访问现有有界发送队列。
 */
#ifndef MC02_PLATFORM_UART_H
#define MC02_PLATFORM_UART_H
#include "../board/platform_resources.h"
#include "usart_rx.h"
#include "usart_tx.h"

/** @brief 平台持有的串口实例；调用者只能保存指针，不得访问内部数据。 */
typedef void PlatformUartInstance;

/** @brief 注册时复制的接收和发送检查配置。 */
typedef struct {
    uint8_t receive_capacity; /**< 一次接收容量，范围 1～255 字节。 */
    usart_module_callback receive; /**< 任务派发的只读接收块回调。 */
    void *context; /**< 回调共用的长期有效上下文，可为 NULL。 */
    USARTTxGuard_t tx_guard; /**< 可选的发送前业务检查。 */
    USARTTxGate_t tx_gate; /**< 可选的发送额度检查。 */
    USARTTxObserver_t tx_observer; /**< 可选的终态观察回调。 */
} PlatformUartConfig;

/** @brief 查询逻辑串口是否有板级绑定。
 * @param[in] port 车型指定的逻辑编号。
 * @return 1 表示已绑定，0 表示未绑定；不表示注册或通信成功。
 */
uint8_t PlatformUartIsBound(PlatformUartPort_t port);

/** @brief 启动阶段注册接收，当前 H723 后端立即启动 DMA。
 * @param[in] port 已绑定的逻辑编号。
 * @param[in] config 调用期间有效的配置，回调及上下文被复制。
 * @return 成功返回长期有效实例；失败返回 NULL，底层可能隔离仍被硬件占用的资源。
 * @note 与其他注册串行执行；接收回调和发送回调在任务中执行，上下文须持续有效。
 */
PlatformUartInstance *PlatformUartRegister(PlatformUartPort_t port, const PlatformUartConfig *config);

/** @brief 任务中重新准备接收；旧块按底层规则失效。
 * @param[in] instance 已注册实例。
 * @return 1 表示成功，0 表示失败。
 */
uint8_t PlatformUartRestartReceive(PlatformUartInstance *instance);

/** @brief 复制提交有界发送请求。
 * @param[in] instance 已注册实例。
 * @param[in] request 发送请求，成功提交后源数据可复用。
 * @param[out] id 成功时交付编号，失败时保持原值。
 * @return 实际提交结果。
 */
USARTSendStatus_t PlatformUartSubmit(PlatformUartInstance *instance, const USARTTxRequest_t *request, uint32_t *id);

/** @brief 取消指定发送请求。
 * @param[in] instance 已注册实例。
 * @param[in] id 提交时获得的编号。
 * @return 实际取消结果，最终状态仍由回调报告。
 */
USARTSendStatus_t PlatformUartCancel(PlatformUartInstance *instance, uint32_t id);

/** @brief 读取发送状态的副本。
 * @param[in] instance 已注册实例。
 * @param[out] output 成功时写入结果，失败时保持原值。
 * @return 实际查询结果。
 */
USARTSendStatus_t PlatformUartGetTxState(const PlatformUartInstance *instance, USARTTxState_t *output);
#endif
