/** @file usb_transport.h
 * @brief USB单CDC的复制收发和有限重建；初始化后由一个普通任务推进。
 * @note 不包含ST/HAL类型；接收解析和完成通知都在普通任务中执行。
 * @copyright Copyright (c) 2023; MC-02 contributors改造原USB公开入口。
 */
#ifndef MC02_USB_TRANSPORT_H
#define MC02_USB_TRANSPORT_H

#include <stddef.h>
#include <stdint.h>

#define USB_RX_CAPACITY 8u
#define USB_MAX_BYTES 128u
#define USB_RX_BUDGET 4u
#define USB_SOURCE_MAX_AGE_MS 50u
#define USB_TX_TIMEOUT_MS 100u
#define USB_RESTART_LIMIT 3u
#define USB_STABLE_MS 60000u
#define USB_TRANSPORT_TASK_STACK_WORDS 256u

typedef enum {
    USB_OK, USB_ACCEPTED, USB_BAD_ARGUMENT, USB_NOT_INITIALIZED,
    USB_ALREADY_INITIALIZED, USB_CONTEXT_ERROR, USB_NOT_CONFIGURED,
    USB_BUSY, USB_SEND_ERROR, USB_ISOLATED, USB_TIMEBASE_ERROR
} USBStatus_t;

typedef enum {
    USB_PHASE_STOPPED, USB_PHASE_STARTING, USB_PHASE_WAIT_CONFIGURATION,
    USB_PHASE_READY, USB_PHASE_STOPPING, USB_PHASE_DISCONNECT_WAIT,
    USB_PHASE_RETRY_WAIT, USB_PHASE_EXHAUSTED, USB_PHASE_ISOLATED
} USBPhase_t;

typedef enum {
    USB_LINK_CONFIGURED, USB_LINK_RESET, USB_LINK_DECONFIGURED,
    USB_LINK_SUSPENDED, USB_LINK_RESUMED, USB_LINK_DISCONNECTED,
    USB_LINK_TRANSFER_ERROR, USB_LINK_RESTART
} USBLinkReason_t;

typedef enum {
    USB_TX_NONE, USB_TX_COMPLETE, USB_TX_CANCELLED, USB_TX_TIMEOUT,
    USB_TX_ERROR
} USBTxResult_t;

typedef struct {
    const uint8_t *data;
    size_t length;
    uint32_t received_ms; // 原接收通知时间，0合法，不能用派发时刻代替。
    uint32_t sequence;    // 丢弃块也推进编号，分帧者因此可以发现缺块。
    uint32_t generation;  // 每次连接中断后改变，旧连接的数据不可继续使用。
} USBRxBlock_t;

typedef struct {
    uint32_t id, submitted_ms, completed_ms, generation;
    USBTxResult_t result;
} USBTxEvent_t;

typedef void (*USBRxCallback_t)(void *context, const USBRxBlock_t *block);
typedef void (*USBTxCallback_t)(void *context, const USBTxEvent_t *event);
typedef void (*USBLinkCallback_t)(void *context, uint32_t generation, USBLinkReason_t reason);

typedef struct {
    USBRxCallback_t receive;
    USBTxCallback_t complete;
    USBLinkCallback_t link_changed;
    void *context; // 回调及context须持续有效；回调不得等待或保存接收块地址。
} USBTransportConfig_t;

typedef struct {
    USBPhase_t phase;
    USBLinkReason_t last_link_reason;
    USBStatus_t last_error;
    USBTxEvent_t last_tx;
    uint32_t generation, restart_attempts, restart_total, restart_exhausted;
    uint32_t accepted, completed, rejected, cancelled, timeouts, tx_errors;
    uint32_t rx_blocks, rx_dropped, rx_oversized, rx_expired, rx_dispatched;
    uint32_t lifecycle_changes, stable_since_ms;
    uint8_t initialized, rx_used, tx_buffer_held, stable_tracking;
} USBTransportState_t;

/** @brief 注册唯一CDC调用方，不操作外设；只在启动阶段串行调用一次。
 * @param[in] config 可为NULL，表示无业务回调；有配置时按值保存，不保留配置表地址。
 * @return USB_OK成功；重复初始化返回USB_ALREADY_INITIALIZED且保留原状态。
 * @note 普通任务或调度前调用；不能从ISR调用。先初始化，再推进或提交。
 */
USBStatus_t USBTransportInit(const USBTransportConfig_t *config);

/** @brief 复制1～128字节并立即尝试发送，忙时不保存等待项。
 * @param[in] data 至少可读length字节；成功返回后可以改写原存储。
 * @param[in] length 字节数，允许1～USB_MAX_BYTES。
 * @param[out] id 成功写非零编号；失败保留原值，不能为NULL。
 * @return USB_ACCEPTED仅代表本机接受；其他状态不产生完成通知。
 * @note 仅普通任务；完成回调不暴露内部缓冲，结果由唯一推进任务通知一次。
 */
USBStatus_t USBTransportSubmit(const uint8_t *data, size_t length, uint32_t *id);

/** @brief 唯一生命周期任务每轮调用，最多一次后端操作及一次完成／连接通知。
 * @return USB_OK正常；重入返回USB_BUSY，ISR返回USB_CONTEXT_ERROR。
 * @note 不执行协议解析；重建等待跨多轮进行，停止失败保持隔离。
 */
USBStatus_t USBTransportProcess(void);

/** @brief 唯一解析任务每轮派发最多4个接收块；回调返回后接收块地址失效。
 * @return USB_OK正常；重入、ISR或未初始化返回相应错误。
 */
USBStatus_t USBTransportProcessRx(void);

/** @brief 普通任务复制当前状态，不执行发送、恢复或续期。
 * @param[out] output 不能为NULL；失败保留原值。
 * @return USB_OK成功，其他状态表示参数、上下文或初始化错误。
 */
USBStatus_t USBTransportGetState(USBTransportState_t *output);

/** @brief 检查业务数据是否仍属于当前可用连接，ISR可用。
 * @param[in] generation 该数据接收时的连接代号。
 * @return 1表示当前连接仍可使用，0表示必须撤销旧数据。
 */
uint8_t USBTransportGenerationIsCurrent(uint32_t generation);

/** @brief 业务完整合法帧发布后报告原时间，用于判断60秒双向通信。
 * @param[in] generation 接收块的连接代号。
 * @param[in] received_ms 原接收毫秒值，0合法；本接口不替业务延长期限。
 * @note 仅任务调用；过期或旧连接报告忽略，不产生新命令。
 */
void USBTransportReportValidCommand(uint32_t generation, uint32_t received_ms);

#endif
