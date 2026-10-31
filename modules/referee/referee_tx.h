/** @file referee_tx.h
 * @brief 裁判共享发送的纯数据接口；RefereeInit及UART发送任务准备后从普通任务提交。
 * @note 本机完成不表示对方接收或显示；上下文保存至最终通知结束。
 */
#ifndef MC02_REFEREE_TX_H
#define MC02_REFEREE_TX_H
#include <stddef.h>
#include <stdint.h>
#include "usart_tx.h"

#define REFEREE_TX_FRAME_MAX 127u
#define REFEREE_TX_CONTENT_MAX 112u
#define REFEREE_TX_DEFAULT_TIMEOUT_MS 100u
typedef enum
{
    REFEREE_SEND_ACCEPTED = 0,
    REFEREE_SEND_BAD_ARGUMENT,
    REFEREE_SEND_NOT_INITIALIZED,
    REFEREE_SEND_BUSY,
    REFEREE_SEND_ERROR,
    REFEREE_SEND_TIMEOUT,
    REFEREE_SEND_CRC,
    REFEREE_SEND_IDENTITY,
    REFEREE_SEND_ISOLATED
} RefereeSendStatus_t;
typedef struct
{
    uint16_t receiver_id, content_id;
    const uint8_t *content;     /**< 提交期间可读length字节；length为0可NULL。 */
    size_t length;              /**< 机器人0～112字节，UI必须匹配固定子命令长度。 */
    uint32_t timeout_ms;        /**< 入队起总期限1～INT32_MAX ms，含限流等待。 */
    USARTTxCallback_t complete; /**< 非NULL，发送任务通知一次，不等待。 */
    USARTTxGuard_t guard;       /**< 可NULL，实际启动前检查业务代号，不保存data地址。 */
    void *context;              /**< 长期保存至最终通知结束。 */
} RefereeTxRequest_t;
typedef enum
{
    REFEREE_TX_WAIT_NONE,
    REFEREE_TX_WAIT_RATE,
    REFEREE_TX_WAIT_BYTES
} RefereeTxWait_t;
typedef struct
{
    USARTTxState_t uart; /**< 已接受、完成、拒绝及失败统计；last_event保留最近通知。 */
    uint32_t rejected, validation_rejected, cancelled, expired, errors, gate_deferrals;
    uint32_t last_start_ms;
    uint8_t has_start, next_sequence;
    RefereeTxWait_t wait_reason;
} RefereeTxState_t;

/** @brief 检查身份、子命令和长度，构造完整帧并复制入四条UART队列。
 * @param[in] request 非NULL，源内容仅提交期间借用；发送者来自新鲜0x0201。
 * @param[out] id 非NULL，成功写入非零编号；拒绝保持原值、不通知、不消耗序号。
 * @return 具体提交状态，ACCEPTED不表示已启动；并发提交争用时立即返回BUSY。
 * @note 普通任务使用；UI保留上层115ms节奏。入队后不自动重发。
 *       机器人交互检查初始化时配置的目标掩码；具体赛项及阶段的规则由配置者核验。
 */
RefereeSendStatus_t RefereeSubmit(const RefereeTxRequest_t *request, uint32_t *id);
/** @brief 请求取消本模块编号；终态观察及通知仍在UART发送任务中执行。
 * @param[in] id 成功提交的非零编号。
 * @return USART取消结果；在途字节不能撤回，已经形成的终态不改写。
 */
USARTSendStatus_t RefereeCancelSend(uint32_t id);
/** @brief 复制发送统计及限流原因，不消费数据、不推进调度。
 * @param[out] output 非NULL；未初始化或参数错误保持原输出。
 * @return ACCEPTED、NOT_READY或BAD_ARGUMENT；普通任务使用。
 * @note UART与裁判统计分别复制，适合诊断；不作为跨模块原子读取或控制许可。
 */
USARTSendStatus_t RefereeGetTxState(RefereeTxState_t *output);
#endif
