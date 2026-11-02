/** @file platform_can.h
 * @brief 按逻辑总线提交有界 CAN 帧组及查询发送结果。
 * @note 接口不暴露芯片句柄；接收注册仍使用现有设备接口。
 */
#ifndef MC02_PLATFORM_CAN_H
#define MC02_PLATFORM_CAN_H
#include "can_tx.h"
#include "../board/platform_resources.h"

/** @brief 一帧经典 CAN 接收结果；时间是进入接收队列时的本机毫秒计数。 */
typedef struct {
    uint8_t data[8]; /**< 接收字节，只有 length 范围内有效。 */
    uint8_t length; /**< 实际接收长度，范围 0～8。 */
    uint32_t received_ms; /**< 接收队列记录的原始本机时间，允许回绕。 */
} PlatformCanRxFrame_t;

/** @brief 普通接收派发任务中的回调；帧只在本次调用期间有效。 */
typedef void (*PlatformCanRxCallback_t)(void *context, const PlatformCanRxFrame_t *frame);

/** @brief 调用者长期保存的接收端点；初始化前清零，注册后不可复制或改写。 */
typedef struct {
    void *backend; /**< 平台私有对象；业务不得读取或修改。 */
    PlatformCanRxCallback_t callback; /**< 平台派发时调用的处理函数。 */
    void *context; /**< 原样交给处理函数的长期上下文。 */
} PlatformCanRxPort_t;

/** @brief 查询板级表是否声明了逻辑 CAN 总线。
 * @param[in] bus 车型使用的逻辑总线编号。
 * @return 1 为有板级绑定，0 为未绑定；不代表外设已初始化或设备在线。
 */
uint8_t PlatformCanBusIsBound(PlatformCanBus_t bus);

/** @brief 注册一个标准 ID 接收端点。
 * @param[in,out] port 调用前清零，成功后长期保存在原地址；失败不修改。
 * @param[in] endpoint 本板逻辑总线和收发 ID，调用期间可读。
 * @param[in] callback 非空处理函数，由接收派发任务调用。
 * @param[in] context 交给回调的上下文，可为 NULL；非空时须长期有效。
 * @return 1 为注册成功，0 为配置、绑定或底层资源错误。
 * @note 仅在调度器启动前串行注册，注册期间不能并发派发接收。
 */
uint8_t PlatformCanRxRegister(PlatformCanRxPort_t *port, const PlatformCanEndpoint_t *endpoint,
                              PlatformCanRxCallback_t callback, void *context);

/** @brief 查询端点是否已完成注册。
 * @param[in] port 长期保存的端点；可为 NULL。
 * @return 1 为已注册，0 为未注册或参数为空；不代表设备在线。
 */
uint8_t PlatformCanRxIsRegistered(const PlatformCanRxPort_t *port);

/** @brief 撤销启动期间尚未投入运行的接收注册。
 * @param[in,out] port 已注册端点；NULL 或未注册时不做处理。
 * @note 仅供启动失败回退；不得与接收派发、发送请求或运行中的回调并发。
 * 已排队的本端点帧会被丢弃，其他端点的帧保持原有顺序。
 */
void PlatformCanRxRollback(PlatformCanRxPort_t *port);

/** @brief 提交可被高优先级请求中断的帧组。
 * @param[in] bus 板级声明的逻辑总线。
 * @param[in] source 长期有效的请求来源标识，完成前不能复用其地址。
 * @param[in] frames,count 可读帧及数量；成功提交时复制数据，返回后可复用。
 * @param[in] priority 队列优先级；deadline_ms 从入队开始，单位 ms，范围 1～INT32_MAX。
 * @param[in] deadline_ms 从入队开始的总期限，单位 ms。
 * @param[out] request_id 成功时写入非零编号；失败保持原值。
 * @return 队列提交结果；总线未绑定返回 NOT_READY。
 * @note 限普通任务调用；成功只表示队列接收，不表示发送完成。
 */
CAN_TxSubmitStatus_t PlatformCanSubmitBatch(PlatformCanBus_t bus, const void *source,
    const CAN_TxFrame_t *frames, size_t count, CAN_TxPriority_t priority,
    uint32_t deadline_ms, uint32_t *request_id);

/** @brief 使用本机绝对截止时间提交可中断帧组。
 * @param[in] bus,source,frames,count,priority 与 PlatformCanSubmitBatch 相同。
 * @param[in] expiry_ms 本机毫秒计数的绝对截止时间，允许 32 位回绕。
 * @param[out] request_id 成功时写入；失败保持原值。
 * @return 队列提交结果；过期数据或非法参数由队列拒绝。
 */
CAN_TxSubmitStatus_t PlatformCanSubmitBatchUntil(PlatformCanBus_t bus, const void *source,
    const CAN_TxFrame_t *frames, size_t count, CAN_TxPriority_t priority,
    uint32_t expiry_ms, uint32_t *request_id);

/** @brief 查询发送请求结果；失败不改变输出。
 * @param[in] request_id 非零请求编号。
 * @param[out] result 已找到时接收结果。
 * @return 1 为找到记录，0 为无效或记录已回收。
 */
uint8_t PlatformCanGetRequest(uint32_t request_id, CAN_TxResult_t *result);

/** @brief 取消队列中尚未交给硬件的帧。
 * @param[in] request_id 非零请求编号。
 * @return 1 为已取消剩余帧，0 为请求不存在或无法取消。
 */
uint8_t PlatformCanCancelRequest(uint32_t request_id);

/** @brief 推进指定总线最多一帧。
 * @param[in] bus 板级声明的逻辑总线。
 * @return 1 为总线已绑定并已调用队列服务，0 为未绑定；不表示帧已发送。
 * @note 仅普通任务串行调用，不等待硬件空闲。
 */
uint8_t PlatformCanProcessTxBus(PlatformCanBus_t bus);
#endif
