/** @file platform_can.h
 * @brief 按逻辑总线提交有界 CAN 帧组及查询发送结果。
 * @note 接口不暴露芯片句柄；接收由平台端点注册并在任务中派发。
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
    uint32_t received_ticks; /**< 入队时的平台周期计数，32 位回绕；频率由平台计时接口解释。 */
} PlatformCanRxFrame_t;

/** @brief 普通接收派发任务中的回调；帧只在本次调用期间有效。 */
typedef void (*PlatformCanRxCallback_t)(void *context, const PlatformCanRxFrame_t *frame);

/** @brief 逻辑总线的控制器状态，不表示各设备在线。 */
typedef enum {
    PLATFORM_CAN_STATUS_OK, /**< 控制器未报告错误。 */
    PLATFORM_CAN_STATUS_ERROR, /**< 控制器报告错误。 */
    PLATFORM_CAN_STATUS_BUSOFF, /**< 控制器进入总线关闭状态。 */
    PLATFORM_CAN_STATUS_UNKNOWN /**< 总线未绑定或状态无法识别。 */
} PlatformCanStatus_t;

/** @brief 查询逻辑总线的控制器状态。
 * @param[in] bus 板级声明的逻辑总线编号。
 * @return 当前控制器状态；没有绑定时返回 UNKNOWN。
 */
PlatformCanStatus_t PlatformCanGetStatus(PlatformCanBus_t bus);

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

/** @brief 提交一帧一次性 CAN 命令，沿用现有有界重试规则。
 * @param[in] bus 已绑定的逻辑总线。
 * @param[in] source 请求来源标识，必须保持有效直到完成。
 * @param[in] frame 只读帧，提交成功后数据已复制，可立即复用。
 * @param[in] priority 调度优先级，deadline_ms 是入队起的总时限。
 * @param[in] deadline_ms 总期限，单位 ms，范围 1～INT32_MAX。
 * @param[out] request_id 接受时写入编号；失败保持原值。
 * @return 队列提交结果，未绑定时返回 NOT_READY。
 */
CAN_TxSubmitStatus_t PlatformCanSubmitOneShot(PlatformCanBus_t bus, const void *source,
    const CAN_TxFrame_t *frame, CAN_TxPriority_t priority, uint32_t deadline_ms, uint32_t *request_id);

/** @brief 撤销启动期间尚未投入运行的接收注册。
 * @param[in,out] port 已注册端点；NULL 或未注册时不做处理。
 * @note 仅供启动失败回退；不得与接收派发、发送请求或运行中的回调并发。
 * 已排队的本端点帧会被丢弃，其他端点的帧保持原有顺序。
 */
void PlatformCanRxRollback(PlatformCanRxPort_t *port);

/** @brief 登记周期帧，必要时在交给硬件前重新检查并修改队列副本。
 * @param[in] bus 板级声明的逻辑总线。
 * @param[in] source 长期有效的来源标识，也会传给 prepare。
 * @param[in] key 同一来源内的流编号。
 * @param[in] frame 待复制的经典 CAN 帧；提交后数据可复用。
 * @param[in] priority 调度优先级，数值越小越优先。
 * @param[in] deadline_ms 从提交开始的期限，单位 ms，范围 1～INT32_MAX。
 * @param[in] period_us 周期流预算间隔，单位 us，必须大于零。
 * @param[in] prepare 可为空；非空时在硬件接收前修改队列中的帧副本。
 * @param[out] request_id 接受或合并时写入；失败保持原值。
 * @return 提交结果；未绑定的总线返回 NOT_READY。
 * @note 限普通任务调用；同一来源和键的旧目标可被替换。prepare 在短临界区运行，
 * 不得阻塞、分配内存或再次提交 CAN；source 在排队期间必须保持有效。
 * 提交失败时保留旧帧及其回调，硬件已接收的帧无法撤回。
 */
CAN_TxSubmitStatus_t PlatformCanSubmitPeriodic(PlatformCanBus_t bus, const void *source,
    uint32_t key, const CAN_TxFrame_t *frame, CAN_TxPriority_t priority,
    uint32_t deadline_ms, uint32_t period_us, CAN_TxPeriodicPrepareFn_t prepare,
    uint32_t *request_id);

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
