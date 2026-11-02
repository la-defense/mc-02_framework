/** @file platform_can.h
 * @brief 按逻辑总线提交有界 CAN 帧组及查询发送结果。
 * @note 接口不暴露芯片句柄；接收注册仍使用现有设备接口。
 */
#ifndef MC02_PLATFORM_CAN_H
#define MC02_PLATFORM_CAN_H
#include "can_tx.h"
#include "../board/platform_resources.h"

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
