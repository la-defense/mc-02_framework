/** @file can_time_sync.h
 * @brief 通过经典 CAN 交换双板时刻，为远端消息提供本地时间。
 * @note 对象和端点标识长期保存；所有入口由同一任务串行调用。
 */
#ifndef CAN_TIME_SYNC_H
#define CAN_TIME_SYNC_H

#include "../../bsp/can/platform_can.h"
#include "time_mapping.h"
#include <stdint.h>

#define CAN_TIME_SYNC_VALID_MS 500u

typedef enum
{
    CAN_TIME_SYNC_OK = 0,
    CAN_TIME_SYNC_WAITING,
    CAN_TIME_SYNC_ARGUMENT,
    CAN_TIME_SYNC_FRAME_ERROR,
    CAN_TIME_SYNC_BUSY,
    CAN_TIME_SYNC_SEND_ERROR,
    CAN_TIME_SYNC_PEER_RESET
} CanTimeSyncStatus_t;

/** @brief 调用者提供的固定存储；直接修改成员会破坏交换过程。 */
typedef struct
{
    TimeMapping_t mapping;
    PlatformCanBus_t bus; /**< 本板发送使用的逻辑总线。 */
    uint32_t tx_id; /**< 对时帧使用的标准 CAN 发送 ID。 */
    const void *endpoint_identity; /**< 调用方持有的长期端点标识，用于绑定同一接收链路。 */
    uint32_t probe_t1_ms;
    uint32_t previous_probe_ms;
    uint32_t peer_t2_ms;
    uint32_t reply_t2_received_ms;
    uint32_t last_peer_ms;
    uint32_t probe_request_id;
    uint32_t response_request_id;
    uint16_t next_sequence;
    uint16_t pending_sequence;
    uint8_t initialized;
    uint8_t has_sent;
    uint8_t waiting;
    uint8_t has_t2;
    uint8_t has_peer_time;
} CanTimeSync_t;

/** @brief 初始化一个双向对时实例，不注册额外 CAN 接收端点。
 * @param[out] sync 调用前清零；成功后长期保存，失败不改原值。
 * @param[in] bus 本板逻辑 CAN 总线；实际绑定在发送时由平台确认。
 * @param[in] tx_id 发送使用的标准 CAN ID，范围 0～0x7FF。
 * @param[in] endpoint_identity 已注册端点的长期标识，不由对时模块解引用。
 * @return 1为成功，0为参数错误。
 * @note bus 和 tx_id 须来自该端点的配置；绑定链路时再次检查是否匹配。
 */
uint8_t CanTimeSyncInit(CanTimeSync_t *sync, PlatformCanBus_t bus, uint32_t tx_id,
                        const void *endpoint_identity);

/** @brief 按本机时间检查上次请求，并在需要时提交新的探测帧。
 * @param[in,out] sync 已初始化的实例。
 * @param[in] now_ms 本机毫秒，允许0及32位回绕。
 * @return 已提交、无需提交、发送失败或参数错误。
 * @note 每次调用有界且不等待 CAN；只有一个请求可等待响应。
 */
CanTimeSyncStatus_t CanTimeSyncService(CanTimeSync_t *sync, uint32_t now_ms);

/** @brief 处理一帧对时数据，必要时提交包含两个时刻的响应。
 * @param[in,out] sync 已初始化的实例。
 * @param[in] data 本帧可读数据；对时帧必须为8字节。
 * @param[in] length 实际接收长度。
 * @param[in] received_ms 接收中断入队时记录的本机毫秒。
 * @param[in] now_ms 任务处理时的本机毫秒。
 * @return 已处理、等待后一帧、对端重启、发送失败或帧错误。
 * @note 对端重启时调用者须撤销此前收到的消息，再等待重新对时。
 */
CanTimeSyncStatus_t CanTimeSyncReceive(CanTimeSync_t *sync, const uint8_t *data,
                                       uint8_t length, uint32_t received_ms, uint32_t now_ms);

/** @brief 复制当前对时状态，不改变样本或延长有效期。
 * @param[in] sync 已初始化实例。
 * @param[in] now_ms 本机当前毫秒。
 * @param[out] output 接收偏差、样本数和同步状态；失败不改原值。
 * @return TIME_MAPPING_OK 或参数／未初始化／时间错误。
 */
TimeMappingStatus_t CanTimeSyncGetState(const CanTimeSync_t *sync, uint32_t now_ms,
                                       TimeMappingState_t *output);

/** @brief 将远端来源时间换算到本机并给出允许误差。
 * @param[in] context CanTimeSync_t实例，可作为桥接时间回调直接使用。
 * @param[in] remote_ms 远端来源毫秒，允许0及回绕。
 * @param[in] now_ms 本机当前毫秒。
 * @param[out] local_ms,uncertainty_ms 本机时间及误差上限；失败不改原值。
 * @return 1为映射可用；未同步、过期或参数无效时返回0。
 * @note 误差估计依赖两板时基持续运行，且相对频率误差不超过0.2%。
 */
uint8_t CanTimeSyncMap(void *context, uint32_t remote_ms, uint32_t now_ms,
                       uint32_t *local_ms, uint32_t *uncertainty_ms);

#endif
