/** @file time_mapping.h
 * @brief 把远端毫秒时刻换算到本机毫秒时基。
 * @note 计算只使用标准 C。实例由调用者长期保存，初始化后才可更新或查询。
 */
#ifndef MC02_TIME_MAPPING_H
#define MC02_TIME_MAPPING_H

#include <stdint.h>

#define TIME_MAPPING_REQUIRED_SAMPLES 3u
#define TIME_MAPPING_DEFAULT_MAX_RTT_MS 50u
#define TIME_MAPPING_DEFAULT_MAX_EXCHANGE_MS 100u
#define TIME_MAPPING_DEFAULT_VALID_FOR_MS 2000u
#define TIME_MAPPING_DEFAULT_MAX_OFFSET_JUMP_MS 50u

typedef enum
{
    TIME_MAPPING_OK = 0,
    TIME_MAPPING_ERR_ARGUMENT,
    TIME_MAPPING_ERR_CONFIG,
    TIME_MAPPING_ERR_UNINITIALIZED,
    TIME_MAPPING_ERR_TIME,
    TIME_MAPPING_ERR_ROUND_TRIP,
    TIME_MAPPING_ERR_NOT_SYNCHRONIZED,
    TIME_MAPPING_ERR_EXPIRED
} TimeMappingStatus_t;

/** @brief 时间映射接受样本和失效条件；所有时长单位均为毫秒。 */
typedef struct
{
    uint32_t max_round_trip_ms;
    uint32_t max_exchange_ms;
    uint32_t valid_for_ms;
    uint32_t max_offset_jump_ms;
} TimeMappingConfig_t;

/** @brief 调用者提供的固定存储；字段仅供 TimeMapping 实现使用。 */
typedef struct
{
    TimeMappingConfig_t config;
    int32_t offset_samples_ms[TIME_MAPPING_REQUIRED_SAMPLES];
    int32_t remote_minus_local_ms;
    uint32_t last_local_sample_ms;
    uint32_t last_remote_sample_ms;
    uint32_t last_round_trip_ms;
    uint32_t accepted_samples;
    uint32_t rejected_samples;
    uint8_t sample_count;
    uint8_t next_sample;
    uint8_t initialized;
} TimeMapping_t;

/** @brief 可复制读取的偏差与同步状态。 */
typedef struct
{
    int32_t remote_minus_local_ms;
    uint32_t last_local_sample_ms;
    uint32_t sample_age_ms;
    uint32_t last_round_trip_ms;
    uint32_t accepted_samples;
    uint32_t rejected_samples;
    uint8_t has_sample;
    uint8_t synchronized;
    uint8_t sample_count;
} TimeMappingState_t;

/** @brief 使用给定参数初始化映射；NULL配置采用本模块默认值。
 * @param[out] mapping 调用者长期保存的实例；可与原有效实例相同。
 * @param[in] config 可为空；否则时长须在1～INT32_MAX毫秒，RTT不得大于总交换期限。
 * @return TIME_MAPPING_OK；参数非法时返回错误且不改原实例。
 */
TimeMappingStatus_t TimeMappingInit(TimeMapping_t *mapping,
                                    const TimeMappingConfig_t *config);

/** @brief 清除样本和统计，保留已验证配置，重新等待三个有效交换。
 * @param[in,out] mapping 已初始化实例。
 * @return TIME_MAPPING_OK或空／未初始化错误；失败不改实例。
 */
TimeMappingStatus_t TimeMappingReset(TimeMapping_t *mapping);

/** @brief 提交一次四时刻测量并更新远端相对本机的毫秒偏差。
 * @param[in,out] mapping 已初始化实例；同一实例由调用者串行访问。
 * @param[in] t1_ms MCU发送探测的本机毫秒时刻，0合法。
 * @param[in] t2_ms PC收到探测的远端毫秒时刻。
 * @param[in] t3_ms PC发送响应的远端毫秒时刻。
 * @param[in] t4_ms MCU收到完整响应末字节的本机毫秒时刻。
 * @return TIME_MAPPING_OK；非法／超时样本不进入映射，仅增加拒绝计数。
 * @note 任意间隔按32位回绕计算；无法判定的半周期及超过配置期限的样本会拒绝。
 */
TimeMappingStatus_t TimeMappingObserveExchange(TimeMapping_t *mapping,
                                               uint32_t t1_ms, uint32_t t2_ms,
                                               uint32_t t3_ms, uint32_t t4_ms);

/** @brief 复制映射状态，不推进、续期或重置同步。
 * @param[in] mapping 已初始化实例。
 * @param[in] now_local_ms 当前本机毫秒时刻，0合法。
 * @param[out] output 状态输出；失败保留原值。
 * @return TIME_MAPPING_OK或参数／未初始化错误。
 */
TimeMappingStatus_t TimeMappingGetState(const TimeMapping_t *mapping,
                                        uint32_t now_local_ms,
                                        TimeMappingState_t *output);

/** @brief 将远端毫秒时刻换算到本机时基，不改变输入或映射。
 * @param[in] mapping 已初始化并仍有效的映射。
 * @param[in] remote_ms 远端原始毫秒时刻，0合法。
 * @param[in] now_local_ms 当前本机毫秒时刻，用于检查映射是否过期。
 * @param[out] local_ms 换算结果；失败保留原值。
 * @return TIME_MAPPING_OK；不同步、过期或时间差歧义时返回错误。
 * @note 换算不会延长消息期限；业务仍须按原本机接收时间判断新鲜度。
 */
TimeMappingStatus_t TimeMappingRemoteToLocal(const TimeMapping_t *mapping,
                                             uint32_t remote_ms,
                                             uint32_t now_local_ms,
                                             uint32_t *local_ms);

#endif
