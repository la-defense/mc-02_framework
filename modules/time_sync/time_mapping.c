/** @file time_mapping.c
 * @brief 远端与本机毫秒时钟映射的纯C实现。
 * @note 本文件不读取硬件时钟；调用者传入四个测量时刻并串行访问实例。
 */
#include "time_mapping.h"

#include <limits.h>
#include <string.h>

#define TIME_MAPPING_TICK_PERIOD 0x100000000LL

static uint8_t IsDurationValid(uint32_t duration_ms)
{
    return (uint8_t)(duration_ms != 0u && duration_ms <= (uint32_t)INT32_MAX);
}

static uint8_t ConfigIsValid(const TimeMappingConfig_t *config)
{
    return (uint8_t)(IsDurationValid(config->max_round_trip_ms) &&
        IsDurationValid(config->max_exchange_ms) &&
        IsDurationValid(config->valid_for_ms) &&
        config->max_round_trip_ms <= config->max_exchange_ms &&
        config->max_offset_jump_ms <= (uint32_t)INT32_MAX);
}

/* 选择模32位的最近有符号差值，并拒绝正好半个计数周期的歧义值。 */
static uint8_t TimeDifference(uint32_t later_ms, uint32_t earlier_ms,
                              int64_t *difference_ms)
{
    const uint32_t raw = later_ms - earlier_ms;
    int64_t candidate = raw;
    if (raw > (uint32_t)INT32_MAX)
        candidate = (int64_t)raw - TIME_MAPPING_TICK_PERIOD;
    if (candidate == INT32_MIN)
        return 0u;
    *difference_ms = candidate;
    return 1u;
}

static void CountSaturating(uint32_t *count)
{
    if (*count != UINT32_MAX)
        ++*count;
}

static int32_t MedianOfThree(const int32_t values[TIME_MAPPING_REQUIRED_SAMPLES])
{
    int32_t first = values[0];
    int32_t second = values[1];
    int32_t third = values[2];
    if (first > second) { const int32_t swap = first; first = second; second = swap; }
    if (second > third) { const int32_t swap = second; second = third; third = swap; }
    if (first > second) { const int32_t swap = first; first = second; second = swap; }
    return second;
}

static void ClearSamples(TimeMapping_t *mapping)
{
    memset(mapping->offset_samples_ms, 0, sizeof(mapping->offset_samples_ms));
    mapping->sample_count = 0u;
    mapping->next_sample = 0u;
}

TimeMappingStatus_t TimeMappingInit(TimeMapping_t *mapping,
                                    const TimeMappingConfig_t *config)
{
    if (mapping == NULL)
        return TIME_MAPPING_ERR_ARGUMENT;
    const TimeMappingConfig_t candidate_config = config != NULL ? *config :
        (TimeMappingConfig_t){
            .max_round_trip_ms = TIME_MAPPING_DEFAULT_MAX_RTT_MS,
            .max_exchange_ms = TIME_MAPPING_DEFAULT_MAX_EXCHANGE_MS,
            .valid_for_ms = TIME_MAPPING_DEFAULT_VALID_FOR_MS,
            .max_offset_jump_ms = TIME_MAPPING_DEFAULT_MAX_OFFSET_JUMP_MS,
        };
    if (ConfigIsValid(&candidate_config) == 0u)
        return TIME_MAPPING_ERR_CONFIG;

    // 先检查完整配置，再一次替换实例，避免错误配置破坏已有映射。
    TimeMapping_t candidate = {0};
    candidate.config = candidate_config;
    candidate.initialized = 1u;
    *mapping = candidate;
    return TIME_MAPPING_OK;
}

TimeMappingStatus_t TimeMappingReset(TimeMapping_t *mapping)
{
    if (mapping == NULL)
        return TIME_MAPPING_ERR_ARGUMENT;
    if (mapping->initialized == 0u)
        return TIME_MAPPING_ERR_UNINITIALIZED;
    const TimeMappingConfig_t config = mapping->config;
    TimeMapping_t candidate = {0};
    candidate.config = config;
    candidate.initialized = 1u;
    *mapping = candidate;
    return TIME_MAPPING_OK;
}

TimeMappingStatus_t TimeMappingObserveExchange(TimeMapping_t *mapping,
                                               uint32_t t1_ms, uint32_t t2_ms,
                                               uint32_t t3_ms, uint32_t t4_ms)
{
    if (mapping == NULL)
        return TIME_MAPPING_ERR_ARGUMENT;
    if (mapping->initialized == 0u)
        return TIME_MAPPING_ERR_UNINITIALIZED;

    int64_t local_elapsed_ms = 0;
    int64_t remote_elapsed_ms = 0;
    int64_t outbound_delta_ms = 0;
    int64_t return_delta_ms = 0;
    if (TimeDifference(t4_ms, t1_ms, &local_elapsed_ms) == 0u ||
        TimeDifference(t3_ms, t2_ms, &remote_elapsed_ms) == 0u ||
        TimeDifference(t2_ms, t1_ms, &outbound_delta_ms) == 0u ||
        TimeDifference(t3_ms, t4_ms, &return_delta_ms) == 0u ||
        local_elapsed_ms < 0 || remote_elapsed_ms < 0 ||
        local_elapsed_ms > mapping->config.max_exchange_ms ||
        remote_elapsed_ms > mapping->config.max_exchange_ms)
    {
        CountSaturating(&mapping->rejected_samples);
        return TIME_MAPPING_ERR_TIME;
    }

    const int64_t round_trip_ms = local_elapsed_ms - remote_elapsed_ms;
    if (round_trip_ms < 0 || round_trip_ms > mapping->config.max_round_trip_ms)
    {
        CountSaturating(&mapping->rejected_samples);
        return TIME_MAPPING_ERR_ROUND_TRIP;
    }

    // 平均两条方向偏差，半毫秒时远离零取整，避免依赖浮点计算。
    const int64_t offset_sum_ms = outbound_delta_ms + return_delta_ms;
    const int64_t offset_candidate_ms = offset_sum_ms >= 0 ?
        (offset_sum_ms + 1) / 2 : (offset_sum_ms - 1) / 2;
    if (offset_candidate_ms < INT32_MIN + 1LL || offset_candidate_ms > INT32_MAX)
    {
        CountSaturating(&mapping->rejected_samples);
        return TIME_MAPPING_ERR_TIME;
    }

    if (mapping->sample_count != 0u)
    {
        int64_t sample_age_ms = 0;
        if (TimeDifference(t4_ms, mapping->last_local_sample_ms, &sample_age_ms) == 0u ||
            sample_age_ms < 0)
        {
            CountSaturating(&mapping->rejected_samples);
            return TIME_MAPPING_ERR_TIME;
        }
        // 太久没有交换样本时，旧样本不能参与新一轮映射。
        if ((uint64_t)sample_age_ms > mapping->config.valid_for_ms)
            ClearSamples(mapping);
    }

    if (mapping->sample_count == TIME_MAPPING_REQUIRED_SAMPLES)
    {
        const int64_t jump_ms = offset_candidate_ms - mapping->remote_minus_local_ms;
        const uint64_t jump_magnitude_ms = (uint64_t)(jump_ms < 0 ? -jump_ms : jump_ms);
        if (jump_magnitude_ms > mapping->config.max_offset_jump_ms)
        {
            // 时钟发生明显跳变时重新计满三份稳定样本，再开放换算。
            ClearSamples(mapping);
        }
    }

    mapping->offset_samples_ms[mapping->next_sample] = (int32_t)offset_candidate_ms;
    mapping->next_sample = (uint8_t)((mapping->next_sample + 1u) %
                                     TIME_MAPPING_REQUIRED_SAMPLES);
    if (mapping->sample_count < TIME_MAPPING_REQUIRED_SAMPLES)
        ++mapping->sample_count;
    mapping->remote_minus_local_ms = mapping->sample_count == TIME_MAPPING_REQUIRED_SAMPLES ?
        MedianOfThree(mapping->offset_samples_ms) : (int32_t)offset_candidate_ms;
    mapping->last_local_sample_ms = t4_ms;
    mapping->last_remote_sample_ms = t3_ms;
    mapping->last_round_trip_ms = (uint32_t)round_trip_ms;
    CountSaturating(&mapping->accepted_samples);
    return TIME_MAPPING_OK;
}

TimeMappingStatus_t TimeMappingGetState(const TimeMapping_t *mapping,
                                        uint32_t now_local_ms,
                                        TimeMappingState_t *output)
{
    if (mapping == NULL || output == NULL)
        return TIME_MAPPING_ERR_ARGUMENT;
    if (mapping->initialized == 0u)
        return TIME_MAPPING_ERR_UNINITIALIZED;

    TimeMappingState_t candidate = {0};
    candidate.remote_minus_local_ms = mapping->remote_minus_local_ms;
    candidate.last_local_sample_ms = mapping->last_local_sample_ms;
    candidate.last_round_trip_ms = mapping->last_round_trip_ms;
    candidate.accepted_samples = mapping->accepted_samples;
    candidate.rejected_samples = mapping->rejected_samples;
    candidate.sample_count = mapping->sample_count;
    candidate.has_sample = mapping->sample_count != 0u ? 1u : 0u;
    if (candidate.has_sample != 0u)
    {
        int64_t age_ms = 0;
        if (TimeDifference(now_local_ms, mapping->last_local_sample_ms, &age_ms) == 0u ||
            age_ms < 0)
            return TIME_MAPPING_ERR_TIME;
        candidate.sample_age_ms = (uint32_t)age_ms;
        candidate.synchronized = (uint8_t)(mapping->sample_count == TIME_MAPPING_REQUIRED_SAMPLES &&
            (uint64_t)age_ms <= mapping->config.valid_for_ms);
    }
    *output = candidate;
    return TIME_MAPPING_OK;
}

TimeMappingStatus_t TimeMappingRemoteToLocal(const TimeMapping_t *mapping,
                                             uint32_t remote_ms,
                                             uint32_t now_local_ms,
                                             uint32_t *local_ms)
{
    if (mapping == NULL || local_ms == NULL)
        return TIME_MAPPING_ERR_ARGUMENT;
    TimeMappingState_t state = {0};
    const TimeMappingStatus_t status = TimeMappingGetState(mapping, now_local_ms, &state);
    if (status != TIME_MAPPING_OK)
        return status;
    if (state.sample_count < TIME_MAPPING_REQUIRED_SAMPLES)
        return TIME_MAPPING_ERR_NOT_SYNCHRONIZED;
    if (state.synchronized == 0u)
        return TIME_MAPPING_ERR_EXPIRED;

    int64_t source_distance_ms = 0;
    if (TimeDifference(remote_ms, mapping->last_remote_sample_ms, &source_distance_ms) == 0u)
        return TIME_MAPPING_ERR_TIME;
    // 无符号运算按32位回绕完成减偏差，输出单位仍是毫秒。
    *local_ms = remote_ms - (uint32_t)mapping->remote_minus_local_ms;
    return TIME_MAPPING_OK;
}
