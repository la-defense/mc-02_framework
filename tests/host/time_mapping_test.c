/** @file time_mapping_test.c
 * @brief 用固定四时刻样本验证时间偏差、回绕、失效和恢复。
 * @note 直接测试生产纯C接口；不依赖 HAL、RTOS、网络或硬件。
 */
#include "test_check.h"
#include "time_mapping.h"

#include <limits.h>
#include <string.h>

static TimeMappingConfig_t DefaultConfig(void)
{
    const TimeMappingConfig_t config = {
        TIME_MAPPING_DEFAULT_MAX_RTT_MS,
        TIME_MAPPING_DEFAULT_MAX_EXCHANGE_MS,
        TIME_MAPPING_DEFAULT_VALID_FOR_MS,
        TIME_MAPPING_DEFAULT_MAX_OFFSET_JUMP_MS,
    };
    return config;
}

static void AddStableSamples(TimeMapping_t *mapping, uint32_t local_start,
                             uint32_t remote_start)
{
    for (uint32_t i = 0u; i < TIME_MAPPING_REQUIRED_SAMPLES; ++i)
    {
        const uint32_t t1 = local_start + i * 10u;
        const uint32_t t2 = remote_start + i * 10u;
        const uint32_t t3 = t2 + 1u;
        const uint32_t t4 = t1 + 3u;
        TEST_CHECK(TimeMappingObserveExchange(mapping, t1, t2, t3, t4) == TIME_MAPPING_OK);
    }
}

static int TestOffsetAndExpiry(void)
{
    TimeMapping_t mapping = {0};
    TimeMappingState_t state = {0};
    uint32_t mapped = 0xdeadbeefu;
    TEST_CHECK(TimeMappingInit(&mapping, NULL) == TIME_MAPPING_OK);
    AddStableSamples(&mapping, 1000u, 1020u);
    TEST_CHECK(TimeMappingGetState(&mapping, 1023u, &state) == TIME_MAPPING_OK);
    TEST_CHECK(state.synchronized != 0u && state.sample_count == 3u &&
        state.remote_minus_local_ms == 19 && state.last_round_trip_ms == 2u);
    TEST_CHECK(TimeMappingRemoteToLocal(&mapping, 2000u, 1023u, &mapped) == TIME_MAPPING_OK);
    TEST_CHECK(mapped == 1981u);
    TEST_CHECK(TimeMappingGetState(&mapping, 3023u, &state) == TIME_MAPPING_OK &&
        state.synchronized != 0u && state.sample_age_ms == 2000u);
    mapped = 0xdeadbeefu;
    TEST_CHECK(TimeMappingRemoteToLocal(&mapping, 2000u, 3024u, &mapped) ==
        TIME_MAPPING_ERR_EXPIRED);
    TEST_CHECK(mapped == 0xdeadbeefu);
    TEST_CHECK(TimeMappingGetState(&mapping, 3024u, &state) == TIME_MAPPING_OK &&
        state.synchronized == 0u && state.remote_minus_local_ms == 19);
    return 0;
}

static int TestWrapAndNegativeOffset(void)
{
    TimeMapping_t mapping = {0};
    TimeMappingState_t state = {0};
    uint32_t mapped = 0u;
    TEST_CHECK(TimeMappingInit(&mapping, NULL) == TIME_MAPPING_OK);
    AddStableSamples(&mapping, 0xfffffff0u, 0x00000004u);
    TEST_CHECK(TimeMappingGetState(&mapping, 0x00000007u, &state) == TIME_MAPPING_OK &&
        state.synchronized != 0u && state.remote_minus_local_ms == 19);
    TEST_CHECK(TimeMappingRemoteToLocal(&mapping, 0x00000020u, 0x00000007u,
        &mapped) == TIME_MAPPING_OK && mapped == 0x0000000du);

    TEST_CHECK(TimeMappingInit(&mapping, NULL) == TIME_MAPPING_OK);
    AddStableSamples(&mapping, 2000u, 1980u);
    TEST_CHECK(TimeMappingGetState(&mapping, 2023u, &state) == TIME_MAPPING_OK &&
        state.synchronized != 0u && state.remote_minus_local_ms == -21);
    TEST_CHECK(TimeMappingRemoteToLocal(&mapping, 1000u, 2023u, &mapped) ==
        TIME_MAPPING_OK && mapped == 1021u);
    return 0;
}

static int TestRejectAndReacquire(void)
{
    TimeMapping_t mapping = {0};
    TimeMappingState_t state = {0};
    const TimeMappingConfig_t config = DefaultConfig();
    TEST_CHECK(TimeMappingInit(&mapping, &config) == TIME_MAPPING_OK);
    AddStableSamples(&mapping, 1000u, 1020u);
    TEST_CHECK(TimeMappingObserveExchange(&mapping, 1030u, 1050u, 1060u, 1039u) ==
        TIME_MAPPING_ERR_ROUND_TRIP);
    TEST_CHECK(TimeMappingGetState(&mapping, 1039u, &state) == TIME_MAPPING_OK &&
        state.synchronized != 0u && state.remote_minus_local_ms == 19u);
    TEST_CHECK(TimeMappingObserveExchange(&mapping, 1030u, 1050u, 1051u, 1090u) ==
        TIME_MAPPING_ERR_ROUND_TRIP);
    TEST_CHECK(TimeMappingObserveExchange(&mapping, 1030u, 1050u, 1051u,
        1030u + 0x80000000u) == TIME_MAPPING_ERR_TIME);

    // 跳变样本作为新一轮的首个候选，后两份稳定样本才恢复转换。
    TEST_CHECK(TimeMappingObserveExchange(&mapping, 4000u, 4100u, 4101u, 4003u) ==
        TIME_MAPPING_OK);
    TEST_CHECK(TimeMappingGetState(&mapping, 4003u, &state) == TIME_MAPPING_OK &&
        state.synchronized == 0u && state.sample_count == 1u);
    TEST_CHECK(TimeMappingObserveExchange(&mapping, 4010u, 4110u, 4111u, 4013u) ==
        TIME_MAPPING_OK);
    TEST_CHECK(TimeMappingObserveExchange(&mapping, 4020u, 4120u, 4121u, 4023u) ==
        TIME_MAPPING_OK);
    TEST_CHECK(TimeMappingGetState(&mapping, 4023u, &state) == TIME_MAPPING_OK &&
        state.synchronized != 0u && state.remote_minus_local_ms == 99);
    return 0;
}

static int TestInvalidConfigurationPreservesInstance(void)
{
    TimeMapping_t mapping = {0};
    TimeMapping_t previous;
    TimeMappingConfig_t invalid = DefaultConfig();
    TEST_CHECK(TimeMappingInit(&mapping, NULL) == TIME_MAPPING_OK);
    previous = mapping;
    invalid.max_round_trip_ms = 0u;
    TEST_CHECK(TimeMappingInit(&mapping, &invalid) == TIME_MAPPING_ERR_CONFIG);
    TEST_CHECK(memcmp(&mapping, &previous, sizeof(mapping)) == 0);
    TEST_CHECK(TimeMappingInit(NULL, NULL) == TIME_MAPPING_ERR_ARGUMENT);
    TEST_CHECK(TimeMappingInit(&mapping, &(TimeMappingConfig_t){
        50u, (uint32_t)INT32_MAX + 1u, 2000u, 50u}) == TIME_MAPPING_ERR_CONFIG);
    return 0;
}

static int TestRoundTripAndExchangeBoundaries(void)
{
    TimeMapping_t mapping = {0};
    TimeMappingState_t state = {0};
    TEST_CHECK(TimeMappingInit(&mapping, NULL) == TIME_MAPPING_OK);

    // 总交换100ms、PC处理60ms，网络往返40ms，截止边界内可接受。
    TEST_CHECK(TimeMappingObserveExchange(&mapping, 1000u, 1020u,
        1080u, 1100u) == TIME_MAPPING_OK);
    TEST_CHECK(TimeMappingGetState(&mapping, 1100u, &state) == TIME_MAPPING_OK &&
        state.sample_count == 1u && state.last_round_trip_ms == 40u);

    // 最大往返50ms边界允许；多1ms拒绝且不改偏差和样本窗。
    TEST_CHECK(TimeMappingObserveExchange(&mapping, 2000u, 2050u,
        2051u, 2051u) == TIME_MAPPING_OK);
    const int32_t previous_offset = mapping.remote_minus_local_ms;
    TEST_CHECK(TimeMappingObserveExchange(&mapping, 3000u, 3051u,
        3052u, 3052u) == TIME_MAPPING_ERR_ROUND_TRIP);
    TEST_CHECK(mapping.remote_minus_local_ms == previous_offset &&
        mapping.sample_count == 2u);

    // 总交换时间超过100ms、逆序以及半周期歧义均不能进入样本窗。
    TEST_CHECK(TimeMappingObserveExchange(&mapping, 4000u, 4001u,
        4002u, 4101u) == TIME_MAPPING_ERR_TIME);
    TEST_CHECK(TimeMappingObserveExchange(&mapping, 5000u, 5001u,
        5002u, 4999u) == TIME_MAPPING_ERR_TIME);
    TEST_CHECK(TimeMappingObserveExchange(&mapping, 6000u, 6001u,
        6002u, 6000u + 0x80000000u) == TIME_MAPPING_ERR_TIME);
    TEST_CHECK(mapping.sample_count == 2u &&
        mapping.remote_minus_local_ms == previous_offset);
    return 0;
}

static int TestUninitializedOutputsRemainUnchanged(void)
{
    TimeMapping_t mapping = {0};
    TimeMappingState_t state;
    memset(&state, 0xa5, sizeof(state));
    const TimeMappingState_t original_state = state;
    uint32_t mapped = 0x12345678u;
    TEST_CHECK(TimeMappingGetState(&mapping, 10u, &state) ==
        TIME_MAPPING_ERR_UNINITIALIZED);
    TEST_CHECK(memcmp(&state, &original_state, sizeof(state)) == 0);
    TEST_CHECK(TimeMappingRemoteToLocal(&mapping, 1u, 10u, &mapped) ==
        TIME_MAPPING_ERR_UNINITIALIZED && mapped == 0x12345678u);
    return 0;
}

int main(void)
{
    TEST_CHECK(TestOffsetAndExpiry() == 0);
    TEST_CHECK(TestWrapAndNegativeOffset() == 0);
    TEST_CHECK(TestRejectAndReacquire() == 0);
    TEST_CHECK(TestInvalidConfigurationPreservesInstance() == 0);
    TEST_CHECK(TestRoundTripAndExchangeBoundaries() == 0);
    TEST_CHECK(TestUninitializedOutputsRemainUnchanged() == 0);
    return 0;
}
