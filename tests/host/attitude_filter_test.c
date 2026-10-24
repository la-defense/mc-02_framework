/**
 * @file attitude_filter_test.c
 * @brief 验证姿态后端的单位、预测期限、实例隔离和恢复结果。
 * @author MC-02 contributors
 */
#include "attitude_filter.h"
#include "test_check.h"
#include <float.h>
#include <math.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

static AttitudeFilterConfig_t make_config(AttitudeBackend_t backend)
{
    AttitudeFilterConfig_t config = {0};
    config.backend = backend;
    config.max_prediction_ms = 20u;
    config.max_measurement_age_ms = 20u;
    config.max_dt_s = 0.02f;
    config.qekf.process_noise1 = 10.0f;
    config.qekf.process_noise2 = 0.001f;
    config.qekf.measure_noise = 1000000.0f;
    config.qekf.lambda = 1.0f;
    config.qekf.lpf = 0.0f;
    config.qekf.variance_ratio = 2.0f;
    config.fusion.sample_rate_hz = 1000.0f;
    config.fusion.gain = 0.5f;
    config.fusion.gyroscope_range_deg_s = 2000.0f;
    config.fusion.acceleration_rejection_deg = 10.0f;
    config.fusion.rejection_timeout_s = 0.02f;
    config.fusion.accel_min_m_s2 = 7.0f;
    config.fusion.accel_max_m_s2 = 12.5f;
    return config;
}

static AttitudeFilterInput_t make_input(uint32_t now_ms, uint32_t sequence)
{
    AttitudeFilterInput_t input = {0};
    input.dt_s = 0.001f;
    input.now_ms = now_ms;
    input.sample_time_ms = now_ms;
    input.sequence = sequence;
    input.accelerometer_valid = 1u;
    input.accelerometer_m_s2[2] = 9.81f;
    return input;
}

static void test_backend(AttitudeBackend_t backend)
{
    AttitudeFilter_t filter = {0};
    AttitudeFilterConfig_t config = make_config(backend);
    AttitudeFilterState_t state = {0};
    max_align_t workspace[(ATTITUDE_FILTER_STORAGE_BYTES + sizeof(max_align_t) - 1u) / sizeof(max_align_t)];
    float initial[4] = {1.0f, 0.0f, 0.0f, 0.0f};
    size_t required = 0u;
    size_t alignment = 0u;

    TEST_CHECK(AttitudeFilterGetWorkspaceRequirements(backend, &required, &alignment) == KF_OK);
    TEST_CHECK(required <= sizeof(workspace));
    TEST_CHECK(((uintptr_t)workspace % alignment) == 0u);
    TEST_CHECK(AttitudeFilterInit(&filter, &config, initial, workspace, sizeof(workspace)) == KF_OK);

    AttitudeFilterInput_t input = make_input(100u, 1u);
    TEST_CHECK(AttitudeFilterUpdate(&filter, &input, &state) == KF_OK);
    TEST_CHECK(state.filter.valid == 1u && state.filter.mode == KF_CORRECTED);
    TEST_CHECK(state.reference_version == 1u);
    TEST_CHECK(fabsf(state.quaternion[0] - 1.0f) < 0.01f);
    TEST_CHECK(fabsf(state.yaw_deg) < 0.1f);

    input = make_input(120u, 2u);
    input.accelerometer_valid = 0u;
    input.dt_s = 0.02f;
    input.gyroscope_rad_s[2] = 1.57079632679f;
    TEST_CHECK(AttitudeFilterUpdate(&filter, &input, &state) == KF_PREDICTED);
    TEST_CHECK(state.filter.valid == 1u && state.filter.mode == KF_PREDICTION);
    TEST_CHECK(state.filter.measurement_ms == 100u);
    TEST_CHECK(fabsf(state.yaw_deg - 1.8f) < 0.25f);

    input = make_input(121u, 3u);
    input.accelerometer_valid = 0u;
    TEST_CHECK(AttitudeFilterUpdate(&filter, &input, &state) == KF_PREDICTION_LIMIT);
    TEST_CHECK(state.filter.valid == 0u && state.filter.measurement_ms == 100u);

    input = make_input(122u, 4u);
    TEST_CHECK(AttitudeFilterUpdate(&filter, &input, &state) == KF_OK);
    TEST_CHECK(state.filter.valid == 1u && state.filter.mode == KF_CORRECTED);
    TEST_CHECK(state.filter.measurement_ms == 122u);
}

static void test_fusion_rejection_does_not_force_acceleration(void)
{
    AttitudeFilter_t filter = {0};
    AttitudeFilterConfig_t config = make_config(ATTITUDE_BACKEND_FUSION);
    AttitudeFilterState_t state = {0};
    max_align_t workspace[(ATTITUDE_FILTER_STORAGE_BYTES + sizeof(max_align_t) - 1u) / sizeof(max_align_t)];
    float initial[4] = {1.0f, 0.0f, 0.0f, 0.0f};

    TEST_CHECK(AttitudeFilterInit(&filter, &config, initial, workspace, sizeof(workspace)) == KF_OK);
    AttitudeFilterInput_t input = make_input(10u, 1u);
    TEST_CHECK(AttitudeFilterUpdate(&filter, &input, &state) == KF_OK);

    // 反向重力方向超过拒绝角；恢复计数不能强行把它作为校正量测。
    for (uint32_t i = 0u; i < 30u; ++i) {
        input = make_input(11u + i, 2u + i);
        input.accelerometer_m_s2[2] = -9.81f;
        KFStatus_t status = AttitudeFilterUpdate(&filter, &input, &state);
        TEST_CHECK(status == KF_PREDICTED || status == KF_PREDICTION_LIMIT);
        TEST_CHECK(state.filter.mode != KF_CORRECTED);
        TEST_CHECK(state.filter.measurement_ms == 10u);
    }

    input = make_input(50u, 32u);
    TEST_CHECK(AttitudeFilterUpdate(&filter, &input, &state) == KF_OK);
    TEST_CHECK(state.filter.mode == KF_CORRECTED && state.filter.valid == 1u);
}

static void test_failure_isolation_and_explicit_backend_reinitialisation(void)
{
    AttitudeFilter_t filter = {0};
    AttitudeFilterConfig_t config = make_config(ATTITUDE_BACKEND_FUSION);
    AttitudeFilterConfig_t invalid_config = config;
    AttitudeFilterState_t state = {0};
    max_align_t workspace[(ATTITUDE_FILTER_STORAGE_BYTES + sizeof(max_align_t) - 1u) / sizeof(max_align_t)];
    max_align_t workspace_before[(ATTITUDE_FILTER_STORAGE_BYTES + sizeof(max_align_t) - 1u) / sizeof(max_align_t)];
    AttitudeFilter_t filter_before;
    AttitudeFilter_t misaligned_filter = {0};
    float initial[4] = {1.0f, 0.0f, 0.0f, 0.0f};
    uint8_t misaligned_storage[ATTITUDE_FILTER_STORAGE_BYTES + 1u];
    size_t required = 0u;
    size_t alignment = 0u;
    size_t bytes_sentinel = 77u;
    size_t align_sentinel = 33u;

    TEST_CHECK(AttitudeFilterGetWorkspaceRequirements((AttitudeBackend_t)99, &bytes_sentinel, &align_sentinel) == KF_ERROR_CONFIG);
    TEST_CHECK(bytes_sentinel == 77u && align_sentinel == 33u);
    TEST_CHECK(AttitudeFilterGetWorkspaceRequirements(ATTITUDE_BACKEND_FUSION, &required, &alignment) == KF_OK);
    memset(misaligned_storage, 0x3c, sizeof(misaligned_storage));
    size_t offset = 0u;
    while (((uintptr_t)(misaligned_storage + offset) % alignment) == 0u) ++offset;
    TEST_CHECK(AttitudeFilterInit(&misaligned_filter, &config, initial, misaligned_storage + offset,
        sizeof(misaligned_storage) - offset) == KF_ERROR_WORKSPACE);
    TEST_CHECK(AttitudeFilterInit(&filter, &config, initial, workspace, required - 1u) == KF_ERROR_WORKSPACE);
    memset(workspace, 0x5a, sizeof(workspace));
    TEST_CHECK(AttitudeFilterInit(&filter, &config, initial, workspace, sizeof(workspace)) == KF_OK);
    TEST_CHECK(AttitudeFilterGetState(&filter, (AttitudeFilterState_t *)workspace) == KF_ERROR_WORKSPACE);
    filter_before = filter;
    memcpy(workspace_before, workspace, sizeof(workspace));
    invalid_config.max_prediction_ms = 0u;
    TEST_CHECK(AttitudeFilterInit(&filter, &invalid_config, initial, workspace, sizeof(workspace)) == KF_ERROR_CONFIG);
    TEST_CHECK(filter.initialized == filter_before.initialized);
    TEST_CHECK(filter.reference_version == filter_before.reference_version);
    TEST_CHECK(filter.backend == filter_before.backend && filter.workspace == filter_before.workspace);
    TEST_CHECK(filter.config.max_prediction_ms == filter_before.config.max_prediction_ms);
    TEST_CHECK(filter.state.reference_version == filter_before.state.reference_version);
    TEST_CHECK(memcmp(filter.state.quaternion, filter_before.state.quaternion, sizeof(filter.state.quaternion)) == 0);
    TEST_CHECK(memcmp(workspace, workspace_before, sizeof(workspace)) == 0);

    AttitudeFilterInput_t input = make_input(500u, 1u);
    TEST_CHECK(AttitudeFilterUpdate(&filter, &input, &state) == KF_OK);
    AttitudeFilterState_t last_valid = state;
    input = make_input(501u, 2u);
    input.gyroscope_rad_s[0] = FLT_MAX;
    TEST_CHECK(AttitudeFilterUpdate(&filter, &input, &state) == KF_ERROR_NUMERIC);
    TEST_CHECK(state.filter.valid == 0u);
    TEST_CHECK(memcmp(state.quaternion, last_valid.quaternion, sizeof(state.quaternion)) == 0);
    input = make_input(502u, 3u);
    TEST_CHECK(AttitudeFilterUpdate(&filter, &input, &state) == KF_ERROR_STATE);
    TEST_CHECK(state.filter.valid == 0u && state.filter.cause == KF_ERROR_NUMERIC);

    config.backend = ATTITUDE_BACKEND_QUATERNION_EKF;
    TEST_CHECK(AttitudeFilterInit(&filter, &config, initial, workspace, sizeof(workspace)) == KF_OK);
    TEST_CHECK(filter.reference_version == 2u);
    input = make_input(1u, 1u);
    TEST_CHECK(AttitudeFilterUpdate(&filter, &input, &state) == KF_OK);
    TEST_CHECK(state.filter.valid == 1u && state.reference_version == 2u);
}

static void test_instance_isolation_and_bad_input(AttitudeBackend_t backend)
{
    AttitudeFilter_t filters[2] = {0};
    AttitudeFilterConfig_t configs[2] = {make_config(backend), make_config(backend)};
    AttitudeFilterState_t states[2] = {0};
    max_align_t workspaces[2][(ATTITUDE_FILTER_STORAGE_BYTES + sizeof(max_align_t) - 1u) / sizeof(max_align_t)];
    float initial[4] = {1.0f, 0.0f, 0.0f, 0.0f};

    for (size_t i = 0u; i < 2u; ++i)
        TEST_CHECK(AttitudeFilterInit(&filters[i], &configs[i], initial, workspaces[i], sizeof(workspaces[i])) == KF_OK);

    AttitudeFilterInput_t input = make_input(200u, 1u);
    input.gyroscope_rad_s[2] = 1.0f;
    TEST_CHECK(AttitudeFilterUpdate(&filters[0], &input, &states[0]) == KF_OK);
    input = make_input(200u, 1u);
    input.gyroscope_rad_s[2] = -2.0f;
    TEST_CHECK(AttitudeFilterUpdate(&filters[1], &input, &states[1]) == KF_OK);
    TEST_CHECK(states[0].yaw_deg > 0.0f && states[1].yaw_deg < 0.0f);

    AttitudeFilterState_t previous = states[0];
    input = make_input(201u, 2u);
    input.gyroscope_rad_s[0] = NAN;
    TEST_CHECK(AttitudeFilterUpdate(&filters[0], &input, &states[0]) == KF_ERROR_NUMERIC);
    TEST_CHECK(states[0].filter.valid == 0u);
    TEST_CHECK(memcmp(states[0].quaternion, previous.quaternion, sizeof(previous.quaternion)) == 0);
    TEST_CHECK(AttitudeFilterGetState(&filters[1], &states[1]) == KF_OK);
    TEST_CHECK(states[1].filter.valid == 1u);
}

int main(void)
{
    test_backend(ATTITUDE_BACKEND_QUATERNION_EKF);
    test_backend(ATTITUDE_BACKEND_FUSION);
    test_fusion_rejection_does_not_force_acceleration();
    test_instance_isolation_and_bad_input(ATTITUDE_BACKEND_QUATERNION_EKF);
    test_instance_isolation_and_bad_input(ATTITUDE_BACKEND_FUSION);
    test_failure_isolation_and_explicit_backend_reinitialisation();
    return 0;
}
