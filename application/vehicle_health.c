/**
 * @file vehicle_health.c
 * @brief 以固定周期采集并原子发布整车只读健康快照。
 * @note 生产数据来自 vehicle_health_port_stm32.c；本文件不读取硬件或分配内存。
 */
#include "vehicle_health.h"
#include "vehicle_health_source.h"
#include "config/build_selection.h"
#if defined(MC02_CONFIG_GENERATED)
#include "config/vehicle_selection.h"
#endif
#include <stddef.h>
#include <math.h>
#include <string.h>

static VehicleHealthSnapshot_t published_snapshot;
static uint32_t last_attempt_ms;
static uint8_t initialized;
static uint8_t has_attempt;
static uint8_t has_snapshot;

static void AddModule(uint32_t *modules, VehicleHealthModule_t module)
{
    *modules |= VEHICLE_HEALTH_MODULE_BIT(module);
}

VehicleHealthStatus_t VehicleHealthGetBuildProfile(VehicleHealthBuildProfile_t *output)
{
    if (output == NULL)
        return VEHICLE_HEALTH_ERROR_ARGUMENT;

    VehicleHealthBuildProfile_t candidate = {0};
#if defined(MC02_CONFIG_GENERATED)
    if (strcmp(MC02_BUILD_ROLE_NAME, "bench") == 0)
        candidate.role = VEHICLE_HEALTH_ROLE_BENCH;
    else if (strcmp(MC02_BUILD_ROLE_NAME, "chassis") == 0)
        candidate.role = VEHICLE_HEALTH_ROLE_CHASSIS;
    else if (strcmp(MC02_BUILD_ROLE_NAME, "gimbal") == 0)
        candidate.role = VEHICLE_HEALTH_ROLE_GIMBAL;
    else
        candidate.role = VEHICLE_HEALTH_ROLE_SINGLE;
    AddModule(&candidate.enabled_modules, VEHICLE_HEALTH_MODULE_INIT);
    AddModule(&candidate.enabled_modules, VEHICLE_HEALTH_MODULE_SAFETY);
    AddModule(&candidate.enabled_modules, VEHICLE_HEALTH_MODULE_TASK_MONITOR);
#define ADD_IF_ENABLED(feature_, module_) \
    do { if (feature_) AddModule(&candidate.enabled_modules, module_); } while (0)
    ADD_IF_ENABLED(MC02_FEATURE_IMU, VEHICLE_HEALTH_MODULE_IMU);
    ADD_IF_ENABLED(MC02_FEATURE_REMOTE, VEHICLE_HEALTH_MODULE_REMOTE);
    ADD_IF_ENABLED(MC02_FEATURE_VISION, VEHICLE_HEALTH_MODULE_VISION);
    ADD_IF_ENABLED(MC02_FEATURE_REFEREE, VEHICLE_HEALTH_MODULE_REFEREE);
    ADD_IF_ENABLED(MC02_FEATURE_HEATER, VEHICLE_HEALTH_MODULE_HEATER);
    ADD_IF_ENABLED(MC02_FEATURE_LOG, VEHICLE_HEALTH_MODULE_LOG);
    ADD_IF_ENABLED(MC02_FEATURE_CHASSIS, VEHICLE_HEALTH_MODULE_CHASSIS);
    ADD_IF_ENABLED(MC02_FEATURE_GIMBAL, VEHICLE_HEALTH_MODULE_GIMBAL);
    ADD_IF_ENABLED(MC02_FEATURE_SHOOTER, VEHICLE_HEALTH_MODULE_SHOOTER);
    ADD_IF_ENABLED(MC02_FEATURE_COMMAND, VEHICLE_HEALTH_MODULE_COMMAND);
    ADD_IF_ENABLED(MC02_FEATURE_COMMUNICATION, VEHICLE_HEALTH_MODULE_COMMUNICATION);
    ADD_IF_ENABLED(MC02_FEATURE_ALARM, VEHICLE_HEALTH_MODULE_ALARM);
    ADD_IF_ENABLED(MC02_FEATURE_LCD, VEHICLE_HEALTH_MODULE_LCD);
    ADD_IF_ENABLED(MC02_FEATURE_UI, VEHICLE_HEALTH_MODULE_UI);
    ADD_IF_ENABLED(MC02_FEATURE_SUPERCAP, VEHICLE_HEALTH_MODULE_SUPERCAP);
#undef ADD_IF_ENABLED
    if (candidate.role != VEHICLE_HEALTH_ROLE_BENCH)
    {
        AddModule(&candidate.enabled_modules, VEHICLE_HEALTH_MODULE_CAN1);
        AddModule(&candidate.enabled_modules, VEHICLE_HEALTH_MODULE_CAN2);
        AddModule(&candidate.enabled_modules, VEHICLE_HEALTH_MODULE_CAN3);
    }
#else
#if defined(MC02_PROFILE_BENCH_SAFE)
    candidate.role = VEHICLE_HEALTH_ROLE_BENCH;
#elif defined(CHASSIS_BOARD)
    candidate.role = VEHICLE_HEALTH_ROLE_CHASSIS;
#elif defined(GIMBAL_BOARD)
    candidate.role = VEHICLE_HEALTH_ROLE_GIMBAL;
#else
    candidate.role = VEHICLE_HEALTH_ROLE_SINGLE;
#endif
    AddModule(&candidate.enabled_modules, VEHICLE_HEALTH_MODULE_INIT);
    AddModule(&candidate.enabled_modules, VEHICLE_HEALTH_MODULE_SAFETY);
    AddModule(&candidate.enabled_modules, VEHICLE_HEALTH_MODULE_TASK_MONITOR);
    AddModule(&candidate.enabled_modules, VEHICLE_HEALTH_MODULE_IMU);
    AddModule(&candidate.enabled_modules, VEHICLE_HEALTH_MODULE_HEATER);
    AddModule(&candidate.enabled_modules, VEHICLE_HEALTH_MODULE_LOG);

#if defined(MC02_PROFILE_ROBOT)
    AddModule(&candidate.enabled_modules, VEHICLE_HEALTH_MODULE_CAN1);
    AddModule(&candidate.enabled_modules, VEHICLE_HEALTH_MODULE_CAN2);
    AddModule(&candidate.enabled_modules, VEHICLE_HEALTH_MODULE_CAN3);
#if defined(ONE_BOARD) || defined(CHASSIS_BOARD)
    AddModule(&candidate.enabled_modules, VEHICLE_HEALTH_MODULE_CHASSIS);
    AddModule(&candidate.enabled_modules, VEHICLE_HEALTH_MODULE_REFEREE);
#endif
#if defined(ONE_BOARD) || defined(GIMBAL_BOARD)
    AddModule(&candidate.enabled_modules, VEHICLE_HEALTH_MODULE_GIMBAL);
    AddModule(&candidate.enabled_modules, VEHICLE_HEALTH_MODULE_SHOOTER);
    AddModule(&candidate.enabled_modules, VEHICLE_HEALTH_MODULE_REMOTE);
    AddModule(&candidate.enabled_modules, VEHICLE_HEALTH_MODULE_VISION);
#endif
#endif
#if defined(MC02_PROFILE_BENCH_SAFE)
    AddModule(&candidate.enabled_modules, VEHICLE_HEALTH_MODULE_VISION);
#endif
#if defined(MC02_BENCH_RC_DIAGNOSTICS) && MC02_BENCH_RC_DIAGNOSTICS
    AddModule(&candidate.enabled_modules, VEHICLE_HEALTH_MODULE_REMOTE);
#endif
#if defined(MC02_BENCH_REFEREE_DIAGNOSTICS) && MC02_BENCH_REFEREE_DIAGNOSTICS
    AddModule(&candidate.enabled_modules, VEHICLE_HEALTH_MODULE_REFEREE);
#endif
#if (defined(VISION_USE_VCP) && (defined(MC02_PROFILE_BENCH_SAFE) || defined(ONE_BOARD) || defined(GIMBAL_BOARD))) || \
    (defined(MC02_BENCH_USB_DIAGNOSTICS) && MC02_BENCH_USB_DIAGNOSTICS)
    AddModule(&candidate.enabled_modules, VEHICLE_HEALTH_MODULE_USB);
#endif
#endif
    *output = candidate;
    return VEHICLE_HEALTH_OK;
}

static uint32_t NextSequence(uint32_t sequence)
{
    ++sequence;
    return sequence == 0u ? 1u : sequence;
}

VehicleHealthStatus_t VehicleHealthInit(void)
{
    if (VehicleHealthSourceInIsr() != 0u)
        return VEHICLE_HEALTH_ERROR_CONTEXT;

    uint32_t previous = VehicleHealthSourceEnterCritical();
    if (initialized != 0u)
    {
        VehicleHealthSourceExitCritical(previous);
        return VEHICLE_HEALTH_ALREADY_INITIALIZED;
    }
    published_snapshot = (VehicleHealthSnapshot_t){0};
    last_attempt_ms = 0u;
    has_attempt = 0u;
    has_snapshot = 0u;
    initialized = 1u;
    VehicleHealthSourceExitCritical(previous);
    return VEHICLE_HEALTH_OK;
}

VehicleHealthStatus_t VehicleHealthProcess(void)
{
    if (VehicleHealthSourceInIsr() != 0u)
        return VEHICLE_HEALTH_ERROR_CONTEXT;

    uint32_t now_ms = VehicleHealthSourceNowMs();
    uint32_t previous = VehicleHealthSourceEnterCritical();
    if (initialized == 0u)
    {
        VehicleHealthSourceExitCritical(previous);
        return VEHICLE_HEALTH_ERROR_NOT_INITIALIZED;
    }
    if ((has_attempt != 0u) && ((uint32_t)(now_ms - last_attempt_ms) < VEHICLE_HEALTH_UPDATE_PERIOD_MS))
    {
        VehicleHealthSourceExitCritical(previous);
        return VEHICLE_HEALTH_NOT_DUE;
    }
    last_attempt_ms = now_ms;
    has_attempt = 1u;
    VehicleHealthSourceExitCritical(previous);

    VehicleHealthSourceSnapshot_t source;
    if (VehicleHealthSourceCapture(&source) == 0u)
        return VEHICLE_HEALTH_ERROR_CAPTURE;

    VehicleHealthSnapshot_t candidate = {0};
    candidate.captured_ms = VehicleHealthSourceNowMs();
    candidate.role = source.role;
    candidate.enabled_modules = source.enabled_modules;
    candidate.available_modules = source.available_modules & source.enabled_modules;
    candidate.degraded_modules = source.degraded_modules |
                                 (source.enabled_modules & ~candidate.available_modules);
    candidate.init_failed = source.init_failed;
    candidate.init_stage = source.init_stage;
    candidate.init_module = source.init_module;
    candidate.init_reason = source.init_reason;
    candidate.init_resource_id = source.init_resource_id;
    candidate.init_failure_count = source.init_failure_count;
    candidate.init_heartbeat = source.init_heartbeat;
    candidate.safety_state = source.safety_state;
    candidate.safety_fault_flags = source.safety_fault_flags;
    candidate.safety_estop_latched = source.safety_estop_latched;
    candidate.task_monitor_fault_mask = source.task_monitor_fault_mask;
    candidate.imu_valid = source.imu_valid;
    candidate.calibration_valid = source.calibration_valid;
    candidate.imu_sample_ms = source.imu_sample_ms;
    candidate.imu_sequence = source.imu_sequence;
    candidate.remote_online = source.remote_online;
    candidate.remote_pc_valid = source.remote_pc_valid;
    candidate.remote_last_valid_ms = source.remote_last_valid_ms;
    candidate.vision_online = source.vision_online;
    candidate.vision_has_valid_frame = source.vision_has_valid_frame;
    candidate.vision_last_rx_ms = source.vision_last_rx_ms;
    candidate.vision_rx_count = source.vision_rx_count;
    candidate.referee_online = source.referee_online;
    candidate.referee_fresh_command_mask = source.referee_fresh_command_mask;
    candidate.referee_event_command_mask = source.referee_event_command_mask;
    candidate.referee_link_age_ms = source.referee_link_age_ms;
    candidate.referee_latest_source_ms = source.referee_latest_source_ms;
    candidate.can_active_mask = source.can_active_mask;
    candidate.can_busoff_mask = source.can_busoff_mask;
    for (uint32_t index = 0u; index < VEHICLE_HEALTH_CAN_BUS_COUNT; ++index)
        candidate.can_busoff_count[index] = source.can_busoff_count[index];
    candidate.usb_phase = source.usb_phase;
    candidate.usb_restart_quota_known = source.usb_restart_quota_known;
    candidate.usb_restart_limit = source.usb_restart_limit;
    candidate.usb_restart_attempts = source.usb_restart_attempts;
    candidate.usb_restart_remaining = source.usb_restart_remaining;
    candidate.system_reset_quota_known = 0u;
    candidate.system_reset_quota_remaining = VEHICLE_HEALTH_UNKNOWN_QUOTA;
    candidate.heater_initialized = source.heater_initialized;
    candidate.heater_compiled_enabled = source.heater_compiled_enabled;
    candidate.heater_sensor_valid = source.heater_sensor_valid;
    candidate.heater_pwm_active = source.heater_pwm_active;
    candidate.heater_fault = source.heater_fault;
    candidate.heater_overtemp = source.heater_overtemp;
    candidate.heater_timeout_fault = source.heater_timeout_fault;
    candidate.thermal_stable_known = 0u;
    candidate.thermal_stable = 0u;
    candidate.heater_duty = source.heater_duty;
    candidate.temperature_c = source.temperature_c;

    previous = VehicleHealthSourceEnterCritical();
    candidate.sequence = NextSequence(published_snapshot.sequence);
    published_snapshot = candidate;
    has_snapshot = 1u;
    VehicleHealthSourceExitCritical(previous);
    return VEHICLE_HEALTH_OK;
}

VehicleHealthStatus_t VehicleHealthGetSnapshot(VehicleHealthSnapshot_t *output)
{
    if (output == NULL)
        return VEHICLE_HEALTH_ERROR_ARGUMENT;
    if (VehicleHealthSourceInIsr() != 0u)
        return VEHICLE_HEALTH_ERROR_CONTEXT;

    uint32_t previous = VehicleHealthSourceEnterCritical();
    if (initialized == 0u)
    {
        VehicleHealthSourceExitCritical(previous);
        return VEHICLE_HEALTH_ERROR_NOT_INITIALIZED;
    }
    if (has_snapshot == 0u)
    {
        VehicleHealthSourceExitCritical(previous);
        return VEHICLE_HEALTH_ERROR_NOT_READY;
    }
    *output = published_snapshot;
    VehicleHealthSourceExitCritical(previous);
    return VEHICLE_HEALTH_OK;
}

VehicleHealthStatus_t VehicleHealthGetInitEvents(InitDiagnosticEventSnapshot_t *output)
{
    if (output == NULL)
        return VEHICLE_HEALTH_ERROR_ARGUMENT;
    if (VehicleHealthSourceInIsr() != 0u)
        return VEHICLE_HEALTH_ERROR_CONTEXT;
    return InitDiagnosticGetEvents(output) != 0u ? VEHICLE_HEALTH_OK : VEHICLE_HEALTH_ERROR_CAPTURE;
}

static void PutU16(uint8_t *output, uint32_t *offset, uint16_t value)
{
    output[(*offset)++] = (uint8_t)value;
    output[(*offset)++] = (uint8_t)(value >> 8u);
}

static void PutU32(uint8_t *output, uint32_t *offset, uint32_t value)
{
    output[(*offset)++] = (uint8_t)value;
    output[(*offset)++] = (uint8_t)(value >> 8u);
    output[(*offset)++] = (uint8_t)(value >> 16u);
    output[(*offset)++] = (uint8_t)(value >> 24u);
}

VehicleHealthStatus_t VehicleHealthEncodeDiagnostic(const VehicleHealthSnapshot_t *snapshot,
                                                     uint8_t *output,
                                                     uint32_t capacity,
                                                     uint32_t *written)
{
    if ((snapshot == NULL) || (output == NULL) || (written == NULL))
        return VEHICLE_HEALTH_ERROR_ARGUMENT;
    if (capacity < VEHICLE_HEALTH_DIAGNOSTIC_DATA_SIZE)
        return VEHICLE_HEALTH_ERROR_ARGUMENT;

    uint8_t candidate[VEHICLE_HEALTH_DIAGNOSTIC_DATA_SIZE];
    uint32_t offset = 0u;
    candidate[offset++] = VEHICLE_HEALTH_DIAGNOSTIC_SCHEMA_VERSION;
    PutU32(candidate, &offset, snapshot->sequence);
    PutU32(candidate, &offset, snapshot->captured_ms);
    candidate[offset++] = (uint8_t)snapshot->role;
    PutU32(candidate, &offset, snapshot->enabled_modules);
    PutU32(candidate, &offset, snapshot->available_modules);
    PutU32(candidate, &offset, snapshot->degraded_modules);
    candidate[offset++] = snapshot->init_failed;
    candidate[offset++] = snapshot->init_stage;
    candidate[offset++] = snapshot->init_module;
    candidate[offset++] = snapshot->init_reason;
    PutU16(candidate, &offset, snapshot->init_resource_id);
    PutU32(candidate, &offset, snapshot->init_failure_count);
    PutU32(candidate, &offset, snapshot->init_heartbeat);
    candidate[offset++] = snapshot->safety_state;
    PutU32(candidate, &offset, snapshot->safety_fault_flags);
    candidate[offset++] = snapshot->safety_estop_latched;
    candidate[offset++] = snapshot->task_monitor_fault_mask;
    candidate[offset++] = snapshot->imu_valid;
    candidate[offset++] = snapshot->calibration_valid;
    PutU32(candidate, &offset, snapshot->imu_sample_ms);
    PutU32(candidate, &offset, snapshot->imu_sequence);
    candidate[offset++] = snapshot->remote_online;
    candidate[offset++] = snapshot->remote_pc_valid;
    PutU32(candidate, &offset, snapshot->remote_last_valid_ms);
    candidate[offset++] = snapshot->vision_online;
    candidate[offset++] = snapshot->vision_has_valid_frame;
    PutU32(candidate, &offset, snapshot->vision_last_rx_ms);
    PutU32(candidate, &offset, snapshot->vision_rx_count);
    candidate[offset++] = snapshot->referee_online;
    PutU32(candidate, &offset, snapshot->referee_fresh_command_mask);
    PutU32(candidate, &offset, snapshot->referee_event_command_mask);
    PutU32(candidate, &offset, snapshot->referee_link_age_ms);
    PutU32(candidate, &offset, snapshot->referee_latest_source_ms);
    candidate[offset++] = snapshot->can_active_mask;
    candidate[offset++] = snapshot->can_busoff_mask;
    for (uint32_t index = 0u; index < VEHICLE_HEALTH_CAN_BUS_COUNT; ++index)
    {
        uint32_t count = snapshot->can_busoff_count[index];
        PutU16(candidate, &offset, (uint16_t)(count > UINT16_MAX ? UINT16_MAX : count));
    }
    candidate[offset++] = snapshot->usb_phase;
    candidate[offset++] = snapshot->usb_restart_quota_known;
    candidate[offset++] = snapshot->usb_restart_limit;
    candidate[offset++] = snapshot->usb_restart_attempts;
    candidate[offset++] = snapshot->usb_restart_remaining;
    candidate[offset++] = snapshot->system_reset_quota_known;
    PutU32(candidate, &offset, snapshot->system_reset_quota_remaining);
    candidate[offset++] = snapshot->heater_initialized;
    candidate[offset++] = snapshot->heater_compiled_enabled;
    candidate[offset++] = snapshot->heater_sensor_valid;
    candidate[offset++] = snapshot->heater_pwm_active;
    candidate[offset++] = snapshot->heater_fault;
    candidate[offset++] = snapshot->heater_overtemp;
    candidate[offset++] = snapshot->heater_timeout_fault;
    candidate[offset++] = snapshot->thermal_stable_known;
    candidate[offset++] = snapshot->thermal_stable;
    PutU16(candidate, &offset, snapshot->heater_duty);

    // 温度按0.01°C编码；无效、非有限或超出有符号16位范围时使用INT16_MIN。
    int16_t temperature = INT16_MIN;
    if ((snapshot->heater_sensor_valid != 0u) && isfinite(snapshot->temperature_c) &&
        (snapshot->temperature_c >= -327.67f) && (snapshot->temperature_c <= 327.67f))
    {
        long rounded = lroundf(snapshot->temperature_c * 100.0f);
        if ((rounded > INT16_MIN) && (rounded <= INT16_MAX))
            temperature = (int16_t)rounded;
    }
    PutU16(candidate, &offset, (uint16_t)temperature);

    // 保留一个零字节，固定118字节版本1格式，后续可扩展而不改变现有偏移。
    candidate[offset++] = 0u;

    if (offset != VEHICLE_HEALTH_DIAGNOSTIC_DATA_SIZE)
        return VEHICLE_HEALTH_ERROR_CAPTURE;
    for (uint32_t index = 0u; index < offset; ++index)
        output[index] = candidate[index];
    *written = offset;
    return VEHICLE_HEALTH_OK;
}
