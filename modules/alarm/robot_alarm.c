/** @file robot_alarm.c
 * @brief 把一致的 safety 快照交给告警策略，并连接板载 RGB 与 PWM 蜂鸣器。
 * @note 通道初始化或输出失败只记录在本模块；不修改整车许可和故障位。
 * @author MC-02 project contributors
 * @copyright Copyright (c) HNU YueLu EC 2022 all rights reserved
 */
#include "robot_alarm.h"
#include "robot_alarm_policy.h"

#include "buzzer.h"
#include "led.h"
#include "platform_irq.h"
#include "platform_sync.h"
#include "robot_safety.h"

#include <string.h>

static RobotAlarmPolicy_t robot_alarm_policy;
static const RobotAlarmConfig_t *robot_alarm_config;
static uint8_t robot_alarm_initialized;
static uint8_t mute_request_pending;
static uint8_t mute_request_processing;
static uint32_t mute_request_generation;
static uint32_t mute_request_duration_ms;
static RobotAlarmStatus_t last_mute_status = ROBOT_ALARM_ERROR_UNINITIALIZED;
static RobotAlarmState_t published_alarm_state;

static PlatformIrqState_t RobotAlarmEnterCritical(void)
{
    return PlatformIrqSaveAndDisable(); // 保存已有屏蔽状态并保护请求和输出状态。
}

static void RobotAlarmExitCritical(PlatformIrqState_t previous)
{
    PlatformIrqRestore(previous); // 恢复进入前的状态，允许与外层屏蔽嵌套。
}

static RobotAlarmWriteResult_t RobotAlarmWriteLed(void *context,
                                                 uint8_t r,
                                                 uint8_t g,
                                                 uint8_t b)
{
    (void)context;
    const LEDStatus_t status = LEDSetRGB(r, g, b);
    if (status == LED_STATUS_OK)
        return ROBOT_ALARM_WRITE_APPLIED;
    if (status == LED_STATUS_BUSY)
        return ROBOT_ALARM_WRITE_BUSY;
    return ROBOT_ALARM_WRITE_ERROR;
}

static RobotAlarmWriteResult_t RobotAlarmWriteBuzzer(void *context,
                                                    BuzzerNote_t note,
                                                    uint16_t duty_permille)
{
    (void)context;
    const BuzzerStatus_t status = note == BUZZER_SILENT ?
        (duty_permille == 0u ? BuzzerSilence() : BUZZER_STATUS_ARGUMENT) :
        BuzzerSetTone(note, duty_permille);
    return status == BUZZER_STATUS_OK ? ROBOT_ALARM_WRITE_APPLIED :
                                        ROBOT_ALARM_WRITE_ERROR;
}

static RobotAlarmRobotState_t RobotAlarmMapState(Robot_Status_e state)
{
    switch (state)
    {
    case ROBOT_INIT: return ROBOT_ALARM_STATE_INIT;
    case ROBOT_SAFE: return ROBOT_ALARM_STATE_SAFE;
    case ROBOT_CALIB: return ROBOT_ALARM_STATE_CALIB;
    case ROBOT_READY: return ROBOT_ALARM_STATE_READY;
    case ROBOT_FAULT: return ROBOT_ALARM_STATE_FAULT;
    case ROBOT_ESTOP: return ROBOT_ALARM_STATE_ESTOP;
    case ROBOT_STOP: return ROBOT_ALARM_STATE_STOP;
    default: return ROBOT_ALARM_STATE_COUNT;
    }
}

RobotAlarmStatus_t RobotAlarmInit(const RobotAlarmConfig_t *config)
{
    const RobotAlarmStatus_t validation = RobotAlarmConfigValidate(config);
    if (validation != ROBOT_ALARM_OK)
        return validation;
    if (robot_alarm_initialized != 0u)
        return robot_alarm_config == config ? published_alarm_state.last_status :
               ROBOT_ALARM_ERROR_BUSY;

    const uint8_t led_available = LEDInit(config->rgb_port) == LED_STATUS_OK ? 1u : 0u;
    const uint8_t buzzer_available = BuzzerInit(config->buzzer_pwm) == BUZZER_STATUS_OK ? 1u : 0u;
    const RobotAlarmOutputOps_t output = {
        .write_led = RobotAlarmWriteLed,
        .write_buzzer = RobotAlarmWriteBuzzer,
        .context = NULL,
    };
    const RobotAlarmStatus_t status = RobotAlarmPolicyInit(&robot_alarm_policy,
                                                           config,
                                                           &output,
                                                           led_available,
                                                           buzzer_available);
    if (status != ROBOT_ALARM_OK && status != ROBOT_ALARM_DEGRADED)
        return status;

    robot_alarm_config = config;
    robot_alarm_initialized = 1u;
    last_mute_status = ROBOT_ALARM_ERROR_MUTE_DENIED;
    memset(&published_alarm_state, 0, sizeof(published_alarm_state));
    (void)RobotAlarmPolicyGetState(&robot_alarm_policy, &published_alarm_state);
    published_alarm_state.last_mute_status = last_mute_status;
    return status;
}

RobotAlarmStatus_t RobotAlarmTask(void)
{
    if (robot_alarm_initialized == 0u)
        return ROBOT_ALARM_ERROR_UNINITIALIZED;

    // 即使本轮安全快照暂不可读，也要推进在途 LED 事务的完成与超时。
    LEDProcess();
    RobotSafetySnapshot_t safety_snapshot;
    if (RobotSafetyGetSnapshot(&safety_snapshot) == 0u)
        return ROBOT_ALARM_ERROR_ARGUMENT;

    const RobotAlarmInput_t input = {
        .state = RobotAlarmMapState(safety_snapshot.state),
        .fault_flags = safety_snapshot.fault_flags,
        .estop_latched = safety_snapshot.estop_latched,
    };
    const uint32_t now_ms = PlatformTimeNowMs(); // 使用后端原有的毫秒计数推进灯声节奏。
    RobotAlarmStatus_t result = RobotAlarmPolicyUpdate(&robot_alarm_policy, &input,
                                                       now_ms);
    RobotAlarmState_t current_state;
    if (RobotAlarmPolicyGetState(&robot_alarm_policy, &current_state) != ROBOT_ALARM_OK)
        return ROBOT_ALARM_ERROR_UNINITIALIZED;

    uint8_t apply_mute = 0u;
    uint8_t publish_mute_status = 0u;
    uint32_t request_generation = 0u;
    uint32_t request_duration_ms = 0u;
    RobotAlarmStatus_t requested_mute_status = ROBOT_ALARM_ERROR_MUTE_DENIED;
    const uint32_t previous = RobotAlarmEnterCritical();
    if (mute_request_pending != 0u)
    {
        apply_mute = 1u;
        request_generation = mute_request_generation;
        request_duration_ms = mute_request_duration_ms;
        mute_request_pending = 0u;
        mute_request_processing = 1u;
    }
    RobotAlarmExitCritical(previous);

    if (apply_mute != 0u)
    {
        if (request_generation != current_state.event_generation)
            requested_mute_status = ROBOT_ALARM_ERROR_EVENT_CHANGED;
        else
            requested_mute_status = RobotAlarmPolicyMuteFor(&robot_alarm_policy, now_ms,
                                                            request_duration_ms);
        publish_mute_status = 1u;
        if (requested_mute_status == ROBOT_ALARM_OK)
            result = RobotAlarmPolicyUpdate(&robot_alarm_policy, &input, now_ms);
        (void)RobotAlarmPolicyGetState(&robot_alarm_policy, &current_state);
    }

    const uint32_t publish_previous = RobotAlarmEnterCritical();
    if (publish_mute_status != 0u)
    {
        mute_request_processing = 0u;
        last_mute_status = requested_mute_status;
    }
    current_state.last_mute_status = last_mute_status;
    published_alarm_state = current_state;
    RobotAlarmExitCritical(publish_previous);
    return result;
}

RobotAlarmStatus_t RobotAlarmMuteFor(uint32_t duration_ms)
{
    if (robot_alarm_initialized == 0u)
        return ROBOT_ALARM_ERROR_UNINITIALIZED;
    if (duration_ms == 0u || duration_ms > robot_alarm_config->max_mute_ms)
        return ROBOT_ALARM_ERROR_RANGE;

    const uint32_t previous = RobotAlarmEnterCritical();
    if (published_alarm_state.event_kind != ROBOT_ALARM_EVENT_FAULT ||
        published_alarm_state.mute_allowed == 0u)
    {
        RobotAlarmExitCritical(previous);
        return ROBOT_ALARM_ERROR_MUTE_DENIED;
    }
    if (mute_request_pending != 0u || mute_request_processing != 0u)
    {
        RobotAlarmExitCritical(previous);
        return ROBOT_ALARM_ERROR_BUSY;
    }

    mute_request_generation = published_alarm_state.event_generation;
    mute_request_duration_ms = duration_ms;
    mute_request_pending = 1u;
    last_mute_status = ROBOT_ALARM_QUEUED;
    published_alarm_state.last_mute_status = last_mute_status;
    RobotAlarmExitCritical(previous);
    return ROBOT_ALARM_QUEUED;
}

RobotAlarmStatus_t RobotAlarmGetState(RobotAlarmState_t *state)
{
    if (state == NULL)
        return ROBOT_ALARM_ERROR_ARGUMENT;
    if (robot_alarm_initialized == 0u)
        return ROBOT_ALARM_ERROR_UNINITIALIZED;

    RobotAlarmState_t candidate;
    const uint32_t previous = RobotAlarmEnterCritical();
    candidate = published_alarm_state;
    RobotAlarmExitCritical(previous);
    *state = candidate;
    return ROBOT_ALARM_OK;
}
