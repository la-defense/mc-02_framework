/** @file robot_alarm_adapter_test.c
 * @brief 检查生产告警适配层只读 safety 快照并将外设失败限制在辅助输出。
 * @note HAL、安全输出和告警外设由独立进程中的替身提供；不连接机器人硬件。
 */
#include "test_check.h"
#include "robot_alarm.h"
#include "robot_safety.h"
#include "buzzer.h"
#include "led.h"

#include <stdint.h>
#include <string.h>

static uint32_t tick_ms;
static uint32_t primask;
static RobotSafetySnapshot_t safety_snapshot;
static uint8_t snapshot_valid = 1u;
static LEDStatus_t led_init_result = LED_STATUS_OK;
static BuzzerStatus_t buzzer_init_result = BUZZER_STATUS_OK;
static uint32_t led_init_calls;
static uint32_t led_process_calls;
static uint32_t buzzer_init_calls;
static uint32_t led_write_calls;
static uint32_t buzzer_write_calls;
static uint8_t last_r, last_g, last_b;
static BuzzerNote_t last_note;
static uint16_t last_duty;
static uint8_t request_mute_during_silence;
static RobotAlarmStatus_t callback_mute_status;

static const RobotAlarmSoundSegment_t sound[] = {
    {100u, OCTAVE_2, 200u}};
static RobotAlarmFaultRule_t fault_rules[9];
static RobotAlarmStateRule_t state_rules[ROBOT_ALARM_STATE_COUNT];
static RobotAlarmConfig_t config;

uint32_t HAL_GetTick(void) { return tick_ms; }
uint32_t __get_PRIMASK(void) { return primask; }
void __disable_irq(void) { primask = 1u; }
void __set_PRIMASK(uint32_t previous) { primask = previous; }

uint8_t RobotSafetyGetSnapshot(RobotSafetySnapshot_t *snapshot)
{
    if (snapshot == NULL || snapshot_valid == 0u)
        return 0u;
    *snapshot = safety_snapshot;
    return 1u;
}

LEDStatus_t LEDInit(void)
{
    ++led_init_calls;
    return led_init_result;
}

void LEDProcess(void) { ++led_process_calls; }

LEDStatus_t LEDSetRGB(uint8_t r, uint8_t g, uint8_t b)
{
    ++led_write_calls;
    last_r = r;
    last_g = g;
    last_b = b;
    return LED_STATUS_OK;
}

BuzzerStatus_t BuzzerInit(void)
{
    ++buzzer_init_calls;
    return buzzer_init_result;
}

BuzzerStatus_t BuzzerSetTone(BuzzerNote_t note, uint16_t duty_permille)
{
    ++buzzer_write_calls;
    last_note = note;
    last_duty = duty_permille;
    return BUZZER_STATUS_OK;
}

BuzzerStatus_t BuzzerSilence(void)
{
    ++buzzer_write_calls;
    last_note = BUZZER_SILENT;
    last_duty = 0u;
    if (request_mute_during_silence != 0u)
        callback_mute_status = RobotAlarmMuteFor(10u);
    return BUZZER_STATUS_OK;
}

static void PrepareConfig(void)
{
    const RobotAlarmStyle_t fault_style = {
        .led = {.mode = ROBOT_ALARM_LED_SOLID, .r = 100u, .g = 20u, .b = 10u},
        .sound = {.segment_count = 1u, .segments = sound},
    };
    for (uint8_t i = 0u; i < 9u; ++i)
    {
        fault_rules[i] = (RobotAlarmFaultRule_t){
            .fault_bit = 1u << i,
            .priority = i,
            .mute_level = i < ROBOT_ALARM_MAX_MUTE_LEVELS ? i :
                          ROBOT_ALARM_MAX_MUTE_LEVELS - 1u,
            .style = fault_style,
        };
    }
    for (uint8_t i = 0u; i < ROBOT_ALARM_STATE_COUNT; ++i)
    {
        state_rules[i] = (RobotAlarmStateRule_t){
            .state = (RobotAlarmRobotState_t)i,
            .style = {.led = {.mode = ROBOT_ALARM_LED_SOLID,
                              .r = i, .g = i + 1u, .b = i + 2u}},
        };
    }
    config = (RobotAlarmConfig_t){
        .fault_rule_count = 9u,
        .fault_rules = fault_rules,
        .state_rule_count = ROBOT_ALARM_STATE_COUNT,
        .state_rules = state_rules,
        .unknown_fault_style = {.led = {.mode = ROBOT_ALARM_LED_OFF}},
        .unknown_state_style = {.led = {.mode = ROBOT_ALARM_LED_OFF}},
        .mute_level_mask = 1u << 4,
        .max_mute_ms = 1000u,
        .max_pattern_ms = ROBOT_ALARM_MAX_PATTERN_MS,
    };
}

static void ResetFakeState(void)
{
    tick_ms = 0u;
    primask = 0u;
    safety_snapshot = (RobotSafetySnapshot_t){.state = ROBOT_READY};
    snapshot_valid = 1u;
    led_init_result = LED_STATUS_OK;
    buzzer_init_result = BUZZER_STATUS_OK;
    led_init_calls = 0u;
    led_process_calls = 0u;
    buzzer_init_calls = 0u;
    led_write_calls = 0u;
    buzzer_write_calls = 0u;
    last_r = last_g = last_b = 0u;
    last_note = BUZZER_SILENT;
    last_duty = 0u;
    request_mute_during_silence = 0u;
    callback_mute_status = ROBOT_ALARM_ERROR_UNINITIALIZED;
}

static void TestInitDegraded(void)
{
    ResetFakeState();
    led_init_result = LED_STATUS_TRANSFER;
    buzzer_init_result = BUZZER_STATUS_RESOURCE;
    TEST_CHECK(RobotAlarmInit(&config) == ROBOT_ALARM_DEGRADED);
    TEST_CHECK(led_init_calls == 1u && buzzer_init_calls == 1u);
    TEST_CHECK(RobotAlarmTask() == ROBOT_ALARM_DEGRADED);
    RobotAlarmState_t state;
    TEST_CHECK(RobotAlarmGetState(&state) == ROBOT_ALARM_OK);
    TEST_CHECK(state.led_available == 0u && state.buzzer_available == 0u);
    TEST_CHECK(state.last_status == ROBOT_ALARM_DEGRADED);
    TEST_CHECK(led_write_calls == 0u && buzzer_write_calls == 0u);
}

static void TestPartialChannel(void)
{
    ResetFakeState();
    buzzer_init_result = BUZZER_STATUS_RESOURCE;
    TEST_CHECK(RobotAlarmInit(&config) == ROBOT_ALARM_DEGRADED);
    TEST_CHECK(RobotAlarmTask() == ROBOT_ALARM_DEGRADED);
    TEST_CHECK(led_write_calls == 1u);
    TEST_CHECK(last_r == ROBOT_ALARM_STATE_READY &&
               last_g == ROBOT_ALARM_STATE_READY + 1u &&
               last_b == ROBOT_ALARM_STATE_READY + 2u);
    TEST_CHECK(buzzer_write_calls == 0u);
}

static void TestMuteGeneration(void)
{
    ResetFakeState();
    TEST_CHECK(RobotAlarmInit(&config) == ROBOT_ALARM_OK);
    safety_snapshot.fault_flags = 1u << 4;
    TEST_CHECK(RobotAlarmTask() == ROBOT_ALARM_OK);
    TEST_CHECK(RobotAlarmMuteFor(500u) == ROBOT_ALARM_QUEUED);

    safety_snapshot.fault_flags = 1u << 5;
    tick_ms = 1u;
    TEST_CHECK(RobotAlarmTask() == ROBOT_ALARM_OK);
    RobotAlarmState_t state;
    TEST_CHECK(RobotAlarmGetState(&state) == ROBOT_ALARM_OK);
    TEST_CHECK(state.event_id == (1u << 5));
    TEST_CHECK(state.mute_active == 0u);
    TEST_CHECK(state.last_mute_status == ROBOT_ALARM_ERROR_EVENT_CHANGED);
    TEST_CHECK(last_note == OCTAVE_2 && last_duty == 200u);
}

static void TestMuteRequestDuringProcessing(void)
{
    ResetFakeState();
    TEST_CHECK(RobotAlarmInit(&config) == ROBOT_ALARM_OK);
    safety_snapshot.fault_flags = 1u << 4;
    TEST_CHECK(RobotAlarmTask() == ROBOT_ALARM_OK);
    TEST_CHECK(RobotAlarmMuteFor(500u) == ROBOT_ALARM_QUEUED);
    request_mute_during_silence = 1u;
    tick_ms = 1u;
    TEST_CHECK(RobotAlarmTask() == ROBOT_ALARM_OK);
    TEST_CHECK(callback_mute_status == ROBOT_ALARM_ERROR_BUSY);
    RobotAlarmState_t state;
    TEST_CHECK(RobotAlarmGetState(&state) == ROBOT_ALARM_OK);
    TEST_CHECK(state.mute_active != 0u && state.last_mute_status == ROBOT_ALARM_OK);
}

static void TestSnapshotFailure(void)
{
    ResetFakeState();
    TEST_CHECK(RobotAlarmInit(&config) == ROBOT_ALARM_OK);
    TEST_CHECK(RobotAlarmTask() == ROBOT_ALARM_OK);
    RobotAlarmState_t before, after;
    TEST_CHECK(RobotAlarmGetState(&before) == ROBOT_ALARM_OK);
    snapshot_valid = 0u;
    TEST_CHECK(RobotAlarmTask() == ROBOT_ALARM_ERROR_ARGUMENT);
    TEST_CHECK(led_process_calls == 2u);
    TEST_CHECK(RobotAlarmGetState(&after) == ROBOT_ALARM_OK);
    TEST_CHECK(memcmp(&before, &after, sizeof(before)) == 0);
}

static void TestUnknownState(void)
{
    ResetFakeState();
    safety_snapshot.state = (Robot_Status_e)99;
    TEST_CHECK(RobotAlarmInit(&config) == ROBOT_ALARM_OK);
    TEST_CHECK(RobotAlarmTask() == ROBOT_ALARM_OK);
    RobotAlarmState_t state;
    TEST_CHECK(RobotAlarmGetState(&state) == ROBOT_ALARM_OK);
    TEST_CHECK(state.event_kind == ROBOT_ALARM_EVENT_UNKNOWN_STATE);
    TEST_CHECK(state.event_id == ROBOT_ALARM_STATE_COUNT);
}

static void TestIdempotentInit(void)
{
    ResetFakeState();
    TEST_CHECK(RobotAlarmInit(&config) == ROBOT_ALARM_OK);
    TEST_CHECK(RobotAlarmInit(&config) == ROBOT_ALARM_OK);
    TEST_CHECK(led_init_calls == 1u && buzzer_init_calls == 1u);
    RobotAlarmConfig_t other = config;
    TEST_CHECK(RobotAlarmInit(&other) == ROBOT_ALARM_ERROR_BUSY);
    TEST_CHECK(led_init_calls == 1u && buzzer_init_calls == 1u);
}

int main(int argc, char **argv)
{
    TEST_CHECK(argc == 2);
    PrepareConfig();
    if (strcmp(argv[1], "init-degraded") == 0)
        TestInitDegraded();
    else if (strcmp(argv[1], "partial") == 0)
        TestPartialChannel();
    else if (strcmp(argv[1], "mute-generation") == 0)
        TestMuteGeneration();
    else if (strcmp(argv[1], "mute-serial") == 0)
        TestMuteRequestDuringProcessing();
    else if (strcmp(argv[1], "snapshot-failure") == 0)
        TestSnapshotFailure();
    else if (strcmp(argv[1], "unknown-state") == 0)
        TestUnknownState();
    else if (strcmp(argv[1], "idempotent-init") == 0)
        TestIdempotentInit();
    else
        TEST_CHECK(0);
    return 0;
}
