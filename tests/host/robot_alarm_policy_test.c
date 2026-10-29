/** @file robot_alarm_policy_test.c
 * @brief 用固定车型表检查生产告警仲裁和按时间推进的输出策略。
 * @note 每个场景由独立进程运行；输出回调只记录策略请求，不连接板载外设。
 */
#include "test_check.h"
#include "robot_alarm_policy.h"

#include <stdint.h>
#include <string.h>

typedef struct
{
    RobotAlarmWriteResult_t led_result;
    RobotAlarmWriteResult_t buzzer_result;
    uint8_t r;
    uint8_t g;
    uint8_t b;
    BuzzerNote_t note;
    uint16_t duty;
    uint32_t led_calls;
    uint32_t buzzer_calls;
} OutputLog_t;

static OutputLog_t output_log;
static RobotAlarmLedSegment_t led_segments[] = {
    {10u, 255u, 0u, 0u}, {20u, 0u, 0u, 255u}};
static RobotAlarmSoundSegment_t sound_segments[] = {
    {10u, OCTAVE_1, 100u}, {10u, BUZZER_SILENT, 0u}};
static RobotAlarmFaultRule_t fault_rules[9];
static RobotAlarmStateRule_t state_rules[ROBOT_ALARM_STATE_COUNT];
static RobotAlarmConfig_t config;

static RobotAlarmWriteResult_t WriteLed(void *context, uint8_t r, uint8_t g,
                                        uint8_t b)
{
    OutputLog_t *log = (OutputLog_t *)context;
    ++log->led_calls;
    log->r = r;
    log->g = g;
    log->b = b;
    return log->led_result;
}

static RobotAlarmWriteResult_t WriteBuzzer(void *context, BuzzerNote_t note,
                                           uint16_t duty_permille)
{
    OutputLog_t *log = (OutputLog_t *)context;
    ++log->buzzer_calls;
    log->note = note;
    log->duty = duty_permille;
    return log->buzzer_result;
}

static void PrepareConfig(void)
{
    const RobotAlarmStyle_t blink_sound = {
        .led = {.mode = ROBOT_ALARM_LED_BLINK, .period_ms = 20u,
                .r = 255u, .g = 40u, .b = 10u},
        .sound = {.segment_count = 2u, .segments = sound_segments},
    };
    for (uint8_t i = 0u; i < 9u; ++i)
    {
        fault_rules[i].fault_bit = 1u << i;
        fault_rules[i].priority = i;
        fault_rules[i].mute_level = i < ROBOT_ALARM_MAX_MUTE_LEVELS ? i :
                                                                    ROBOT_ALARM_MAX_MUTE_LEVELS - 1u;
        fault_rules[i].style = blink_sound;
    }
    fault_rules[0].style.led.mode = ROBOT_ALARM_LED_SEGMENTED;
    fault_rules[0].style.led.segment_count = 2u;
    fault_rules[0].style.led.segments = led_segments;

    state_rules[ROBOT_ALARM_STATE_STOP] = (RobotAlarmStateRule_t){
        .state = ROBOT_ALARM_STATE_STOP,
        .style = {.led = {.mode = ROBOT_ALARM_LED_OFF}},};
    state_rules[ROBOT_ALARM_STATE_INIT] = (RobotAlarmStateRule_t){
        .state = ROBOT_ALARM_STATE_INIT,
        .style = {.led = {.mode = ROBOT_ALARM_LED_BLINK, .period_ms = 20u,
                          .r = 0u, .g = 0u, .b = 200u}},};
    state_rules[ROBOT_ALARM_STATE_SAFE] = (RobotAlarmStateRule_t){
        .state = ROBOT_ALARM_STATE_SAFE,
        .style = {.led = {.mode = ROBOT_ALARM_LED_SOLID, .r = 1u, .g = 2u,
                          .b = 3u}},};
    state_rules[ROBOT_ALARM_STATE_CALIB] = (RobotAlarmStateRule_t){
        .state = ROBOT_ALARM_STATE_CALIB,
        .style = {.led = {.mode = ROBOT_ALARM_LED_BREATHE, .period_ms = 20u,
                          .r = 200u, .g = 0u, .b = 0u}},};
    state_rules[ROBOT_ALARM_STATE_READY] = (RobotAlarmStateRule_t){
        .state = ROBOT_ALARM_STATE_READY,
        .style = {.led = {.mode = ROBOT_ALARM_LED_SOLID, .r = 0u, .g = 200u,
                          .b = 0u}},};
    state_rules[ROBOT_ALARM_STATE_FAULT] = (RobotAlarmStateRule_t){
        .state = ROBOT_ALARM_STATE_FAULT,
        .style = {.led = {.mode = ROBOT_ALARM_LED_SOLID, .r = 200u, .g = 0u,
                          .b = 0u}},};
    state_rules[ROBOT_ALARM_STATE_ESTOP] = (RobotAlarmStateRule_t){
        .state = ROBOT_ALARM_STATE_ESTOP,
        .style = {.led = {.mode = ROBOT_ALARM_LED_SEGMENTED,
                          .segment_count = 2u, .segments = led_segments}},};

    config = (RobotAlarmConfig_t){
        .fault_rule_count = 9u,
        .fault_rules = fault_rules,
        .state_rule_count = ROBOT_ALARM_STATE_COUNT,
        .state_rules = state_rules,
        .unknown_fault_style = {.led = {.mode = ROBOT_ALARM_LED_SOLID,
                                        .r = 100u, .g = 0u, .b = 100u}},
        .unknown_state_style = {.led = {.mode = ROBOT_ALARM_LED_SOLID,
                                        .r = 100u, .g = 100u, .b = 0u}},
        .mute_level_mask = (1u << 4) | (1u << 5),
        .max_mute_ms = 1000u,
        .max_pattern_ms = ROBOT_ALARM_MAX_PATTERN_MS,
    };
}

static void InitPolicy(RobotAlarmPolicy_t *policy)
{
    const RobotAlarmOutputOps_t output = {
        .write_led = WriteLed,
        .write_buzzer = WriteBuzzer,
        .context = &output_log,
    };
    memset(&output_log, 0, sizeof(output_log));
    output_log.led_result = ROBOT_ALARM_WRITE_APPLIED;
    output_log.buzzer_result = ROBOT_ALARM_WRITE_APPLIED;
    TEST_CHECK(RobotAlarmPolicyInit(policy, &config, &output, 1u, 1u) ==
               ROBOT_ALARM_OK);
}

static RobotAlarmState_t Update(RobotAlarmPolicy_t *policy,
                                RobotAlarmRobotState_t state,
                                uint32_t faults,
                                uint8_t estop,
                                uint32_t now_ms)
{
    const RobotAlarmInput_t input = {
        .state = state,
        .fault_flags = faults,
        .estop_latched = estop,
    };
    TEST_CHECK(RobotAlarmPolicyUpdate(policy, &input, now_ms) == ROBOT_ALARM_OK);
    RobotAlarmState_t current;
    TEST_CHECK(RobotAlarmPolicyGetState(policy, &current) == ROBOT_ALARM_OK);
    return current;
}

static void TestPriority(void)
{
    RobotAlarmPolicy_t policy;
    InitPolicy(&policy);
    RobotAlarmState_t state = Update(&policy, ROBOT_ALARM_STATE_READY,
                                     (1u << 4) | (1u << 1), 0u, 10u);
    TEST_CHECK(state.event_kind == ROBOT_ALARM_EVENT_FAULT);
    TEST_CHECK(state.event_id == (1u << 1));
    TEST_CHECK(state.event_started_ms == 10u);

    // 相同优先级按配置表顺序选择，结果不依赖故障位的数值大小。
    fault_rules[2].priority = 1u;
    state = Update(&policy, ROBOT_ALARM_STATE_READY,
                   (1u << 2) | (1u << 1), 0u, 11u);
    TEST_CHECK(state.event_id == (1u << 1));

    // 故障优先于状态提示；急停保持时，故障清除后才显示急停状态。
    state = Update(&policy, ROBOT_ALARM_STATE_READY,
                   (1u << 4) | (1u << 1), 1u, 12u);
    TEST_CHECK(state.event_kind == ROBOT_ALARM_EVENT_FAULT &&
               state.event_id == (1u << 1));
    state = Update(&policy, ROBOT_ALARM_STATE_READY, (1u << 4), 1u, 13u);
    TEST_CHECK(state.event_kind == ROBOT_ALARM_EVENT_FAULT &&
               state.event_id == (1u << 4));
    state = Update(&policy, ROBOT_ALARM_STATE_READY, 0u, 1u, 14u);
    TEST_CHECK(state.event_kind == ROBOT_ALARM_EVENT_ESTOP);
    state = Update(&policy, ROBOT_ALARM_STATE_READY, 0u, 0u, 15u);
    TEST_CHECK(state.event_kind == ROBOT_ALARM_EVENT_STATE &&
               state.event_id == ROBOT_ALARM_STATE_READY);
    state = Update(&policy, ROBOT_ALARM_STATE_READY, 1u << 20, 1u, 16u);
    TEST_CHECK(state.event_kind == ROBOT_ALARM_EVENT_UNKNOWN_FAULT &&
               state.event_id == (1u << 20));
}

static void TestTiming(void)
{
    RobotAlarmPolicy_t policy;
    InitPolicy(&policy);
    RobotAlarmState_t state = Update(&policy, ROBOT_ALARM_STATE_INIT, 0u, 0u, 100u);
    TEST_CHECK(state.event_started_ms == 100u);
    TEST_CHECK(output_log.r == 0u && output_log.g == 0u && output_log.b == 200u);
    const uint32_t led_calls = output_log.led_calls;
    state = Update(&policy, ROBOT_ALARM_STATE_INIT, 0u, 0u, 110u);
    TEST_CHECK(output_log.led_calls == led_calls + 1u);
    TEST_CHECK(output_log.r == 0u && output_log.g == 0u && output_log.b == 0u);

    state = Update(&policy, ROBOT_ALARM_STATE_READY, 1u, 0u, 200u);
    TEST_CHECK(state.event_started_ms == 200u);
    TEST_CHECK(output_log.note == OCTAVE_1 && output_log.duty == 100u);
    state = Update(&policy, ROBOT_ALARM_STATE_READY, 1u, 0u, 210u);
    TEST_CHECK(output_log.note == BUZZER_SILENT && output_log.duty == 0u);
    (void)Update(&policy, ROBOT_ALARM_STATE_CALIB, 0u, 0u, 220u);
    TEST_CHECK(output_log.r == 0u); // 呼吸模式在半周期处亮度为零。

    (void)Update(&policy, ROBOT_ALARM_STATE_READY, 1u, 0u, UINT32_MAX - 3u);
    state = Update(&policy, ROBOT_ALARM_STATE_READY, 1u, 0u, 2u);
    TEST_CHECK(state.event_started_ms == UINT32_MAX - 3u);
    TEST_CHECK(output_log.note == OCTAVE_1); // 跨回绕已走过6ms，仍在首个音符段。

    // 同一故障跨过完整计数周期时，节奏应继续按累计时间推进。
    InitPolicy(&policy);
    (void)Update(&policy, ROBOT_ALARM_STATE_READY, 1u, 0u, 0u);
    (void)Update(&policy, ROBOT_ALARM_STATE_READY, 1u, 0u, UINT32_MAX);
    (void)Update(&policy, ROBOT_ALARM_STATE_READY, 1u, 0u, 0u);
    TEST_CHECK(output_log.r == 0u && output_log.b == 255u);
    TEST_CHECK(output_log.note == BUZZER_SILENT);

    // 单次播放结束保持末段，循环模式在同一边界重新进入首段。
    fault_rules[0].style.led.repeat = ROBOT_ALARM_REPEAT_ONCE_HOLD;
    InitPolicy(&policy);
    (void)Update(&policy, ROBOT_ALARM_STATE_READY, 1u, 0u, 0u);
    (void)Update(&policy, ROBOT_ALARM_STATE_READY, 1u, 0u, 30u);
    TEST_CHECK(output_log.r == 0u && output_log.b == 255u);
    (void)Update(&policy, ROBOT_ALARM_STATE_READY, 1u, 0u, 100u);
    TEST_CHECK(output_log.r == 0u && output_log.b == 255u);
    fault_rules[0].style.led.repeat = ROBOT_ALARM_REPEAT_LOOP;
    InitPolicy(&policy);
    (void)Update(&policy, ROBOT_ALARM_STATE_READY, 1u, 0u, 0u);
    (void)Update(&policy, ROBOT_ALARM_STATE_READY, 1u, 0u, 30u);
    TEST_CHECK(output_log.r == 255u && output_log.b == 0u);
}

static void TestMute(void)
{
    RobotAlarmPolicy_t policy;
    InitPolicy(&policy);
    (void)Update(&policy, ROBOT_ALARM_STATE_READY, 1u << 3, 0u, 0u);
    TEST_CHECK(RobotAlarmPolicyMuteFor(&policy, 1u, 100u) ==
               ROBOT_ALARM_ERROR_MUTE_DENIED);
    (void)Update(&policy, ROBOT_ALARM_STATE_READY, 1u << 4, 0u, 10u);
    TEST_CHECK(RobotAlarmPolicyMuteFor(&policy, 11u, 100u) == ROBOT_ALARM_OK);
    (void)Update(&policy, ROBOT_ALARM_STATE_READY, 1u << 4, 0u, 11u);
    RobotAlarmState_t state;
    TEST_CHECK(RobotAlarmPolicyGetState(&policy, &state) == ROBOT_ALARM_OK);
    TEST_CHECK(state.mute_active != 0u && output_log.note == BUZZER_SILENT);
    (void)Update(&policy, ROBOT_ALARM_STATE_READY, 1u << 4, 0u, 111u);
    TEST_CHECK(RobotAlarmPolicyGetState(&policy, &state) == ROBOT_ALARM_OK);
    TEST_CHECK(state.mute_active == 0u && output_log.note == OCTAVE_1);

    TEST_CHECK(RobotAlarmPolicyMuteFor(&policy, 112u, 1001u) ==
               ROBOT_ALARM_ERROR_RANGE);
    TEST_CHECK(RobotAlarmPolicyMuteFor(&policy, 112u, 0u) ==
               ROBOT_ALARM_ERROR_RANGE);
}

static void TestValidation(void)
{
    TEST_CHECK(RobotAlarmConfigValidate(&config) == ROBOT_ALARM_OK);
    RobotAlarmConfig_t invalid = config;
    RobotAlarmFaultRule_t saved = fault_rules[1];
    fault_rules[1].fault_bit = fault_rules[0].fault_bit;
    TEST_CHECK(RobotAlarmConfigValidate(&invalid) == ROBOT_ALARM_ERROR_CONFIG);
    fault_rules[1] = saved;

    saved = fault_rules[0];
    fault_rules[0].style.sound = (RobotAlarmSoundPattern_t){
        .segment_count = ROBOT_ALARM_MAX_SEGMENTS + 1u,
        .segments = sound_segments,
    };
    TEST_CHECK(RobotAlarmConfigValidate(&config) == ROBOT_ALARM_ERROR_CONFIG);
    fault_rules[0] = saved;

    saved = fault_rules[0];
    fault_rules[0].style.sound.segments = sound_segments;
    sound_segments[0].duty_permille = 1001u;
    TEST_CHECK(RobotAlarmConfigValidate(&config) == ROBOT_ALARM_ERROR_CONFIG);
    sound_segments[0].duty_permille = 100u;
    fault_rules[0] = saved;

    saved = fault_rules[0];
    fault_rules[0].style.led.repeat = (RobotAlarmRepeat_t)99;
    TEST_CHECK(RobotAlarmConfigValidate(&config) == ROBOT_ALARM_ERROR_CONFIG);
    fault_rules[0] = saved;

    saved = fault_rules[1];
    fault_rules[1].style.led.repeat = ROBOT_ALARM_REPEAT_ONCE_HOLD;
    TEST_CHECK(RobotAlarmConfigValidate(&config) == ROBOT_ALARM_ERROR_CONFIG);
    fault_rules[1] = saved;

    TEST_CHECK(RobotAlarmConfigValidate(NULL) == ROBOT_ALARM_ERROR_ARGUMENT);
}

static void TestOutputs(void)
{
    RobotAlarmPolicy_t policy;
    InitPolicy(&policy);
    RobotAlarmInput_t input = {.state = ROBOT_ALARM_STATE_INIT};
    TEST_CHECK(RobotAlarmPolicyUpdate(&policy, &input, 0u) == ROBOT_ALARM_OK);
    // A 已确认，B 发送忙后又回到 A；硬件仍可能完成 B，必须重新提交 A。
    output_log.led_result = ROBOT_ALARM_WRITE_BUSY;
    TEST_CHECK(RobotAlarmPolicyUpdate(&policy, &input, 10u) == ROBOT_ALARM_DEGRADED);
    const uint32_t led_calls = output_log.led_calls;
    output_log.led_result = ROBOT_ALARM_WRITE_APPLIED;
    TEST_CHECK(RobotAlarmPolicyUpdate(&policy, &input, 20u) == ROBOT_ALARM_OK);
    TEST_CHECK(output_log.led_calls == led_calls + 1u);

    InitPolicy(&policy);
    output_log.led_result = ROBOT_ALARM_WRITE_BUSY;
    input = (RobotAlarmInput_t){.state = ROBOT_ALARM_STATE_READY};
    TEST_CHECK(RobotAlarmPolicyUpdate(&policy, &input, 1u) == ROBOT_ALARM_DEGRADED);
    RobotAlarmState_t state;
    TEST_CHECK(RobotAlarmPolicyGetState(&policy, &state) == ROBOT_ALARM_OK);
    TEST_CHECK(state.led_busy_count == 1u && state.led_available != 0u);
    TEST_CHECK(state.buzzer_error_count == 0u);

    output_log.led_result = ROBOT_ALARM_WRITE_ERROR;
    output_log.buzzer_result = ROBOT_ALARM_WRITE_ERROR;
    input.state = ROBOT_ALARM_STATE_SAFE;
    input.fault_flags = 1u;
    TEST_CHECK(RobotAlarmPolicyUpdate(&policy, &input, 2u) == ROBOT_ALARM_DEGRADED);
    TEST_CHECK(RobotAlarmPolicyGetState(&policy, &state) == ROBOT_ALARM_OK);
    TEST_CHECK(state.last_status == ROBOT_ALARM_DEGRADED);
    TEST_CHECK(state.led_available == 0u && state.buzzer_available == 0u);
    TEST_CHECK(state.led_error_count == 1u && state.buzzer_error_count == 1u);
}

int main(int argc, char **argv)
{
    TEST_CHECK(argc == 2);
    PrepareConfig();
    if (strcmp(argv[1], "priority") == 0)
        TestPriority();
    else if (strcmp(argv[1], "timing") == 0)
        TestTiming();
    else if (strcmp(argv[1], "mute") == 0)
        TestMute();
    else if (strcmp(argv[1], "validation") == 0)
        TestValidation();
    else if (strcmp(argv[1], "outputs") == 0)
        TestOutputs();
    else
        TEST_CHECK(0);
    return EXIT_SUCCESS;
}
