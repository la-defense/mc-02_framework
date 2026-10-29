/** @file robot_alarm_policy.c
 * @brief 仲裁 safety 告警，按实际经过时间生成 LED 与音调输出请求。
 * @note 本实现只依赖标准 C；输出由任务上下文提供，策略本身不访问硬件。
 * @author MC-02 project contributors
 * @copyright Copyright (c) HNU YueLu EC 2022 all rights reserved
 */
#include "robot_alarm_policy.h"

#include <string.h>

typedef struct
{
    RobotAlarmEventKind_t kind;
    uint32_t id;
    const RobotAlarmStyle_t *style;
    uint8_t mute_level;
    uint8_t mute_allowed;
} RobotAlarmSelection_t;

static uint32_t RobotAlarmSaturatingIncrement(uint32_t value)
{
    return value == UINT32_MAX ? UINT32_MAX : value + 1u;
}

static uint32_t RobotAlarmLedCycleMs(const RobotAlarmLedPattern_t *pattern)
{
    uint32_t total = 0u;
    for (uint8_t i = 0u; i < pattern->segment_count; ++i)
        total += pattern->segments[i].duration_ms;
    return total;
}

static uint32_t RobotAlarmSoundCycleMs(const RobotAlarmSoundPattern_t *pattern)
{
    uint32_t total = 0u;
    for (uint8_t i = 0u; i < pattern->segment_count; ++i)
        total += pattern->segments[i].duration_ms;
    return total;
}

static uint8_t RobotAlarmStyleIsValid(const RobotAlarmStyle_t *style,
                                      uint16_t max_pattern_ms)
{
    if (style == NULL || style->led.segment_count > ROBOT_ALARM_MAX_SEGMENTS ||
        style->sound.segment_count > ROBOT_ALARM_MAX_SEGMENTS)
        return 0u;

    const RobotAlarmLedPattern_t *led = &style->led;
    switch (led->mode)
    {
    case ROBOT_ALARM_LED_OFF:
    case ROBOT_ALARM_LED_SOLID:
        if (led->segment_count != 0u)
            return 0u;
        break;
    case ROBOT_ALARM_LED_BLINK:
    case ROBOT_ALARM_LED_BREATHE:
        if (led->period_ms < 2u || led->period_ms > max_pattern_ms ||
            led->segment_count != 0u)
            return 0u;
        break;
    case ROBOT_ALARM_LED_SEGMENTED:
        if (led->segment_count == 0u || led->segments == NULL ||
            RobotAlarmLedCycleMs(led) == 0u ||
            RobotAlarmLedCycleMs(led) > max_pattern_ms)
            return 0u;
        break;
    default:
        return 0u;
    }
    if (led->segment_count != 0u)
    {
        for (uint8_t i = 0u; i < led->segment_count; ++i)
            if (led->segments[i].duration_ms == 0u)
                return 0u;
    }

    const RobotAlarmSoundPattern_t *sound = &style->sound;
    if ((sound->segment_count != 0u && sound->segments == NULL) ||
        RobotAlarmSoundCycleMs(sound) > max_pattern_ms)
        return 0u;
    for (uint8_t i = 0u; i < sound->segment_count; ++i)
    {
        const RobotAlarmSoundSegment_t *segment = &sound->segments[i];
        if (segment->duration_ms == 0u ||
            (int)segment->note < (int)BUZZER_SILENT ||
            segment->note >= BUZZER_NOTE_COUNT ||
            segment->duty_permille > 1000u ||
            (segment->note == BUZZER_SILENT && segment->duty_permille != 0u) ||
            (segment->note != BUZZER_SILENT && segment->duty_permille == 0u))
            return 0u;
    }
    return 1u;
}

RobotAlarmStatus_t RobotAlarmConfigValidate(const RobotAlarmConfig_t *config)
{
    if (config == NULL)
        return ROBOT_ALARM_ERROR_ARGUMENT;
    if (config->fault_rules == NULL || config->state_rules == NULL ||
        config->fault_rule_count != 9u ||
        config->state_rule_count != ROBOT_ALARM_STATE_COUNT ||
        config->max_pattern_ms == 0u ||
        config->max_pattern_ms > ROBOT_ALARM_MAX_PATTERN_MS ||
        config->max_mute_ms == 0u || config->max_mute_ms > 60000u)
        return ROBOT_ALARM_ERROR_CONFIG;

    uint32_t seen_faults = 0u;
    for (uint8_t i = 0u; i < config->fault_rule_count; ++i)
    {
        const RobotAlarmFaultRule_t *rule = &config->fault_rules[i];
        if (rule->fault_bit == 0u || (rule->fault_bit & (rule->fault_bit - 1u)) != 0u ||
            (seen_faults & rule->fault_bit) != 0u ||
            rule->mute_level >= ROBOT_ALARM_MAX_MUTE_LEVELS ||
            !RobotAlarmStyleIsValid(&rule->style, config->max_pattern_ms))
            return ROBOT_ALARM_ERROR_CONFIG;
        seen_faults |= rule->fault_bit;
    }
    if (seen_faults != ROBOT_ALARM_KNOWN_FAULT_MASK ||
        !RobotAlarmStyleIsValid(&config->unknown_fault_style, config->max_pattern_ms) ||
        !RobotAlarmStyleIsValid(&config->unknown_state_style, config->max_pattern_ms))
        return ROBOT_ALARM_ERROR_CONFIG;

    uint32_t seen_states = 0u;
    for (uint8_t i = 0u; i < config->state_rule_count; ++i)
    {
        const RobotAlarmStateRule_t *rule = &config->state_rules[i];
        if ((int)rule->state < (int)ROBOT_ALARM_STATE_STOP ||
            rule->state >= ROBOT_ALARM_STATE_COUNT ||
            (seen_states & (1u << rule->state)) != 0u ||
            !RobotAlarmStyleIsValid(&rule->style, config->max_pattern_ms))
            return ROBOT_ALARM_ERROR_CONFIG;
        seen_states |= 1u << rule->state;
    }
    if (seen_states != ((1u << ROBOT_ALARM_STATE_COUNT) - 1u))
        return ROBOT_ALARM_ERROR_CONFIG;
    return ROBOT_ALARM_OK;
}

static RobotAlarmSelection_t RobotAlarmSelect(const RobotAlarmConfig_t *config,
                                               const RobotAlarmInput_t *input)
{
    RobotAlarmSelection_t selection = {0};
    if (input->estop_latched != 0u)
    {
        selection.kind = ROBOT_ALARM_EVENT_ESTOP;
        selection.id = 0u;
        selection.style = &config->state_rules[0].style;
        for (uint8_t i = 0u; i < config->state_rule_count; ++i)
            if (config->state_rules[i].state == ROBOT_ALARM_STATE_ESTOP)
                selection.style = &config->state_rules[i].style;
        return selection;
    }

    const uint32_t known_flags = input->fault_flags & ROBOT_ALARM_KNOWN_FAULT_MASK;
    const uint32_t unknown_flags = input->fault_flags & ~ROBOT_ALARM_KNOWN_FAULT_MASK;
    if (known_flags != 0u)
    {
        const RobotAlarmFaultRule_t *best = NULL;
        for (uint8_t i = 0u; i < config->fault_rule_count; ++i)
        {
            const RobotAlarmFaultRule_t *rule = &config->fault_rules[i];
            if ((known_flags & rule->fault_bit) != 0u &&
                (best == NULL || rule->priority < best->priority))
                best = rule;
        }
        selection.kind = ROBOT_ALARM_EVENT_FAULT;
        selection.id = best->fault_bit;
        selection.style = &best->style;
        selection.mute_level = best->mute_level;
        selection.mute_allowed = (uint8_t)((config->mute_level_mask &
                                           (uint8_t)(1u << best->mute_level)) != 0u);
        return selection;
    }
    if (unknown_flags != 0u)
    {
        selection.kind = ROBOT_ALARM_EVENT_UNKNOWN_FAULT;
        selection.id = unknown_flags;
        selection.style = &config->unknown_fault_style;
        return selection;
    }

    if (input->state < ROBOT_ALARM_STATE_COUNT)
    {
        for (uint8_t i = 0u; i < config->state_rule_count; ++i)
        {
            if (config->state_rules[i].state == input->state)
            {
                selection.kind = ROBOT_ALARM_EVENT_STATE;
                selection.id = (uint32_t)input->state;
                selection.style = &config->state_rules[i].style;
                return selection;
            }
        }
    }
    selection.kind = ROBOT_ALARM_EVENT_UNKNOWN_STATE;
    selection.id = (uint32_t)input->state;
    selection.style = &config->unknown_state_style;
    return selection;
}

static uint8_t RobotAlarmEventChanged(const RobotAlarmState_t *state,
                                      const RobotAlarmSelection_t *selection)
{
    return state->event_kind != selection->kind || state->event_id != selection->id;
}

static void RobotAlarmSelectLed(const RobotAlarmLedPattern_t *pattern,
                                uint32_t elapsed_ms,
                                uint8_t *r,
                                uint8_t *g,
                                uint8_t *b)
{
    *r = 0u;
    *g = 0u;
    *b = 0u;
    switch (pattern->mode)
    {
    case ROBOT_ALARM_LED_OFF:
        return;
    case ROBOT_ALARM_LED_SOLID:
        *r = pattern->r;
        *g = pattern->g;
        *b = pattern->b;
        return;
    case ROBOT_ALARM_LED_BLINK:
        if ((elapsed_ms % pattern->period_ms) < (pattern->period_ms / 2u))
        {
            *r = pattern->r;
            *g = pattern->g;
            *b = pattern->b;
        }
        return;
    case ROBOT_ALARM_LED_BREATHE:
    {
        const uint32_t phase = elapsed_ms % pattern->period_ms;
        const uint32_t half = pattern->period_ms / 2u;
        const uint32_t level = phase < half ? (phase * 255u) / half :
                               ((pattern->period_ms - phase) * 255u) /
                                   (pattern->period_ms - half);
        *r = (uint8_t)(((uint32_t)pattern->r * level) / 255u);
        *g = (uint8_t)(((uint32_t)pattern->g * level) / 255u);
        *b = (uint8_t)(((uint32_t)pattern->b * level) / 255u);
        return;
    }
    case ROBOT_ALARM_LED_SEGMENTED:
    {
        const uint32_t cycle = RobotAlarmLedCycleMs(pattern);
        uint32_t phase = elapsed_ms % cycle;
        for (uint8_t i = 0u; i < pattern->segment_count; ++i)
        {
            const RobotAlarmLedSegment_t *segment = &pattern->segments[i];
            if (phase < segment->duration_ms)
            {
                *r = segment->r;
                *g = segment->g;
                *b = segment->b;
                return;
            }
            phase -= segment->duration_ms;
        }
        return;
    }
    default:
        return;
    }
}

static void RobotAlarmSelectSound(const RobotAlarmSoundPattern_t *pattern,
                                  uint32_t elapsed_ms,
                                  BuzzerNote_t *note,
                                  uint16_t *duty_permille)
{
    *note = BUZZER_SILENT;
    *duty_permille = 0u;
    if (pattern->segment_count == 0u)
        return;

    const uint32_t cycle = RobotAlarmSoundCycleMs(pattern);
    uint32_t phase = elapsed_ms % cycle;
    for (uint8_t i = 0u; i < pattern->segment_count; ++i)
    {
        const RobotAlarmSoundSegment_t *segment = &pattern->segments[i];
        if (phase < segment->duration_ms)
        {
            *note = segment->note;
            *duty_permille = segment->duty_permille;
            return;
        }
        phase -= segment->duration_ms;
    }
}

static RobotAlarmStatus_t RobotAlarmApplyLed(RobotAlarmPolicy_t *policy,
                                              uint8_t r,
                                              uint8_t g,
                                              uint8_t b)
{
    if (policy->state.led_available == 0u)
        return ROBOT_ALARM_DEGRADED;
    if (policy->led_output_valid != 0u && policy->led_r == r &&
        policy->led_g == g && policy->led_b == b)
        return ROBOT_ALARM_OK;

    const RobotAlarmWriteResult_t result = policy->output.write_led(
        policy->output.context, r, g, b);
    if (result == ROBOT_ALARM_WRITE_APPLIED)
    {
        policy->led_output_valid = 1u;
        policy->led_r = r;
        policy->led_g = g;
        policy->led_b = b;
        return ROBOT_ALARM_OK;
    }
    if (result == ROBOT_ALARM_WRITE_BUSY)
    {
        policy->state.led_busy_count = RobotAlarmSaturatingIncrement(
            policy->state.led_busy_count);
        return ROBOT_ALARM_DEGRADED;
    }
    policy->state.led_available = 0u;
    policy->state.led_error_count = RobotAlarmSaturatingIncrement(
        policy->state.led_error_count);
    policy->led_output_valid = 0u;
    return ROBOT_ALARM_DEGRADED;
}

static RobotAlarmStatus_t RobotAlarmApplyBuzzer(RobotAlarmPolicy_t *policy,
                                                 BuzzerNote_t note,
                                                 uint16_t duty_permille)
{
    if (policy->state.buzzer_available == 0u)
        return ROBOT_ALARM_DEGRADED;
    if (policy->buzzer_output_valid != 0u && policy->buzzer_note == note &&
        policy->buzzer_duty_permille == duty_permille)
        return ROBOT_ALARM_OK;

    const RobotAlarmWriteResult_t result = policy->output.write_buzzer(
        policy->output.context, note, duty_permille);
    if (result == ROBOT_ALARM_WRITE_APPLIED)
    {
        policy->buzzer_output_valid = 1u;
        policy->buzzer_note = note;
        policy->buzzer_duty_permille = duty_permille;
        return ROBOT_ALARM_OK;
    }
    if (result == ROBOT_ALARM_WRITE_BUSY)
    {
        policy->state.buzzer_busy_count = RobotAlarmSaturatingIncrement(
            policy->state.buzzer_busy_count);
        return ROBOT_ALARM_DEGRADED;
    }
    policy->state.buzzer_available = 0u;
    policy->state.buzzer_error_count = RobotAlarmSaturatingIncrement(
        policy->state.buzzer_error_count);
    policy->buzzer_output_valid = 0u;
    return ROBOT_ALARM_DEGRADED;
}

RobotAlarmStatus_t RobotAlarmPolicyInit(RobotAlarmPolicy_t *policy,
                                        const RobotAlarmConfig_t *config,
                                        const RobotAlarmOutputOps_t *output,
                                        uint8_t led_available,
                                        uint8_t buzzer_available)
{
    if (policy == NULL || output == NULL)
        return ROBOT_ALARM_ERROR_ARGUMENT;
    if (RobotAlarmConfigValidate(config) != ROBOT_ALARM_OK)
        return ROBOT_ALARM_ERROR_CONFIG;
    if ((led_available != 0u && output->write_led == NULL) ||
        (buzzer_available != 0u && output->write_buzzer == NULL))
        return ROBOT_ALARM_ERROR_ARGUMENT;

    RobotAlarmPolicy_t candidate;
    memset(&candidate, 0, sizeof(candidate));
    candidate.config = config;
    candidate.output = *output;
    candidate.state.led_available = led_available != 0u ? 1u : 0u;
    candidate.state.buzzer_available = buzzer_available != 0u ? 1u : 0u;
    candidate.state.last_status = (led_available != 0u && buzzer_available != 0u) ?
                                      ROBOT_ALARM_OK : ROBOT_ALARM_DEGRADED;
    candidate.initialized = 1u;
    *policy = candidate;
    return candidate.state.last_status;
}

RobotAlarmStatus_t RobotAlarmPolicyUpdate(RobotAlarmPolicy_t *policy,
                                          const RobotAlarmInput_t *input,
                                          uint32_t now_ms)
{
    if (policy == NULL || input == NULL)
        return ROBOT_ALARM_ERROR_ARGUMENT;
    if (policy->initialized == 0u)
        return ROBOT_ALARM_ERROR_UNINITIALIZED;

    const RobotAlarmSelection_t selection = RobotAlarmSelect(policy->config, input);
    if (RobotAlarmEventChanged(&policy->state, &selection))
    {
        policy->state.event_kind = selection.kind;
        policy->state.event_id = selection.id;
        policy->state.event_started_ms = now_ms;
        ++policy->state.event_generation;
        if (policy->state.event_generation == 0u)
            ++policy->state.event_generation;
        policy->state.mute_active = 0u;
        policy->led_output_valid = 0u;
        policy->buzzer_output_valid = 0u;
    }
    policy->mute_allowed = selection.mute_allowed;
    policy->state.mute_level = selection.mute_level;
    policy->state.mute_allowed = selection.mute_allowed;

    const uint32_t elapsed_ms = now_ms - policy->state.event_started_ms;
    if (policy->state.mute_active != 0u &&
        (uint32_t)(now_ms - policy->mute_started_ms) >= policy->mute_duration_ms)
        policy->state.mute_active = 0u;

    uint8_t r, g, b;
    BuzzerNote_t note;
    uint16_t duty_permille;
    RobotAlarmSelectLed(&selection.style->led, elapsed_ms, &r, &g, &b);
    RobotAlarmSelectSound(&selection.style->sound, elapsed_ms, &note, &duty_permille);
    if (policy->state.mute_active != 0u)
    {
        note = BUZZER_SILENT;
        duty_permille = 0u;
    }

    const RobotAlarmStatus_t led_status = RobotAlarmApplyLed(policy, r, g, b);
    const RobotAlarmStatus_t buzzer_status = RobotAlarmApplyBuzzer(policy, note,
                                                                   duty_permille);
    policy->state.last_status =
        (led_status == ROBOT_ALARM_OK && buzzer_status == ROBOT_ALARM_OK &&
         policy->state.led_available != 0u && policy->state.buzzer_available != 0u) ?
            ROBOT_ALARM_OK : ROBOT_ALARM_DEGRADED;
    return policy->state.last_status;
}

RobotAlarmStatus_t RobotAlarmPolicyMuteFor(RobotAlarmPolicy_t *policy,
                                           uint32_t now_ms,
                                           uint32_t duration_ms)
{
    if (policy == NULL)
        return ROBOT_ALARM_ERROR_ARGUMENT;
    if (policy->initialized == 0u)
        return ROBOT_ALARM_ERROR_UNINITIALIZED;
    if (duration_ms == 0u || duration_ms > policy->config->max_mute_ms)
        return ROBOT_ALARM_ERROR_RANGE;
    if (policy->state.event_kind != ROBOT_ALARM_EVENT_FAULT ||
        policy->mute_allowed == 0u)
        return ROBOT_ALARM_ERROR_MUTE_DENIED;

    policy->mute_started_ms = now_ms;
    policy->mute_duration_ms = duration_ms;
    policy->state.mute_active = 1u;
    return ROBOT_ALARM_OK;
}

RobotAlarmStatus_t RobotAlarmPolicyGetState(const RobotAlarmPolicy_t *policy,
                                            RobotAlarmState_t *state)
{
    if (policy == NULL || state == NULL)
        return ROBOT_ALARM_ERROR_ARGUMENT;
    if (policy->initialized == 0u)
        return ROBOT_ALARM_ERROR_UNINITIALIZED;
    *state = policy->state;
    return ROBOT_ALARM_OK;
}
