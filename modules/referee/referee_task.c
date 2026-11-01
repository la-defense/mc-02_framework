/**
 * @file referee_task.c
 * @brief 读取裁判数据并按车型配置分批更新 UI；显示故障不影响裁判接收。
 * @author kidneygood (you@domain.com)
 * @version 0.1
 * @date 2022-11-18
 * @copyright Copyright (c) 2022
 */
#include "referee_task.h"
#include <stdio.h>
#include <string.h>

static const RefereeUIConfig_t *ui_config;
static referee_id_t ui_identity;
static RefereeSnapshot_t ui_snapshot;
static UIState_t state;
typedef struct
{
    uint16_t first, second;
    uint8_t valid;
} UIValue;
static UIValue field_values[REFEREE_UI_MAX_FIELDS];
static uint32_t versions[REFEREE_UI_MAX_FIELDS], last_start_ms;
static uint8_t dirty[REFEREE_UI_MAX_FIELDS], have_start, have_identity;
static uint8_t request_pending, recovery_pending, identity_was_fresh, next_field;

typedef struct {
    uint32_t id, generation, version;
    uint8_t field, initializing, event_ready;
    USARTTxEvent_t event;
} UIFlight;
static UIFlight flight;

static uint32_t UIEnter(void)
{
    uint32_t saved = __get_PRIMASK();
    __disable_irq();
    return saved;
}
static void UIExit(uint32_t saved)
{
    __set_PRIMASK(saved);
}
static void UIPhase(UIPhase_t phase, UIStatus_t error)
{
    uint32_t saved = UIEnter();
    state.phase = phase;
    state.last_error = error;
    UIExit(saved);
}

static uint8_t source_is_fresh(const RefereeSnapshot_t *snapshot, RefereeUIValueSource_t source)
{
    RefereeCommandIndex_t index = REF_CMD_ROBOT_STATE;
    if (source == REFEREE_UI_SOURCE_GAME_STAGE_REMAIN)
        index = REF_CMD_GAME_STATE;
    else if (source == REFEREE_UI_SOURCE_BUFFER_ENERGY || source == REFEREE_UI_SOURCE_HEAT_17MM)
        index = REF_CMD_POWER_HEAT;
    return snapshot->command[index].state == REFEREE_DATA_FRESH;
}

static UIStatus_t ReadFieldValue(unsigned index, UIValue *value)
{
    if (ui_config == NULL || value == NULL || index >= ui_config->field_count)
        return UI_ERROR_ARGUMENT;

    const RefereeUIFieldConfig_t *field = &ui_config->fields[index];
    UIValue current = {0};
    if (source_is_fresh(&ui_snapshot, field->source))
    {
        current.valid = 1u;
        switch (field->source)
        {
        case REFEREE_UI_SOURCE_GAME_STAGE_REMAIN:
        {
            const RefereeGameState_t *game =
                &ui_snapshot.command[REF_CMD_GAME_STATE].data.game_state;
            current.first = game->game_progress;
            current.second = game->stage_remain_s;
            break;
        }
        case REFEREE_UI_SOURCE_ROBOT_HP:
        {
            const RefereeRobotState_t *robot =
                &ui_snapshot.command[REF_CMD_ROBOT_STATE].data.robot_state;
            current.first = robot->current_hp;
            current.second = robot->maximum_hp;
            break;
        }
        case REFEREE_UI_SOURCE_CHASSIS_POWER_LIMIT:
            current.first = ui_snapshot.command[REF_CMD_ROBOT_STATE]
                                .data.robot_state.chassis_power_limit_w;
            break;
        case REFEREE_UI_SOURCE_BUFFER_ENERGY:
            current.first = ui_snapshot.command[REF_CMD_POWER_HEAT]
                                .data.power_heat.buffer_energy_j;
            break;
        case REFEREE_UI_SOURCE_HEAT_17MM:
            current.first = ui_snapshot.command[REF_CMD_POWER_HEAT]
                                .data.power_heat.heat_17mm;
            break;
        default:
            return UI_ERROR_FORMAT;
        }
    }
    *value = current;
    return UI_OK;
}

// 每种来源只读取对应命令的新鲜数据；过期或尚未收到时明确显示“--”。
static UIStatus_t FormatField(unsigned index, char output[REFEREE_UI_TEXT_MAX_BYTES + 1u])
{
    if (ui_config == NULL || output == NULL || index >= ui_config->field_count)
        return UI_ERROR_ARGUMENT;
    const RefereeUIFieldConfig_t *field = &ui_config->fields[index];
    char value[16];
    UIValue current;
    UIStatus_t result = ReadFieldValue(index, &current);
    if (result != UI_OK)
        return result;

    if (current.valid == 0u)
    {
        memcpy(value, "--", 3u);
    }
    else
    {
        int written = 0;
        switch (field->source)
        {
        case REFEREE_UI_SOURCE_GAME_STAGE_REMAIN:
            written = snprintf(value, sizeof(value), "%u/%us",
                               (unsigned)current.first,
                               (unsigned)current.second);
            break;
        case REFEREE_UI_SOURCE_ROBOT_HP:
            written = snprintf(value, sizeof(value), "%u/%u",
                               (unsigned)current.first,
                               (unsigned)current.second);
            break;
        case REFEREE_UI_SOURCE_CHASSIS_POWER_LIMIT:
            written = snprintf(value, sizeof(value), "%uW", (unsigned)current.first);
            break;
        case REFEREE_UI_SOURCE_BUFFER_ENERGY:
            written = snprintf(value, sizeof(value), "%uJ", (unsigned)current.first);
            break;
        case REFEREE_UI_SOURCE_HEAT_17MM:
            written = snprintf(value, sizeof(value), "%u", (unsigned)current.first);
            break;
        default:
            return UI_ERROR_FORMAT;
        }
        if (written < 0)
            return UI_ERROR_FORMAT;
        if ((size_t)written >= sizeof(value))
            return UI_ERROR_LENGTH;
    }

    int written = snprintf(output, REFEREE_UI_TEXT_MAX_BYTES + 1u, "%s%s",
                           field->label, value);
    if (written < 0)
        return UI_ERROR_FORMAT;
    return (size_t)written <= REFEREE_UI_TEXT_MAX_BYTES ? UI_OK : UI_ERROR_LENGTH;
}

uint8_t RefereeUIConfigIsValid(const RefereeUIConfig_t *config)
{
    if (config == NULL || config->fields == NULL || config->field_count == 0u ||
        config->field_count > REFEREE_UI_MAX_FIELDS ||
        config->frame_interval_ms < REFEREE_UI_MIN_FRAME_INTERVAL_MS ||
        config->frame_interval_ms > REFEREE_UI_MAX_FRAME_INTERVAL_MS)
        return 0u;

    for (unsigned i = 0u; i < config->field_count; ++i)
    {
        const RefereeUIFieldConfig_t *field = &config->fields[i];
        if (field->label == NULL || field->x > 1920u || field->y > 1080u ||
            field->font_size > 511u || field->line_width > 1023u || field->color > 8u ||
            field->source < REFEREE_UI_SOURCE_GAME_STAGE_REMAIN ||
            field->source > REFEREE_UI_SOURCE_HEAT_17MM)
            return 0u;

        size_t label_length = 0u;
        while (label_length <= REFEREE_UI_LABEL_MAX_BYTES && field->label[label_length] != '\0')
        {
            unsigned char c = (unsigned char)field->label[label_length];
            if (c < 0x20u || c > 0x7eu)
                return 0u;
            ++label_length;
        }
        if (label_length == 0u || label_length > REFEREE_UI_LABEL_MAX_BYTES ||
            label_length + 11u > REFEREE_UI_TEXT_MAX_BYTES)
            return 0u;

        for (unsigned name_byte = 0u; name_byte < 3u; ++name_byte)
        {
            unsigned char c = field->graphic_name[name_byte];
            if (c < 0x21u || c > 0x7eu)
                return 0u;
        }
        for (unsigned previous = 0u; previous < i; ++previous)
        {
            if (memcmp(field->graphic_name, config->fields[previous].graphic_name, 3u) == 0)
                return 0u;
        }
    }
    return 1u;
}

// 同一字段连续变化时只保留最新文字；在途旧版本完成后不会清掉新变化。
static UIStatus_t ObserveFields(void)
{
    for (unsigned i = 0u; i < ui_config->field_count; ++i)
    {
        UIValue latest;
        UIStatus_t result = ReadFieldValue(i, &latest);
        if (result != UI_OK)
            return result;
        if (versions[i] == 0u || latest.first != field_values[i].first ||
            latest.second != field_values[i].second || latest.valid != field_values[i].valid)
        {
            field_values[i] = latest;
            if (++versions[i] == 0u)
                ++versions[i];
            dirty[i] = 1u;
        }
    }
    return UI_OK;
}

// 发送完成回调只复制结果；UI任务下次运行时才推进字段和初始化步骤。
static void UICompleted(void *context, const USARTTxEvent_t *event)
{
    UIFlight *pending = context;
    uint32_t saved = UIEnter();
    pending->event = *event;
    pending->event_ready = 1u;
    UIExit(saved);
}

static uint8_t UICanStart(void *context, const uint8_t *data, size_t length)
{
    UIFlight *pending = context;
    uint32_t saved = UIEnter();
    uint8_t current = (uint8_t)(pending->generation == state.generation &&
                                (state.phase == UI_PHASE_INITIALIZING || state.phase == UI_PHASE_RUNNING));
    UIExit(saved);
    // 等待发送预算时身份可能过期，实际启动前再次检查当前裁判身份。
    return current && length >= 15u &&
           RefereeIdentityMatches((uint16_t)(data[9] | ((uint16_t)data[10] << 8u)));
}

UIStatus_t UITaskInit(const RefereeUIConfig_t *config)
{
    if (config == NULL)
        return UI_ERROR_ARGUMENT;
    if (!RefereeUIConfigIsValid(config))
        return UI_ERROR_RANGE;
    if (flight.id != 0u)
        return UI_SEND_BUSY;

    uint32_t saved = UIEnter();
    ui_config = config;
    memset(&state, 0, sizeof(state));
    state.phase = UI_PHASE_NEED_INIT;
    memset(field_values, 0, sizeof(field_values));
    memset(versions, 0, sizeof(versions));
    memset(dirty, 0, sizeof(dirty));
    have_start = have_identity = request_pending = recovery_pending = 0u;
    identity_was_fresh = next_field = 0u;
    UIExit(saved);
    return UI_OK;
}

UIStatus_t MyUIInit(void)
{
    if (ui_config == NULL)
        return UI_NOT_INITIALIZED;
    uint32_t saved = UIEnter();
    uint32_t old = flight.id;
    if (++state.generation == 0u)
        ++state.generation;
    state.phase = UI_PHASE_WAIT_ID;
    state.init_step = 0u;
    state.last_error = UI_OK;
    request_pending = 1u;
    recovery_pending = 0u;
    UIExit(saved);
    if (old != 0u)
        (void)RefereeCancelSend(old);
    return UI_SUBMITTED;
}

UIStatus_t UIGetState(UIState_t *output)
{
    if (output == NULL)
        return UI_ERROR_ARGUMENT;
    uint32_t saved = UIEnter();
    *output = state;
    output->pending_id = flight.id;
    UIExit(saved);
    return UI_OK;
}

static void UIIdentityLost(void)
{
    uint32_t saved = UIEnter();
    uint32_t old = flight.id;
    if (have_identity && state.phase != UI_PHASE_UNBOUND && state.phase != UI_PHASE_NEED_INIT)
    {
        if (++state.generation == 0u)
            ++state.generation;
        state.phase = UI_PHASE_NEED_INIT;
        state.last_error = UI_NOT_INITIALIZED;
        state.init_step = 0u;
        recovery_pending = 1u;
    }
    UIExit(saved);
    if (old != 0u)
        (void)RefereeCancelSend(old);
}

#define UI_BUILD(call)                                                        \
    do                                                                        \
    {                                                                         \
        UIStatus_t result_ = (call);                                           \
        if (result_ != UI_OK)                                                  \
            return result_;                                                    \
    } while (0)

// 初始化先清除旧图形，再按配置每帧绘制一个字段，单次调用最多提交一帧。
static UIStatus_t BuildInit(unsigned step, const UITxOptions_t *options)
{
    if (step == 0u)
        return UIDelete(&ui_identity, UI_Data_Del_ALL, 0u, options);
    unsigned field_index = step - 1u;
    if (field_index >= ui_config->field_count)
        return UI_ERROR_RANGE;

    const RefereeUIFieldConfig_t *field = &ui_config->fields[field_index];
    char value[REFEREE_UI_TEXT_MAX_BYTES + 1u];
    String_Data_t text;
    UI_BUILD(FormatField(field_index, value));
    UI_BUILD(UICharDraw(&text, (const char *)field->graphic_name, UI_Graph_ADD, 8u,
                        field->color, field->font_size, field->line_width,
                        field->x, field->y, "%s", value));
    return UICharRefresh(&ui_identity, &text, options);
}

// 增量刷新仍使用原图形名，只修改值或无效占位。
static UIStatus_t BuildChange(unsigned field_index, const UITxOptions_t *options)
{
    if (field_index >= ui_config->field_count)
        return UI_ERROR_RANGE;
    const RefereeUIFieldConfig_t *field = &ui_config->fields[field_index];
    char value[REFEREE_UI_TEXT_MAX_BYTES + 1u];
    String_Data_t text;
    UI_BUILD(FormatField(field_index, value));
    UI_BUILD(UICharDraw(&text, (const char *)field->graphic_name, UI_Graph_Change, 8u,
                        field->color, field->font_size, field->line_width,
                        field->x, field->y, "%s", value));
    return UICharRefresh(&ui_identity, &text, options);
}

static void StartInitialization(uint16_t robot_id)
{
    ui_identity = (referee_id_t){
        .Robot_Color = robot_id >= 100u ? Robot_Blue : Robot_Red,
        .Robot_ID = robot_id,
        .Cilent_ID = (uint16_t)(0x100u + robot_id)};
    have_identity = 1u;
    request_pending = recovery_pending = 0u;
    uint32_t saved = UIEnter();
    state.phase = UI_PHASE_INITIALIZING;
    state.last_error = UI_OK;
    state.init_step = 0u;
    UIExit(saved);
}

UIStatus_t UIRefresh(void)
{
    if (ui_config == NULL)
        return UI_NOT_INITIALIZED;
    if (RefereeGetSnapshot(&ui_snapshot) != REFEREE_OK)
        return UI_NOT_INITIALIZED;
    UIStatus_t observed = ObserveFields();
    if (observed != UI_OK)
    {
        UIPhase(UI_PHASE_ERROR, observed);
        return observed;
    }

    uint32_t saved = UIEnter();
    uint8_t event_ready = flight.event_ready;
    USARTTxEvent_t event = flight.event;
    uint32_t generation = flight.generation;
    uint8_t initializing = flight.initializing, field = flight.field;
    uint32_t version = flight.version;
    if (event_ready)
    {
        flight.event_ready = 0u;
        flight.id = 0u;
    }
    UIExit(saved);

    // 旧代帧的完成结果只释放在途项，不推进新一轮初始化。
    if (event_ready)
    {
        if (event.started)
        {
            last_start_ms = event.started_ms;
            have_start = 1u;
        }
        if (generation == state.generation)
        {
            if (event.result == USART_TX_EXPIRED && !event.started)
            {
                // 等待额度时超期不消耗初始化步数，后续仍可重新提交当前内容。
            }
            else if (event.result != USART_TX_COMPLETE)
            {
                if (event.result == USART_TX_CANCELLED &&
                    !RefereeIdentityMatches(ui_identity.Robot_ID))
                    UIIdentityLost();
                else
                    UIPhase(UI_PHASE_ERROR,
                            event.result == USART_TX_EXPIRED ? UI_SEND_TIMEOUT : UI_SEND_ERROR);
            }
            else if (initializing)
            {
                saved = UIEnter();
                ++state.init_step;
                UIExit(saved);
                if (field < ui_config->field_count && versions[field] == version)
                    dirty[field] = 0u;
                if ((unsigned)state.init_step == (unsigned)ui_config->field_count + 1u)
                    UIPhase(UI_PHASE_RUNNING, UI_OK);
            }
            else if (field < ui_config->field_count && versions[field] == version)
                dirty[field] = 0u;
        }
    }

    uint8_t identity_fresh =
        (uint8_t)(ui_snapshot.command[REF_CMD_ROBOT_STATE].state == REFEREE_DATA_FRESH);
    if (!identity_fresh)
    {
        if (identity_was_fresh)
            UIIdentityLost();
        identity_was_fresh = 0u;
        return state.phase == UI_PHASE_WAIT_ID ? UI_OK : UI_NOT_INITIALIZED;
    }

    identity_was_fresh = 1u;
    uint16_t robot_id = ui_snapshot.command[REF_CMD_ROBOT_STATE].data.robot_state.robot_id;
    if (have_identity && ui_identity.Robot_ID != robot_id)
    {
        UIIdentityLost();
        recovery_pending = 1u;
    }
    if ((state.phase == UI_PHASE_WAIT_ID && request_pending) || recovery_pending)
        StartInitialization(robot_id);
    else if (state.phase == UI_PHASE_NEED_INIT || state.phase == UI_PHASE_ERROR ||
             state.phase == UI_PHASE_WAIT_ID)
        return UI_NOT_INITIALIZED;

    if (flight.id != 0u ||
        (have_start && (uint32_t)(HAL_GetTick() - last_start_ms) < ui_config->frame_interval_ms))
        return UI_OK;

    unsigned selected = ui_config->field_count;
    if (state.phase == UI_PHASE_RUNNING)
    {
        for (unsigned n = 0u; n < ui_config->field_count; ++n)
        {
            unsigned candidate = (next_field + n) % ui_config->field_count;
            if (dirty[candidate] != 0u)
            {
                selected = candidate;
                break;
            }
        }
        if (selected == ui_config->field_count)
            return UI_OK;
    }

    saved = UIEnter();
    flight.generation = state.generation;
    flight.initializing = (uint8_t)(state.phase == UI_PHASE_INITIALIZING);
    unsigned flight_field = selected;
    if (flight.initializing && state.init_step > 0u)
        flight_field = state.init_step - 1u;
    flight.field = (uint8_t)flight_field;
    flight.version = flight_field < ui_config->field_count ? versions[flight_field] : 0u;
    UIExit(saved);
    UITxOptions_t options = {UICompleted, UICanStart, &flight, &flight.id};
    UIStatus_t result = flight.initializing ? BuildInit(state.init_step, &options)
                                             : BuildChange(selected, &options);
    if (result == UI_SUBMITTED && selected < ui_config->field_count)
        next_field = (uint8_t)((selected + 1u) % ui_config->field_count);
    else if (result != UI_SUBMITTED && result != UI_SEND_BUSY)
        UIPhase(UI_PHASE_ERROR, result);
    return result;
}

UIStatus_t UITask(void)
{
    return UIRefresh();
}
