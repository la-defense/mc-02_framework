/**
 * @file referee_task.c
 * @brief UI任务分步初始化与刷新；发送任务只回报结果，失败只停用UI，保留裁判接收。
 * @author kidneygood (you@domain.com)
 * @version 0.1
 * @date 2022-11-18
 * @copyright Copyright (c) 2022
 */
#include "referee_task.h"
#include "cmsis_os.h"
#include <math.h>
#include <limits.h>
#include <string.h>

static Referee_Interactive_info_t *Interactive_data;
static referee_id_t ui_identity;
static RefereeSnapshot_t ui_snapshot;
static UIState_t state;
static uint32_t versions[6], last_start_ms, demo_ms;
static double seen[6];
static uint8_t have_start, have_demo, next_group;
uint8_t UI_Seq;
typedef struct
{
    uint32_t id, generation, version;
    uint8_t group, initializing, event_ready;
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
static void Mark(unsigned group, uint8_t dirty)
{
    switch (group)
    {
    case 0:
        Interactive_data->Referee_Interactive_Flag.chassis_flag = dirty;
        break;
    case 1:
        Interactive_data->Referee_Interactive_Flag.gimbal_flag = dirty;
        break;
    case 2:
        Interactive_data->Referee_Interactive_Flag.shoot_flag = dirty;
        break;
    case 3:
        Interactive_data->Referee_Interactive_Flag.friction_flag = dirty;
        break;
    case 4:
        Interactive_data->Referee_Interactive_Flag.lid_flag = dirty;
        break;
    default:
        Interactive_data->Referee_Interactive_Flag.Power_flag = dirty;
        break;
    }
}
static uint8_t Dirty(unsigned group)
{
    switch (group)
    {
    case 0:
        return Interactive_data->Referee_Interactive_Flag.chassis_flag;
    case 1:
        return Interactive_data->Referee_Interactive_Flag.gimbal_flag;
    case 2:
        return Interactive_data->Referee_Interactive_Flag.shoot_flag;
    case 3:
        return Interactive_data->Referee_Interactive_Flag.friction_flag;
    case 4:
        return Interactive_data->Referee_Interactive_Flag.lid_flag;
    default:
        return Interactive_data->Referee_Interactive_Flag.Power_flag;
    }
}
// 同一显示项连续变化只保留最新内容；旧帧完成不能清掉较新版本的待刷新位。
static void Observe(void)
{
    double values[6] = {
        Interactive_data->chassis_mode, Interactive_data->gimbal_mode,
        Interactive_data->shoot_mode,   Interactive_data->friction_mode,
        Interactive_data->lid_mode,     Interactive_data->Chassis_Power_Data.chassis_power_mx};
    for (unsigned i = 0; i < 6u; ++i)
    {
        if (versions[i] == 0u || values[i] != seen[i])
        {
            seen[i] = values[i];
            if (++versions[i] == 0u)
                ++versions[i];
            Mark(i, 1u);
        }
    }
}
// 发送任务仅复制通知；UI任务下次推进时再修改流程和待刷新位。
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
    // 队列可能等待到身份过期；实际启动前再读本组期限，不能只信入队时的检查。
    return current && length >= 15u &&
           RefereeIdentityMatches((uint16_t)(data[9] | ((uint16_t)data[10] << 8u)));
}
UIStatus_t UITaskInit(Referee_Interactive_info_t *data)
{
    if (data == NULL)
        return UI_ERROR_ARGUMENT;
    if (flight.id != 0u)
        return UI_SEND_BUSY;
    uint32_t saved = UIEnter();
    Interactive_data = data;
    memset(&state, 0, sizeof(state));
    state.phase = UI_PHASE_NEED_INIT;
    memset(versions, 0, sizeof(versions));
    have_start = have_demo = next_group = 0u;
    UIExit(saved);
    return UI_OK;
}
UIStatus_t MyUIInit(void)
{
    if (Interactive_data == NULL)
        return UI_NOT_INITIALIZED;
    uint32_t saved = UIEnter();
    uint32_t old = flight.id;
    if (++state.generation == 0u)
        ++state.generation;
    state.phase = UI_PHASE_WAIT_ID;
    state.init_step = 0u;
    state.last_error = UI_OK;
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
#define UI_BUILD(call)                                                                                       \
    do                                                                                                       \
    {                                                                                                        \
        UIStatus_t r = (call);                                                                               \
        if (r != UI_OK)                                                                                      \
            return r;                                                                                        \
    } while (0)
// 每次只构造原初始化顺序中的一帧，15帧全部传输完成才进入运行阶段。
static UIStatus_t BuildInit(unsigned step, const UITxOptions_t *options)
{
    Graph_Data_t graph[5];
    String_Data_t text;
    const referee_id_t *id = &ui_identity;
    if (step == 0u)
        return UIDelete(id, UI_Data_Del_ALL, 0u, options);
    if (step == 1u)
    {
        UI_BUILD(UILineDraw(&graph[0], "sl0", UI_Graph_ADD, 7, UI_Color_White, 3, 710, 540, 1210, 540));
        UI_BUILD(UILineDraw(&graph[1], "sl1", UI_Graph_ADD, 7, UI_Color_White, 3, 960, 340, 960, 740));
        const uint32_t y[3] = {490, 515, 565};
        for (unsigned i = 0; i < 3u; ++i)
        {
            char name[3] = {'s', 'l', (char)('2' + i)};
            UI_BUILD(
                UILineDraw(&graph[i + 2u], name, UI_Graph_ADD, 7, UI_Color_Yellow, 2, 810, y[i], 1110, y[i]));
        }
        return UIGraphRefresh(id, graph, 5u, 5u, options);
    }
    if (step < 12u)
    {
        unsigned group = (step - 2u) / 2u;
        uint8_t dynamic = (uint8_t)((step - 2u) % 2u);
        static const char *const labels[] = {"chassis:", "gimbal:", "shoot:", "frict:", "lid:"};
        static const char *const defaults[] = {"zeroforce", "zeroforce", "off", "off", "open "};
        static const uint32_t colors[] = {UI_Color_Main, UI_Color_Yellow, UI_Color_Orange, UI_Color_Pink,
                                          UI_Color_Pink};
        char name[3] = {'s', dynamic ? 'd' : 's', (char)('0' + group)};
        UI_BUILD(UICharDraw(&text, name, UI_Graph_ADD, 8, colors[group], 15, 2, dynamic ? 270u : 150u,
                            750u - group * 50u, "%s", dynamic ? defaults[group] : labels[group]));
        return UICharRefresh(id, &text, options);
    }
    if (step == 12u)
    {
        UI_BUILD(UICharDraw(&text, "ss5", UI_Graph_ADD, 7, UI_Color_Green, 18, 2, 620, 230, "Power:"));
        return UICharRefresh(id, &text, options);
    }
    if (step == 13u)
    {
        UI_BUILD(UIRectangleDraw(&graph[0], "ss6", UI_Graph_ADD, 7, UI_Color_Green, 2, 720, 140, 1220, 180));
        return UIGraphRefresh(id, graph, 1u, 1u, options);
    }
    UI_BUILD(UIFloatDraw(&graph[0], "sd5", UI_Graph_ADD, 8, UI_Color_Green, 18, 2, 2, 750, 230, 24000));
    UI_BUILD(UILineDraw(&graph[1], "sd6", UI_Graph_ADD, 8, UI_Color_Pink, 30, 720, 160, 1020, 160));
    return UIGraphRefresh(id, graph, 2u, 2u, options);
}
// 功率显示先检查千倍整数和坐标范围，避免浮点转换丢失或越界。
static UIStatus_t BuildChange(unsigned group, const UITxOptions_t *options)
{
    if (group == 5u)
    {
        double milli = seen[5] * 1000.0, end_x = 750.0 + seen[5] * 30.0;
        if (!isfinite(milli) || milli < INT32_MIN || milli > INT32_MAX || !isfinite(end_x) || end_x < 0.0 ||
            end_x > 1920.0)
            return UI_ERROR_RANGE;
        Graph_Data_t graph[2];
        UI_BUILD(UIFloatDraw(&graph[0], "sd5", UI_Graph_Change, 8, UI_Color_Green, 18, 2, 2, 750, 230,
                             (int32_t)milli));
        UI_BUILD(UILineDraw(&graph[1], "sd6", UI_Graph_Change, 8, UI_Color_Pink, 30, 720, 160,
                            (uint32_t)end_x, 160));
        return UIGraphRefresh(&ui_identity, graph, 2u, 2u, options);
    }
    const char *value = NULL;
    if (group == 0u)
    {
        switch (Interactive_data->chassis_mode)
        {
        case CHASSIS_ZERO_FORCE:
            value = "zeroforce";
            break;
        case CHASSIS_ROTATE:
            value = "rotate   ";
            break;
        case CHASSIS_NO_FOLLOW:
            value = "nofollow ";
            break;
        case CHASSIS_FOLLOW_GIMBAL_YAW:
            value = "follow   ";
            break;
        default:
            return UI_ERROR_RANGE;
        }
    }
    else if (group == 1u)
    {
        switch (Interactive_data->gimbal_mode)
        {
        case GIMBAL_ZERO_FORCE:
            value = "zeroforce";
            break;
        case GIMBAL_FREE_MODE:
            value = "free     ";
            break;
        case GIMBAL_GYRO_MODE:
            value = "gyro     ";
            break;
        default:
            return UI_ERROR_RANGE;
        }
    }
    else if (group == 2u)
        value = Interactive_data->shoot_mode == SHOOT_ON ? "on " : "off";
    else if (group == 3u)
        value = Interactive_data->friction_mode == FRICTION_ON ? "on " : "off";
    else
        value = Interactive_data->lid_mode == LID_OPEN ? "open " : "close";
    char name[3] = {'s', 'd', (char)('0' + group)};
    String_Data_t text;
    uint32_t color = group == 0u   ? UI_Color_Main
                     : group == 1u ? UI_Color_Yellow
                     : group == 2u ? UI_Color_Pink
                                   : UI_Color_Orange;
    UI_BUILD(UICharDraw(&text, name, UI_Graph_Change, 8, color, 15, 2, 270, 750u - group * 50u, "%s", value));
    return UICharRefresh(&ui_identity, &text, options);
}
UIStatus_t UIRefresh(void)
{
    if (Interactive_data == NULL)
        return UI_NOT_INITIALIZED;
    Observe();
    uint32_t saved = UIEnter();
    uint8_t event_ready = flight.event_ready;
    USARTTxEvent_t event = flight.event;
    uint32_t generation = flight.generation;
    uint8_t initializing = flight.initializing, group = flight.group;
    uint32_t version = flight.version;
    if (event_ready)
    {
        flight.event_ready = 0u;
        flight.id = 0u;
    }
    UIExit(saved);
    // 重新初始化后仍等旧通知释放其上下文，但旧代结果不推进新代。
    if (event_ready)
    {
        if (event.started)
        {
            last_start_ms = event.started_ms;
            have_start = 1u;
        }
        if (generation == state.generation)
        {
            if (event.result != USART_TX_COMPLETE)
            {
                if (event.result == USART_TX_CANCELLED && !RefereeIdentityMatches(ui_identity.Robot_ID))
                    UIPhase(UI_PHASE_NEED_INIT, UI_NOT_INITIALIZED);
                else
                    UIPhase(UI_PHASE_ERROR,
                            event.result == USART_TX_EXPIRED ? UI_SEND_TIMEOUT : UI_SEND_ERROR);
            }
            else if (initializing)
            {
                saved = UIEnter();
                ++state.init_step;
                UIExit(saved);
                if (state.init_step == 15u)
                {
                    for (unsigned i = 0; i < 6u; ++i)
                        Mark(i, 1u);
                    UIPhase(UI_PHASE_RUNNING, UI_OK);
                }
            }
            else if (group < 6u && versions[group] == version)
                Mark(group, 0u);
        }
    }
    if (state.phase == UI_PHASE_NEED_INIT || state.phase == UI_PHASE_ERROR)
        return UI_NOT_INITIALIZED;
    if (state.phase == UI_PHASE_WAIT_ID)
    {
        if (RefereeGetSnapshot(&ui_snapshot) != REFEREE_OK ||
            ui_snapshot.command[REF_CMD_ROBOT_STATE].state != REFEREE_DATA_FRESH)
            return UI_OK;
        uint16_t robot = ui_snapshot.command[REF_CMD_ROBOT_STATE].data.robot_state.robot_id;
        ui_identity = (referee_id_t){.Robot_Color = robot >= 100u ? Robot_Blue : Robot_Red,
                                     .Robot_ID = robot,
                                     .Cilent_ID = (uint16_t)(0x100u + robot)};
        UIPhase(UI_PHASE_INITIALIZING, UI_OK);
    }
    if (!RefereeIdentityMatches(ui_identity.Robot_ID))
    {
        saved = UIEnter();
        if (++state.generation == 0u)
            ++state.generation;
        UIExit(saved);
        if (flight.id)
            (void)RefereeCancelSend(flight.id);
        UIPhase(UI_PHASE_NEED_INIT, UI_NOT_INITIALIZED);
        return UI_NOT_INITIALIZED;
    }
    // 以实际上线启动时刻计115ms；队列等待较久时也不追赶补发。
    if (flight.id || (have_start && (uint32_t)(HAL_GetTick() - last_start_ms) < 115u))
        return UI_OK;
    unsigned selected = 6u;
    if (state.phase == UI_PHASE_RUNNING)
    {
        for (unsigned n = 0; n < 6u; ++n)
        {
            unsigned candidate = (next_group + n) % 6u;
            if (Dirty(candidate))
            {
                selected = candidate;
                break;
            }
        }
        if (selected == 6u)
            return UI_OK;
    }
    saved = UIEnter();
    flight.generation = state.generation;
    flight.group = (uint8_t)selected;
    flight.initializing = (uint8_t)(state.phase == UI_PHASE_INITIALIZING);
    flight.version = selected < 6u ? versions[selected] : 0u;
    UIExit(saved);
    UITxOptions_t options = {UICompleted, UICanStart, &flight, &flight.id};
    UIStatus_t result =
        flight.initializing ? BuildInit(state.init_step, &options) : BuildChange(selected, &options);
    if (result == UI_SUBMITTED)
        next_group = (uint8_t)((selected + 1u) % 6u);
    else if (result != UI_SEND_BUSY)
        UIPhase(UI_PHASE_ERROR, result);
    return result;
}

// 每50个115ms逻辑步骤切换演示档位；这些值仍为演示数据。
static uint8_t count = 0;
static uint16_t count1 = 0;
static void RobotModeTest(Referee_Interactive_info_t *_Interactive_data) // 测试用函数，实现模式自动变化
{
    count++;
    if (count >= 50)
    {
        count = 0;
        count1++;
    }
    switch (count1 % 4)
    {
    case 0: {
        _Interactive_data->chassis_mode = CHASSIS_ZERO_FORCE;
        _Interactive_data->gimbal_mode = GIMBAL_ZERO_FORCE;
        _Interactive_data->shoot_mode = SHOOT_ON;
        _Interactive_data->friction_mode = FRICTION_ON;
        _Interactive_data->lid_mode = LID_OPEN;
        _Interactive_data->Chassis_Power_Data.chassis_power_mx += 3.5;
        if (_Interactive_data->Chassis_Power_Data.chassis_power_mx >= 18)
            _Interactive_data->Chassis_Power_Data.chassis_power_mx = 0;
        break;
    }
    case 1: {
        _Interactive_data->chassis_mode = CHASSIS_ROTATE;
        _Interactive_data->gimbal_mode = GIMBAL_FREE_MODE;
        _Interactive_data->shoot_mode = SHOOT_OFF;
        _Interactive_data->friction_mode = FRICTION_OFF;
        _Interactive_data->lid_mode = LID_CLOSE;
        break;
    }
    case 2: {
        _Interactive_data->chassis_mode = CHASSIS_NO_FOLLOW;
        _Interactive_data->gimbal_mode = GIMBAL_GYRO_MODE;
        _Interactive_data->shoot_mode = SHOOT_ON;
        _Interactive_data->friction_mode = FRICTION_ON;
        _Interactive_data->lid_mode = LID_OPEN;
        break;
    }
    case 3: {
        _Interactive_data->chassis_mode = CHASSIS_FOLLOW_GIMBAL_YAW;
        _Interactive_data->gimbal_mode = GIMBAL_ZERO_FORCE;
        _Interactive_data->shoot_mode = SHOOT_OFF;
        _Interactive_data->friction_mode = FRICTION_OFF;
        _Interactive_data->lid_mode = LID_CLOSE;
        break;
    }
    default:
        break;
    }
}

UIStatus_t UITask(void)
{
    // 发送每1ms轮询，演示只按115ms逻辑步长推进，不追赶积压的演示步骤。
    if (Interactive_data != NULL && state.phase == UI_PHASE_RUNNING)
    {
        uint32_t now = HAL_GetTick();
        if (!have_demo)
        {
            have_demo = 1u;
            demo_ms = now;
        }
        else if ((uint32_t)(now - demo_ms) >= 115u)
        {
            demo_ms = now;
            RobotModeTest(Interactive_data);
        }
    }
    return UIRefresh();
}
