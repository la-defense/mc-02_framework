/**
 * @file referee_task.c
 * @brief 单任务准备UI并检查每次构图及发送；初始化失败只停用UI，保留裁判接收。
 * @author kidneygood (you@domain.com)
 * @version 0.1
 * @date 2022-11-18
 * @copyright Copyright (c) 2022
 */
#include "referee_task.h"
#include "referee_UI.h"
#include "cmsis_os.h"
#include <math.h>
#include <limits.h>

static Referee_Interactive_info_t *Interactive_data;
static referee_id_t ui_identity;
static RefereeSnapshot_t ui_snapshot;
static uint8_t ui_identity_valid;
static uint8_t ui_ready;
uint8_t UI_Seq;
static void RobotModeTest(Referee_Interactive_info_t *data);
static void UIChangeCheck(Referee_Interactive_info_t *data);
static UIStatus_t MyUIRefresh(Referee_Interactive_info_t *data);

static Graph_Data_t UI_shoot_line[10];
static Graph_Data_t UI_Energy[3];
static String_Data_t UI_State_sta[6], UI_State_dyn[6];
static const uint32_t shoot_line_location[5] = {540, 960, 490, 515, 565};

static UIStatus_t UIIdentityCheck(void)
{
    if (!ui_identity_valid || RefereeGetSnapshot(&ui_snapshot) != REFEREE_OK ||
        ui_snapshot.command[REF_CMD_ROBOT_STATE].state != REFEREE_DATA_FRESH ||
        ui_snapshot.command[REF_CMD_ROBOT_STATE].data.robot_state.robot_id != ui_identity.Robot_ID) {
        ui_ready = 0u;
        ui_identity_valid = 0u;
        return UI_NOT_INITIALIZED;
    }
    return UI_OK;
}

UIStatus_t UITaskInit(Referee_Interactive_info_t *data)
{
    if (data == NULL) return UI_ERROR_ARGUMENT;
    Interactive_data = data;
    ui_ready = 0u;
    ui_identity_valid = 0u;
    return UI_OK;
}

UIStatus_t UITask(void)
{
    if (!ui_ready || Interactive_data == NULL) return UI_NOT_INITIALIZED;
    RobotModeTest(Interactive_data);
    return UIRefresh();
}

UIStatus_t UIRefresh(void)
{
    if (!ui_ready || Interactive_data == NULL) return UI_NOT_INITIALIZED;
    UIStatus_t identity_status = UIIdentityCheck();
    if (identity_status != UI_OK) return identity_status;
    return MyUIRefresh(Interactive_data);
}

// 初始化是一轮显式尝试；任一步失败不置就绪，重试须由调用者再次调用MyUIInit。
#define UI_BUILD(call) do { UIStatus_t result = (call); if (result != UI_OK) return result; } while (0)
#define UI_SUBMIT(call) do { UIStatus_t identity_status = UIIdentityCheck(); if (identity_status != UI_OK) return identity_status; \
    UIStatus_t result = (call); if (result != UI_SUBMITTED) return result; } while (0)
UIStatus_t MyUIInit(void)
{
    ui_ready = 0u;
    ui_identity_valid = 0u;
    if (Interactive_data == NULL) return UI_NOT_INITIALIZED;
    if (RefereeGetSnapshot(&ui_snapshot) != REFEREE_OK ||
        ui_snapshot.command[REF_CMD_ROBOT_STATE].state != REFEREE_DATA_FRESH) return UI_NOT_INITIALIZED;
    uint16_t robot = ui_snapshot.command[REF_CMD_ROBOT_STATE].data.robot_state.robot_id;
    if (robot == 0u) return UI_NOT_INITIALIZED;
    ui_identity.Robot_Color = robot >= 100u ? Robot_Blue : Robot_Red;
    ui_identity.Robot_ID = robot;
    ui_identity.Cilent_ID = (uint16_t)(0x100u + robot);
    ui_identity.Receiver_Robot_ID = 0u;
    ui_identity_valid = 1u;
    const referee_id_t *id = &ui_identity;
    UI_SUBMIT(UIDelete(id, UI_Data_Del_ALL, 0));
    UI_BUILD(UILineDraw(&UI_shoot_line[0], "sl0", UI_Graph_ADD, 7, UI_Color_White, 3, 710, shoot_line_location[0], 1210, shoot_line_location[0]));
    UI_BUILD(UILineDraw(&UI_shoot_line[1], "sl1", UI_Graph_ADD, 7, UI_Color_White, 3, shoot_line_location[1], 340, shoot_line_location[1], 740));
    for (size_t i = 2; i < 5; ++i)
    {
        const char name[3] = {'s','l',(char)('0'+i)};
        UI_BUILD(UILineDraw(&UI_shoot_line[i], name, UI_Graph_ADD, 7, UI_Color_Yellow, 2, 810, shoot_line_location[i], 1110, shoot_line_location[i]));
    }
    UI_SUBMIT(UIGraphRefresh(id, UI_shoot_line, 10u, 5u));
    static const char *const labels[5] = {"chassis:", "gimbal:", "shoot:", "frict:", "lid:"};
    static const char *const defaults[5] = {"zeroforce", "zeroforce", "off", "off", "open "};
    static const uint32_t colors[5] = {UI_Color_Main, UI_Color_Yellow, UI_Color_Orange, UI_Color_Pink, UI_Color_Pink};
    // 固定布局沿用原位置及颜色；每一项只有构图成功才提交。
    for (size_t i = 0; i < 5; ++i)
    {
        const char name[3] = {'s','s',(char)('0'+i)};
        const char dynamic_name[3] = {'s','d',(char)('0'+i)};
        uint32_t y = 750u - (uint32_t)i*50u;
        UI_BUILD(UICharDraw(&UI_State_sta[i], name, UI_Graph_ADD, 8, colors[i], 15, 2, 150, y, "%s", labels[i]));
        UI_SUBMIT(UICharRefresh(id, &UI_State_sta[i]));
        UI_BUILD(UICharDraw(&UI_State_dyn[i], dynamic_name, UI_Graph_ADD, 8, colors[i], 15, 2, 270, y, "%s", defaults[i]));
        UI_SUBMIT(UICharRefresh(id, &UI_State_dyn[i]));
    }
    UI_BUILD(UICharDraw(&UI_State_sta[5], "ss5", UI_Graph_ADD, 7, UI_Color_Green, 18, 2, 620, 230, "Power:"));
    UI_SUBMIT(UICharRefresh(id, &UI_State_sta[5]));
    UI_BUILD(UIRectangleDraw(&UI_Energy[0], "ss6", UI_Graph_ADD, 7, UI_Color_Green, 2, 720, 140, 1220, 180));
    UI_SUBMIT(UIGraphRefresh(id, UI_Energy, 3u, 1u));
    UI_BUILD(UIFloatDraw(&UI_Energy[1], "sd5", UI_Graph_ADD, 8, UI_Color_Green, 18, 2, 2, 750, 230, 24000));
    UI_BUILD(UILineDraw(&UI_Energy[2], "sd6", UI_Graph_ADD, 8, UI_Color_Pink, 30, 720, 160, 1020, 160));
    UI_SUBMIT(UIGraphRefresh(id, &UI_Energy[1], 2u, 2u));
    // 固定初始图之后重新提交当前状态，避免显式重初始化被上轮last_mode掩盖。
    Interactive_data->Referee_Interactive_Flag.chassis_flag = 1u;
    Interactive_data->Referee_Interactive_Flag.gimbal_flag = 1u;
    Interactive_data->Referee_Interactive_Flag.shoot_flag = 1u;
    Interactive_data->Referee_Interactive_Flag.friction_flag = 1u;
    Interactive_data->Referee_Interactive_Flag.lid_flag = 1u;
    Interactive_data->Referee_Interactive_Flag.Power_flag = 1u;
    ui_ready = 1u;
    return UI_SUBMITTED;
}

// 每50次调用切换一次演示状态，方便观察 UI 是否随状态更新。
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
    case 0:
    {
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
    case 1:
    {
        _Interactive_data->chassis_mode = CHASSIS_ROTATE;
        _Interactive_data->gimbal_mode = GIMBAL_FREE_MODE;
        _Interactive_data->shoot_mode = SHOOT_OFF;
        _Interactive_data->friction_mode = FRICTION_OFF;
        _Interactive_data->lid_mode = LID_CLOSE;
        break;
    }
    case 2:
    {
        _Interactive_data->chassis_mode = CHASSIS_NO_FOLLOW;
        _Interactive_data->gimbal_mode = GIMBAL_GYRO_MODE;
        _Interactive_data->shoot_mode = SHOOT_ON;
        _Interactive_data->friction_mode = FRICTION_ON;
        _Interactive_data->lid_mode = LID_OPEN;
        break;
    }
    case 3:
    {
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

// 只在本机接受完整刷新后清标志；失败留到下一轮，不把旧对象当成新构图结果发送。
static UIStatus_t MyUIRefresh(Referee_Interactive_info_t *data)
{
    UIChangeCheck(data);
    const referee_id_t *id = &ui_identity;
    if (data->Referee_Interactive_Flag.chassis_flag)
    {
        const char *text;
        switch (data->chassis_mode)
        {
        case CHASSIS_ZERO_FORCE: text = "zeroforce"; break;
        case CHASSIS_ROTATE: text = "rotate   "; break;
        case CHASSIS_NO_FOLLOW: text = "nofollow "; break;
        case CHASSIS_FOLLOW_GIMBAL_YAW: text = "follow   "; break;
        default: return UI_ERROR_RANGE;
        }
        UI_BUILD(UICharDraw(&UI_State_dyn[0], "sd0", UI_Graph_Change, 8, UI_Color_Main, 15, 2, 270, 750, "%s", text));
        UI_SUBMIT(UICharRefresh(id, &UI_State_dyn[0]));
        data->Referee_Interactive_Flag.chassis_flag = 0u;
    }
    if (data->Referee_Interactive_Flag.gimbal_flag)
    {
        const char *text;
        switch (data->gimbal_mode)
        {
        case GIMBAL_ZERO_FORCE: text = "zeroforce"; break;
        case GIMBAL_FREE_MODE: text = "free     "; break;
        case GIMBAL_GYRO_MODE: text = "gyro     "; break;
        default: return UI_ERROR_RANGE;
        }
        UI_BUILD(UICharDraw(&UI_State_dyn[1], "sd1", UI_Graph_Change, 8, UI_Color_Yellow, 15, 2, 270, 700, "%s", text));
        UI_SUBMIT(UICharRefresh(id, &UI_State_dyn[1]));
        data->Referee_Interactive_Flag.gimbal_flag = 0u;
    }
    if (data->Referee_Interactive_Flag.shoot_flag)
    {
        UI_BUILD(UICharDraw(&UI_State_dyn[2], "sd2", UI_Graph_Change, 8, UI_Color_Pink, 15, 2, 270, 650, data->shoot_mode == SHOOT_ON ? "on " : "off"));
        UI_SUBMIT(UICharRefresh(id, &UI_State_dyn[2]));
        data->Referee_Interactive_Flag.shoot_flag = 0u;
    }
    if (data->Referee_Interactive_Flag.friction_flag)
    {
        UI_BUILD(UICharDraw(&UI_State_dyn[3], "sd3", UI_Graph_Change, 8, UI_Color_Pink, 15, 2, 270, 600, data->friction_mode == FRICTION_ON ? "on " : "off"));
        UI_SUBMIT(UICharRefresh(id, &UI_State_dyn[3]));
        data->Referee_Interactive_Flag.friction_flag = 0u;
    }
    if (data->Referee_Interactive_Flag.lid_flag)
    {
        UI_BUILD(UICharDraw(&UI_State_dyn[4], "sd4", UI_Graph_Change, 8, UI_Color_Pink, 15, 2, 270, 550, data->lid_mode == LID_OPEN ? "open " : "close"));
        UI_SUBMIT(UICharRefresh(id, &UI_State_dyn[4]));
        data->Referee_Interactive_Flag.lid_flag = 0u;
    }
    if (data->Referee_Interactive_Flag.Power_flag)
    {
        double watts = data->Chassis_Power_Data.chassis_power_mx;
        double milli = watts*1000.0, end_x = 750.0+watts*30.0;
        // 先在double中检查再转整数，NaN、过大或负坐标不会进入C的未定义转换。
        if (!isfinite(milli) || milli < INT32_MIN || milli > INT32_MAX ||
            !isfinite(end_x) || end_x < 0.0 || end_x > 1920.0) return UI_ERROR_RANGE;
        Graph_Data_t power[2];
        UI_BUILD(UIFloatDraw(&power[0], "sd5", UI_Graph_Change, 8, UI_Color_Green, 18, 2, 2, 750, 230, (int32_t)milli));
        UI_BUILD(UILineDraw(&power[1], "sd6", UI_Graph_Change, 8, UI_Color_Pink, 30, 720, 160, (uint32_t)end_x, 160));
        UI_SUBMIT(UIGraphRefresh(id, power, 2u, 2u));
        data->Referee_Interactive_Flag.Power_flag = 0u;
    }
    return UI_OK;
}

static void UIChangeCheck(Referee_Interactive_info_t *_Interactive_data)
{
    if (_Interactive_data->chassis_mode != _Interactive_data->chassis_last_mode)
    {
        _Interactive_data->Referee_Interactive_Flag.chassis_flag = 1;
        _Interactive_data->chassis_last_mode = _Interactive_data->chassis_mode;
    }

    if (_Interactive_data->gimbal_mode != _Interactive_data->gimbal_last_mode)
    {
        _Interactive_data->Referee_Interactive_Flag.gimbal_flag = 1;
        _Interactive_data->gimbal_last_mode = _Interactive_data->gimbal_mode;
    }

    if (_Interactive_data->shoot_mode != _Interactive_data->shoot_last_mode)
    {
        _Interactive_data->Referee_Interactive_Flag.shoot_flag = 1;
        _Interactive_data->shoot_last_mode = _Interactive_data->shoot_mode;
    }

    if (_Interactive_data->friction_mode != _Interactive_data->friction_last_mode)
    {
        _Interactive_data->Referee_Interactive_Flag.friction_flag = 1;
        _Interactive_data->friction_last_mode = _Interactive_data->friction_mode;
    }

    if (_Interactive_data->lid_mode != _Interactive_data->lid_last_mode)
    {
        _Interactive_data->Referee_Interactive_Flag.lid_flag = 1;
        _Interactive_data->lid_last_mode = _Interactive_data->lid_mode;
    }

    if (_Interactive_data->Chassis_Power_Data.chassis_power_mx != _Interactive_data->Chassis_last_Power_Data.chassis_power_mx)
    {
        _Interactive_data->Referee_Interactive_Flag.Power_flag = 1;
        _Interactive_data->Chassis_last_Power_Data.chassis_power_mx = _Interactive_data->Chassis_Power_Data.chassis_power_mx;
    }
}
