/** @file lcd_ui.c
* @brief 组合系统、IMU、功率和电机四页信息，通过 lcd_buf.c 绘制。
* 标签与标题按页重画，值区保留上次内容，相同内容跳过；页面被覆盖后须清除记录。
 */
#include "lcd_ui.h"
#include "config/build_selection.h"
#include "lcd.h"
#include "lcd_buf.h"
#include "lcd_font_cn.h"
#include "robot_safety.h"
#include "bsp_adc.h"
#include "bsp_watchdog.h"
#include "task_monitor.h"
#include "remote_control.h"
#include "master_process.h"
#if MC02_FEATURE_HEATER
#include "imu_heater.h"
#endif
#include "ins_task.h"
#include "referee_task.h"
#include "rm_referee.h"
#include "lcd_referee_rows.h"
#include "lcd_internal.h"
#if defined(ONE_BOARD) || defined(CHASSIS_BOARD)
#include "chassis.h"
#endif
#if defined(ONE_BOARD) || defined(GIMBAL_BOARD)
#include "gimbal.h"
#include "shoot.h"
#endif
#include "bsp_can.h"
#include "fdcan.h"
#include "bmi088.h"
#include "bsp_crash.h"
#include <stdio.h>
#include <string.h>
#include <limits.h>

#define LCD_CN_LENGTH(array) (sizeof(array) / sizeof((array)[0]) - 1u)
#define LCD_ASCII_LENGTH(text) (sizeof(text) - 1u)
#define LCD_TEXT_MAX 48u

/* 按键诊断(定义在 lcd_task.c): 上电以来见过的最小原始 ADC 值 */
extern volatile uint16_t lcd_key_raw_min;

#define COL_LABEL_X 4u
#define COL_VALUE_X 80u
#define ROW_Y(n) (28u + (uint16_t)(n) * 20u)

#if !defined(MC02_PROFILE_BENCH_SAFE)
static const Lcd_CnChar_e L_SYS[] = LCD_LABEL_SYS;
#endif
static const Lcd_CnChar_e L_STATE[] = LCD_LABEL_STATE;
static const Lcd_CnChar_e L_FAULT[] = LCD_LABEL_FAULT;
static const Lcd_CnChar_e L_ESTOP[] = LCD_LABEL_ESTOP;
static const Lcd_CnChar_e L_VOLT[] = LCD_LABEL_VOLT;
static const Lcd_CnChar_e L_TASK[] = LCD_LABEL_TASK;
static const Lcd_CnChar_e L_RC[] = LCD_LABEL_RC;
static const Lcd_CnChar_e L_VISION[] = LCD_LABEL_VISION;
static const Lcd_CnChar_e L_WATCHDOG[] = LCD_LABEL_WATCHDOG;
static const Lcd_CnChar_e L_TEMP[] = LCD_LABEL_TEMP;
static const Lcd_CnChar_e L_TARGET[] = LCD_LABEL_TARGET;
static const Lcd_CnChar_e L_DUTY[] = LCD_LABEL_DUTY;
static const Lcd_CnChar_e L_HEAT[] = LCD_LABEL_HEAT;
static const Lcd_CnChar_e L_VALID[] = LCD_LABEL_VALID;
#if MC02_FEATURE_HEATER
static const Lcd_CnChar_e L_INVALID[] = {LCD_CN_WU2, LCD_CN_XIAO, LCD_CN_COUNT};
#endif
static const Lcd_CnChar_e L_REFEREE[] = LCD_LABEL_REFEREE;
static const Lcd_CnChar_e L_STAGE[] = LCD_LABEL_STAGE;
static const Lcd_CnChar_e L_HP[] = LCD_LABEL_HP;
static const Lcd_CnChar_e L_POWER[] = LCD_LABEL_POWER;
static const Lcd_CnChar_e L_BUFFER[] = LCD_LABEL_BUFFER;
static const Lcd_CnChar_e L_HEATQ[] = LCD_LABEL_HEATQ;
static const Lcd_CnChar_e L_AMMO[] = LCD_LABEL_AMMO;
static const Lcd_CnChar_e L_BULLET_SPEED[] = LCD_LABEL_BULLET_SPEED;
static const Lcd_CnChar_e L_COIN[] = LCD_LABEL_COIN;
static const Lcd_CnChar_e L_CHASSIS[] = LCD_LABEL_CHASSIS;
static const Lcd_CnChar_e L_GIMBAL[] = LCD_LABEL_GIMBAL;
static const Lcd_CnChar_e L_SHOOT[] = LCD_LABEL_SHOOT;
static const Lcd_CnChar_e L_MOTOR[] = LCD_LABEL_MOTOR;
static const Lcd_CnChar_e L_BUS[] = LCD_LABEL_BUS;
static const Lcd_CnChar_e L_LINK[] = LCD_LABEL_LINK;
static const Lcd_CnChar_e L_ONLINE[] = LCD_LABEL_ONLINE;
static const Lcd_CnChar_e L_OFFLINE[] = LCD_LABEL_OFFLINE;
static const Lcd_CnChar_e L_FREEZE[] = LCD_LABEL_FREEZE;
static const Lcd_CnChar_e L_INIT[] = LCD_LABEL_INIT;
static const Lcd_CnChar_e L_SAFE[] = LCD_LABEL_SAFE;
static const Lcd_CnChar_e L_CALIB[] = LCD_LABEL_CALIB;
static const Lcd_CnChar_e L_READY[] = LCD_LABEL_READY;
static const Lcd_CnChar_e L_ERROR[] = LCD_LABEL_ERROR;
static const Lcd_CnChar_e L_NO[] = LCD_LABEL_NO;
static const Lcd_CnChar_e L_CAL_RUN[] = LCD_LABEL_CALIB_RUN;
static const Lcd_CnChar_e L_CAL_HOLD[] = LCD_LABEL_CALIB_HOLD;
static const Lcd_CnChar_e L_CAL_OK[] = LCD_LABEL_CALIB_OK;
static const Lcd_CnChar_e L_CAL_FAIL[] = LCD_LABEL_CALIB_FAIL;

static uint8_t lcd_page = 0;
static uint8_t lcd_frozen = 0;

/* 初始化阶段标记(排查用): 0=未开始 1=LCD_Init 2=整屏清屏 3=填充 4=静态绘制 5=数值刷新 6=完成
   启动卡死时读它就能知道卡在哪一步, 不用靠猜。 */
volatile uint8_t lcd_init_stage = 0;

// 将整车状态换成标签；未知状态显示“否”，不改变整车状态。
static const Lcd_CnChar_e *state_label(Robot_Status_e state, size_t *count)
{
    switch (state)
    {
    case ROBOT_INIT:
        *count = LCD_CN_LENGTH(L_INIT);
        return L_INIT;
    case ROBOT_SAFE:
        *count = LCD_CN_LENGTH(L_SAFE);
        return L_SAFE;
    case ROBOT_CALIB:
        *count = LCD_CN_LENGTH(L_CALIB);
        return L_CALIB;
    case ROBOT_READY:
        *count = LCD_CN_LENGTH(L_READY);
        return L_READY;
    case ROBOT_FAULT:
        *count = LCD_CN_LENGTH(L_ERROR);
        return L_ERROR;
    case ROBOT_ESTOP:
        *count = LCD_CN_LENGTH(L_ESTOP);
        return L_ESTOP;
    default:
        *count = LCD_CN_LENGTH(L_NO);
        return L_NO;
    }
}

static const Lcd_CnChar_e *online_label(uint8_t online)
{
    return online ? L_ONLINE : L_OFFLINE;
}

// 把 CAN 状态换成短文字，未识别的状态显示 --。
static const char *can_status_str(CAN_Status_e status)
{
    switch (status)
    {
    case CAN_STATUS_OK:
        return "OK";
    case CAN_STATUS_ERROR:
        return "ERR";
    case CAN_STATUS_BUSOFF:
        return "OFF";
    default:
        return "--";
    }
}

/* 界面文字来自常量或本地定长数组；最多读到 LCD_TEXT_MAX 字节。 */
static size_t lcd_ascii_length(const char *text)
{
    size_t length = 0u;
    if (text == NULL)
        return 0u;
    while (length < LCD_TEXT_MAX && text[length] != '\0')
        ++length;
    return length;
}

// 用底色重画标题栏，同时显示页号和冻结提示，不清理下面的值区。
static void draw_title(const Lcd_CnChar_e *title_cn, size_t title_count, const char *title_ascii)
{
    LCD_BufFill(0, 0, LCD_W - 1u, 23u, BLUE);
    if (title_cn != NULL)
        LCD_BufShowCnString(4, 4, title_cn, title_count, WHITE, BLUE);
    else
        LCD_BufShowAscii(4, 4, title_ascii, lcd_ascii_length(title_ascii), WHITE, BLUE, 16);

    char page_buf[8];
    snprintf(page_buf, sizeof(page_buf), "%u/%u", (unsigned)(lcd_page + 1u), (unsigned)LCD_PAGE_COUNT);
    LCD_BufShowAscii((uint16_t)(LCD_W - 28u), 4, page_buf, lcd_ascii_length(page_buf), CYAN, BLUE, 16);

    if (lcd_frozen)
        LCD_BufShowCnString((uint16_t)(LCD_W - 68u), 4, L_FREEZE, LCD_CN_LENGTH(L_FREEZE), YELLOW, BLUE);
}

// 按页号选择标题；独立重画可让冻结提示变化时只更新顶部。
static void draw_page_title(uint8_t page)
{
    switch (page)
    {
    case 0u:
#if defined(MC02_PROFILE_BENCH_SAFE)
        draw_title(NULL, 0u, "SAFE BENCH");
#else
        draw_title(L_SYS, LCD_CN_LENGTH(L_SYS), NULL);
#endif
        break;
    case 1u:
        draw_title(NULL, 0u, "IMU");
        break;
    case 2u:
        draw_title(L_POWER, LCD_CN_LENGTH(L_POWER), NULL);
        break;
    default:
        draw_title(L_MOTOR, LCD_CN_LENGTH(L_MOTOR), NULL);
        break;
    }
}

// 在固定标签列绘制一行中文标签，row 从 0 开始。
static void draw_label(uint8_t row, const Lcd_CnChar_e *label, size_t count)
{
    LCD_BufShowCnString(COL_LABEL_X, ROW_Y(row), label, count, WHITE, BLACK);
}

// 在固定标签列绘制一行 ASCII 标签。
static void draw_label_ascii(uint8_t row, const char *label)
{
    LCD_BufShowAscii(COL_LABEL_X, ROW_Y(row), label, lcd_ascii_length(label), WHITE, BLACK, 16);
}

// 只清指定行的值区，覆盖旧文本可能留下的较长尾部。
static LCDStatus_t clear_value(uint8_t row)
{
    return LCD_BufFill(COL_VALUE_X, ROW_Y(row), LCD_W - 1u, ROW_Y(row) + 15u, BLACK);
}

// 按“页、行”保存最后绘制的内容；相同内容无需再次清背景和发送像素。
// 整页内容被覆盖后要清除这些记录，避免下次把已消失的值误当成还在屏上。
#define LCD_ROW_COUNT 9u
/* 行内容类型: 0=空(没画过) 1=ASCII 2=中文标签 3=浮点+单位 */
#define VALUE_KIND_NONE 0u
#define VALUE_KIND_ASCII 1u
#define VALUE_KIND_CN 2u
#define VALUE_KIND_FLOAT 3u

static char s_last_text[LCD_PAGE_COUNT][LCD_ROW_COUNT][LCD_TEXT_MAX];
static const Lcd_CnChar_e *s_last_cn[LCD_PAGE_COUNT][LCD_ROW_COUNT];
static uint8_t s_last_decimals[LCD_PAGE_COUNT][LCD_ROW_COUNT];
static uint8_t s_last_kind[LCD_PAGE_COUNT][LCD_ROW_COUNT];

volatile uint32_t lcd_rows_drawn = 0;   /* 实际重绘的行数(累计) */
volatile uint32_t lcd_rows_skipped = 0; /* 因内容未变而跳过的行数(累计) */

/* 内容与上次相同 → 1(可以整行跳过) */
static uint8_t value_unchanged(uint8_t row, uint8_t kind, const void *data, uint8_t decimals)
{
    if (row >= LCD_ROW_COUNT || lcd_page >= LCD_PAGE_COUNT)
        return 0u;
    if (s_last_kind[lcd_page][row] != kind)
        return 0u;

    switch (kind)
    {
    case VALUE_KIND_ASCII:
        return (strcmp(s_last_text[lcd_page][row], (const char *)data) == 0) ? 1u : 0u;
    case VALUE_KIND_CN:
        return (s_last_cn[lcd_page][row] == (const Lcd_CnChar_e *)data) ? 1u : 0u;
    case VALUE_KIND_FLOAT:
        return ((strcmp(s_last_text[lcd_page][row], (const char *)data) == 0) &&
                (s_last_decimals[lcd_page][row] == decimals)) ? 1u : 0u;
    default:
        return 0u;
    }
}

/* 记录"这一行刚画了什么", 供下次比较 */
static void value_mark_drawn(uint8_t row, uint8_t kind, const void *data, uint8_t decimals)
{
    if (row >= LCD_ROW_COUNT || lcd_page >= LCD_PAGE_COUNT)
        return;

    s_last_kind[lcd_page][row] = kind;
    switch (kind)
    {
    case VALUE_KIND_ASCII:
    case VALUE_KIND_FLOAT:
    {
        char *destination = s_last_text[lcd_page][row];
        const char *source = (const char *)data;
        size_t length = 0u;
        while (length < LCD_TEXT_MAX - 1u && source[length] != '\0')
            ++length;
        memcpy(destination, source, length);
        memset(destination + length, '\0', LCD_TEXT_MAX - length);
        s_last_decimals[lcd_page][row] = decimals;
        break;
    }
    case VALUE_KIND_CN:
        s_last_cn[lcd_page][row] = (const Lcd_CnChar_e *)data;
        break;
    default:
        break;
    }
    lcd_rows_drawn++;
}

void LCD_UI_InvalidateValueCache(void)
{
    memset(s_last_kind, 0, sizeof(s_last_kind));
}

static LCDStatus_t lcd_ui_begin(uint32_t budget_ms, uint8_t recovery)
{
    LCDStatus_t status = LCD_InternalBegin(budget_ms, recovery);
    if (status != LCD_OK)
        LCD_UI_InvalidateValueCache();
    return status;
}

static LCDStatus_t lcd_ui_end(LCDStatus_t status)
{
    status = LCD_InternalEnd(status);
    if (status != LCD_OK)
        LCD_UI_InvalidateValueCache();
    return status;
}

static LCDStatus_t draw_value_ascii(uint8_t row, const char *str)
{
    if (row >= LCD_ROW_COUNT)
        return LCD_OUT_OF_RANGE;
    // 文本过长先改用 12 像素字高，仍放不下时只保留能显示的部分并加 ~。
    if (str == NULL)
        return LCD_BAD_ARGUMENT;
    size_t len = lcd_ascii_length(str);
    if (len == LCD_TEXT_MAX)
        return LCD_BAD_TEXT;
    for (size_t i = 0u; i < len; ++i)
    {
        if ((uint8_t)str[i] < 0x20u || (uint8_t)str[i] > 0x7Eu)
            return LCD_BAD_TEXT;
    }
    uint16_t avail = (uint16_t)(LCD_W - COL_VALUE_X);
    uint8_t sizey = 16u;
    uint8_t char_w = 8u;
    if (len * (size_t)char_w > (size_t)avail)
    {
        sizey = 12u;
        char_w = 6u;
    }

    char buf[48];
    size_t max_chars = (size_t)(avail / char_w);
    if (max_chars > sizeof(buf) - 2u)
        max_chars = sizeof(buf) - 2u;

    if (len > max_chars)
    {
        size_t keep = (max_chars >= 2u) ? (max_chars - 1u) : max_chars;
        memcpy(buf, str, keep);
        buf[keep] = '~';
        buf[keep + 1u] = '\0';
    }
    else
    {
        memcpy(buf, str, len);
        buf[len] = '\0';
    }

    // 比较最终显示的文本；前面的缩短处理可能让不同输入得到相同画面。
    if (value_unchanged(row, VALUE_KIND_ASCII, buf, 0u))
    {
        lcd_rows_skipped++;
        return LCD_OK;
    }

    LCDStatus_t status = clear_value(row);
    if (status != LCD_OK)
        return status;
    size_t display_length = lcd_ascii_length(buf);
    status = LCD_BufShowAscii(COL_VALUE_X, ROW_Y(row), buf, display_length, CYAN, BLACK, sizey);
    if (status != LCD_OK)
        return status;
    value_mark_drawn(row, VALUE_KIND_ASCII, buf, 0u);
    return LCD_OK;
}

static LCDStatus_t draw_value_cn(uint8_t row, const Lcd_CnChar_e *str, size_t count)
{
    if (row >= LCD_ROW_COUNT)
        return LCD_OUT_OF_RANGE;
    if (count > 0u && str == NULL)
        return LCD_BAD_ARGUMENT;
    if (count > (LCD_W - COL_VALUE_X) / 16u)
        return LCD_OUT_OF_RANGE;
    for (size_t i = 0u; i < count; ++i)
    {
        if ((unsigned)str[i] >= (unsigned)LCD_CN_COUNT)
            return LCD_BAD_GLYPH;
    }

    /* 中文值都是标签数组(如"在线"/"离线"), 比指针就够 —— 同一内容每次传的是同一个数组 */
    if (value_unchanged(row, VALUE_KIND_CN, str, 0u))
    {
        lcd_rows_skipped++;
        return LCD_OK;
    }

    LCDStatus_t status = clear_value(row);
    if (status != LCD_OK)
        return status;
    status = LCD_BufShowCnString(COL_VALUE_X, ROW_Y(row), str, count, CYAN, BLACK);
    if (status != LCD_OK)
        return status;
    value_mark_drawn(row, VALUE_KIND_CN, str, 0u);
    return LCD_OK;
}

static LCDStatus_t draw_value_float_unit(uint8_t row, float value, uint8_t decimals, const char *unit)
{
    if (row >= LCD_ROW_COUNT)
        return LCD_OUT_OF_RANGE;
    if (unit == NULL)
        return LCD_BAD_ARGUMENT;
    char text[16];
    size_t text_length = 0u;
    LCDStatus_t format_status = LCD_BufFormatFloat(value, decimals, text, sizeof(text), &text_length);
    if (format_status != LCD_OK)
    {
        memcpy(text, "--", 3u);
        text_length = 2u;
    }
    size_t unit_length = lcd_ascii_length(unit);
    size_t unit_capacity = (size_t)(LCD_W - COL_VALUE_X - 8u * 8u) / 8u;
    if (unit_length == LCD_TEXT_MAX)
        return LCD_BAD_TEXT;
    for (size_t i = 0u; i < unit_length; ++i)
    {
        if ((uint8_t)unit[i] < 0x20u || (uint8_t)unit[i] > 0x7Eu)
            return LCD_BAD_TEXT;
    }
    // 数值固定占八个字宽，先确认单位及最终文本都放得下，再清理旧值。
    if (text_length > 8u || unit_length > unit_capacity)
    {
        memcpy(text, "--", 3u);
        text_length = 2u;
    }

    // 缓存比较最终显示的文字；多个无效数值都会显示同一个占位符。
    if (value_unchanged(row, VALUE_KIND_FLOAT, text, decimals))
    {
        lcd_rows_skipped++;
        return LCD_OK;
    }

    LCDStatus_t status = clear_value(row);
    if (status != LCD_OK)
        return status;
    status = LCD_BufShowAscii(COL_VALUE_X, ROW_Y(row), text, text_length, CYAN, BLACK, 16u);
    if (status != LCD_OK)
        return status;
    status = LCD_BufShowAscii(COL_VALUE_X + 8u * 8u, ROW_Y(row), unit,
                              unit_length, GRAY, BLACK, 16u);
    if (status != LCD_OK)
        return status;
    value_mark_drawn(row, VALUE_KIND_FLOAT, text, decimals);
    return LCD_OK;
}

// 在调用方提供的文本空间内写故障编号，并追加已知故障的简称。
static void build_fault_string(uint32_t faults, char *buf, size_t len)
{
    snprintf(buf, len, "0x%04lX ", (unsigned long)faults);
    if (faults & ROBOT_FAULT_RC_OFFLINE)
        strncat(buf, "RC ", len - strlen(buf) - 1u);
    if (faults & ROBOT_FAULT_IMU_INVALID)
        strncat(buf, "IMU ", len - strlen(buf) - 1u);
    if (faults & ROBOT_FAULT_MOTOR_OFFLINE)
        strncat(buf, "MOT ", len - strlen(buf) - 1u);
    if (faults & ROBOT_FAULT_CAN_BUSOFF)
        strncat(buf, "CAN ", len - strlen(buf) - 1u);
    if (faults & ROBOT_FAULT_TILT)
        strncat(buf, "TILT ", len - strlen(buf) - 1u);
    if (faults & ROBOT_FAULT_CALIB_INVALID)
        strncat(buf, "CAL ", len - strlen(buf) - 1u);
    if (faults & ROBOT_FAULT_TASK_TIMEOUT)
        strncat(buf, "TASK", len - strlen(buf) - 1u);
}

// 第 0 页标签：整车状态、故障、输入在线情况和上次异常。
static void draw_page0_static(void)
{
    draw_label(0, L_STATE, LCD_CN_LENGTH(L_STATE));
    draw_label(1, L_FAULT, LCD_CN_LENGTH(L_FAULT));
    draw_label(2, L_ESTOP, LCD_CN_LENGTH(L_ESTOP));
    draw_label(3, L_VOLT, LCD_CN_LENGTH(L_VOLT));
    draw_label(4, L_TASK, LCD_CN_LENGTH(L_TASK));
    draw_label(5, L_RC, LCD_CN_LENGTH(L_RC));
    draw_label(6, L_VISION, LCD_CN_LENGTH(L_VISION));
    draw_label(7, L_WATCHDOG, LCD_CN_LENGTH(L_WATCHDOG));
    draw_label_ascii(8, "CRASH"); /* 上次崩溃(异常/断言/栈溢出)现场, 没崩过显示 -- */
}

// 第 0 页读取系统信息；逐项检查任务是否超时，异常记录不存在时显示 --。
static void draw_page0_values(void)
{
    size_t state_count = 0u;
    const Lcd_CnChar_e *state_text = state_label(RobotSafetyGetState(), &state_count);
    draw_value_cn(0, state_text, state_count);

    char buf[40];
    build_fault_string(RobotSafetyGetFaults(), buf, sizeof(buf));
    draw_value_ascii(1, buf);

    if (EstopIsLatched())
        snprintf(buf, sizeof(buf), "YES %s", (EstopGetReason() == ESTOP_REASON_RC_OFFLINE) ? "RC" : "MAN");
    else
        snprintf(buf, sizeof(buf), "NO");
    draw_value_ascii(2, buf);

    BSP_ADC_Sample_t adc_sample;
    BSP_ADC_Status_e adc_status = BSP_ADCGetSample(&adc_sample);
    if (adc_status == BSP_ADC_STATUS_VALID && adc_sample.voltage_config_valid)
        draw_value_float_unit(3, adc_sample.vcc_in_volts, 1, "V");
    else
        draw_value_ascii(3, "-- V");

    TaskMonitorStatus_t st;
    char t[8][4];
    for (uint8_t i = 0; i < TASK_MONITOR_COUNT; ++i)
    {
        TaskMonitorGetStatus((TaskMonitor_Id_e)i, &st);
        snprintf(t[i], sizeof(t[i]), "%s", st.alive ? "OK" : "--");
    }
    // UART发送已独立为第五个监控任务，沿用当前一行的只读状态显示。
    snprintf(buf, sizeof(buf), "I:%s M:%s R:%s D:%s U:%s", t[0], t[1], t[2], t[3], t[TASK_MONITOR_UART_TX]);
    draw_value_ascii(4, buf);

#if MC02_FEATURE_REMOTE
    draw_value_cn(5, online_label(RemoteControlIsOnline()), LCD_CN_LENGTH(L_ONLINE));
#else
    draw_value_ascii(5, "RC:--");
#endif

    Vision_Status_t vs;
    VisionGetStatus(&vs);
    snprintf(buf, sizeof(buf), "%s M:%u A:%lums", vs.online ? "ON" : "OFF",
             (unsigned)vs.mode, (unsigned long)(vs.online ? (HAL_GetTick() - vs.last_rx_ms) : 0u));
    draw_value_ascii(6, buf);

    snprintf(buf, sizeof(buf), "RUN:%s RST:%s", BSP_WatchdogIsRunning() ? "Y" : "N",
             BSP_WatchdogWasIwdgReset() ? "Y" : "N");
    draw_value_ascii(7, buf);

    /* 上次崩溃现场: 类型 + 出错 PC + 累计次数(没崩过/记录不可信就显示 --)。
       栈溢出那条记录里没有有意义的 PC(原来的 pc 字段是任务名指针), 改显示任务名。 */
    CrashLog_t cl;
    if (CrashLogGetLast(&cl))
    {
        if (cl.type == CRASH_TYPE_STACK_OVF)
            snprintf(buf, sizeof(buf), "%s@%s x%lu", CrashLogTypeShort(cl.type),
                     cl.task_name, (unsigned long)cl.count);
        else
            snprintf(buf, sizeof(buf), "%s@%08lX x%lu", CrashLogTypeShort(cl.type),
                     (unsigned long)cl.pc, (unsigned long)cl.count);
    }
    else
        snprintf(buf, sizeof(buf), "--");
    draw_value_ascii(8, buf);
}

// 第 1 页标签：温度、加热输出、姿态和标定来源。
static void draw_page1_static(void)
{
    draw_label(0, L_TEMP, LCD_CN_LENGTH(L_TEMP));
    draw_label(1, L_TARGET, LCD_CN_LENGTH(L_TARGET));
    draw_label(2, L_DUTY, LCD_CN_LENGTH(L_DUTY));
    draw_label(3, L_HEAT, LCD_CN_LENGTH(L_HEAT));
    draw_label(4, L_VALID, LCD_CN_LENGTH(L_VALID));
    draw_label_ascii(5, "YAW");
    draw_label_ascii(6, "PIT");
    draw_label_ascii(7, "ROL");
    draw_label_ascii(8, "CAL"); /* 标定来源 + 标定温度 */
}

// 第 1 页按构建选择显示温控或禁用占位值，并显示姿态数据；姿态不可用时以 -- 替代角度。
static void draw_page1_values(void)
{
#if MC02_FEATURE_HEATER
    IMUHeaterStatus_t hs;
    IMUHeaterGetStatus(&hs);

    // 显示标定来源及温度；共用绘图模块的有限值和舍入检查。
    char cal_buf[24];
    const char *src;
    switch (bmi088_calib_source)
    {
    case BMI088_CALIB_SRC_FLASH:
        src = "FLASH";
        break;
    case BMI088_CALIB_SRC_FIRST_AUTO:
        src = "AUTO";
        break;
    case BMI088_CALIB_SRC_MANUAL:
        src = "MAN";
        break;
    case BMI088_CALIB_SRC_DEFAULT:
        src = "DEF";
        break;
    default:
        src = "--";
        break;
    }
    char temperature[16];
    size_t temperature_length = 0u;
    LCDStatus_t temperature_status = LCD_BufFormatFloat(bmi088_calib_temp, 1u,
                                                        temperature, sizeof(temperature),
                                                        &temperature_length);
    if (temperature_status == LCD_OK && bmi088_calib_temp > -40.0f &&
        bmi088_calib_temp < 100.0f && bmi088_calib_temp != 0.0f && temperature_length < sizeof(temperature))
    {
        snprintf(cal_buf, sizeof(cal_buf), "%s %sC", src, temperature);
    }
    else
    {
        snprintf(cal_buf, sizeof(cal_buf), "%s --", src);
    }
    draw_value_ascii(8, cal_buf);

    draw_value_float_unit(0, hs.temperature, 1, "C");
    draw_value_float_unit(1, hs.target_temp, 1, "C");

    char buf[32];
    uint32_t permille = (uint32_t)hs.duty * 1000u / 9999u;
    snprintf(buf, sizeof(buf), "%u %lu.%lu%%", (unsigned)hs.duty,
             (unsigned long)(permille / 10u), (unsigned long)(permille % 10u));
    draw_value_ascii(2, buf);

    if (hs.fault)
        snprintf(buf, sizeof(buf), "FAULT%s%s%s", hs.overtemp ? " OVT" : "",
                 hs.timeout_fault ? " TMO" : "", hs.sensor_valid ? "" : " SEN");
    else
        snprintf(buf, sizeof(buf), "%s", hs.heating ? "ON" : "OFF");
    draw_value_ascii(3, buf);

    draw_value_cn(4, hs.sensor_valid ? L_VALID : L_INVALID, LCD_CN_LENGTH(L_VALID));
#else
    draw_value_ascii(0, "--");
    draw_value_ascii(1, "--");
    draw_value_ascii(2, "--");
    draw_value_ascii(3, "DISABLED");
    draw_value_ascii(4, "--");
#endif

    attitude_t *att = INS_GetAttitude();
    if (att == NULL)
    {
        draw_value_ascii(5, "--");
        draw_value_ascii(6, "--");
        draw_value_ascii(7, "--");
        return;
    }
    draw_value_float_unit(5, att->Yaw, 1, "d");
    draw_value_float_unit(6, att->Pitch, 1, "d");
    draw_value_float_unit(7, att->Roll, 1, "d");
}

// 第 2 页标签：裁判状态、功率和弹量。
static void draw_page2_static(void)
{
    draw_label(0, L_REFEREE, LCD_CN_LENGTH(L_REFEREE));
    draw_label(1, L_STAGE, LCD_CN_LENGTH(L_STAGE));
    draw_label(2, L_HP, LCD_CN_LENGTH(L_HP));
    draw_label(3, L_POWER, LCD_CN_LENGTH(L_POWER));
    draw_label(4, L_BUFFER, LCD_CN_LENGTH(L_BUFFER));
    draw_label(5, L_HEATQ, LCD_CN_LENGTH(L_HEATQ));
    draw_label(6, L_AMMO, LCD_CN_LENGTH(L_AMMO));
    draw_label(7, L_BULLET_SPEED, LCD_CN_LENGTH(L_BULLET_SPEED));
    draw_label(8, L_COIN, LCD_CN_LENGTH(L_COIN));
}

// 裁判离线或尚无数据对象时，逐行清掉旧值并显示 --。
static void draw_page2_values(void)
{
    RefereeSnapshot_t ref = {0};
#if MC02_FEATURE_REFEREE
    uint8_t available = (uint8_t)(RefereeGetSnapshot(&ref) == REFEREE_OK);
#else
    uint8_t available = 0u;
#endif
    draw_value_cn(0, online_label((uint8_t)(available && ref.link_online)), LCD_CN_LENGTH(L_ONLINE));
    char values[LCD_REFEREE_VALUE_COUNT][LCD_REFEREE_VALUE_SIZE];
    LCDRefereeFormatRows(available ? &ref : NULL, values);
    for (uint8_t i=0u;i<LCD_REFEREE_VALUE_COUNT;++i) draw_value_ascii((uint8_t)(i+1u),values[i]);
}

// 第 3 页标签：各电机在线情况、CAN、视觉和按键读数。
static void draw_page3_static(void)
{
    draw_label(0, L_CHASSIS, LCD_CN_LENGTH(L_CHASSIS));
    draw_label(1, L_GIMBAL, LCD_CN_LENGTH(L_GIMBAL));
    draw_label(2, L_SHOOT, LCD_CN_LENGTH(L_SHOOT));
    draw_label_ascii(3, "LF");
    draw_label_ascii(4, "YAW");
    draw_label(5, L_BUS, LCD_CN_LENGTH(L_BUS));
    draw_label(6, L_LINK, LCD_CN_LENGTH(L_LINK));
    draw_label_ascii(7, "KEY");
}

// 汇总模块提供的当前状态；无有效 ADC 样本时不显示旧按键读数。
static void draw_page3_values(void)
{
    char buf[48];
#if defined(ONE_BOARD) || defined(CHASSIS_BOARD)
    Chassis_Motor_Summary_t cs = {0};
    Chassis_GetMotorSummary(&cs);
#else
    // 当前角色不编译底盘应用，用占位避免LCD引用未选择的驱动入口。
    draw_value_ascii(0, "--");
    draw_value_ascii(3, "--");
#endif
#if defined(ONE_BOARD) || defined(GIMBAL_BOARD)
    Gimbal_Motor_Summary_t gs = {0};
    Shoot_Motor_Summary_t ss = {0};
    Gimbal_GetMotorSummary(&gs);
    Shoot_GetMotorSummary(&ss);

    snprintf(buf, sizeof(buf), "Y:%s P:%s", gs.yaw.online ? "1" : "0", gs.pitch.online ? "1" : "0");
    draw_value_ascii(1, buf);
    snprintf(buf, sizeof(buf), "FL:%s FR:%s LDR:%s", ss.friction_l.online ? "1" : "0",
             ss.friction_r.online ? "1" : "0", ss.loader.online ? "1" : "0");
    draw_value_ascii(2, buf);
    snprintf(buf, sizeof(buf), "spd:%ld cur:%d", (long)gs.yaw.speed_aps, (int)gs.yaw.current);
    draw_value_ascii(4, buf);
#else
    // 当前角色不编译云台和发射应用，显示占位并保持其余总线诊断可用。
    draw_value_ascii(1, "--");
    draw_value_ascii(2, "--");
    draw_value_ascii(4, "--");
#endif
#if defined(ONE_BOARD) || defined(CHASSIS_BOARD)
    snprintf(buf, sizeof(buf), "%s %s %s %s", cs.lf.online ? "1" : "0", cs.rf.online ? "1" : "0",
             cs.lb.online ? "1" : "0", cs.rb.online ? "1" : "0");
    draw_value_ascii(0, buf);
    snprintf(buf, sizeof(buf), "spd:%ld cur:%d", (long)cs.lf.speed_aps, (int)cs.lf.current);
    draw_value_ascii(3, buf);
#endif

    CAN_Status_t c1, c2, c3;
    CANGetStatus(&hfdcan1, &c1);
    CANGetStatus(&hfdcan2, &c2);
    CANGetStatus(&hfdcan3, &c3);
    snprintf(buf, sizeof(buf), "C1:%s C2:%s C3:%s", can_status_str(c1.status),
             can_status_str(c2.status), can_status_str(c3.status));
    draw_value_ascii(5, buf);

    Vision_Status_t vs;
    VisionGetStatus(&vs);
    snprintf(buf, sizeof(buf), "%s M:%u BC:%u", vs.online ? "ON" : "OFF",
             (unsigned)vs.mode, (unsigned)vs.bullet_count);
    draw_value_ascii(6, buf);

    /* 五向按键原始 ADC 值: 用于把摇杆方向与阈值对上(按一下即可读出)。
       min 是"上电以来见过的最小原始值" —— 摇杆中键按下时 ADC 会趋近 0,
       所以按住中键看一眼 min 就知道中键的实际档位值(即使识别失败也能据此调阈值)。 */
    BSP_ADC_Sample_t adc_sample;
    BSP_ADC_Status_e adc_status = BSP_ADCGetSample(&adc_sample);
    if (adc_status == BSP_ADC_STATUS_VALID)
        snprintf(buf, sizeof(buf), "raw:%u min:%u", (unsigned)adc_sample.raw_key,
                 (unsigned)lcd_key_raw_min);
    else
        snprintf(buf, sizeof(buf), "raw:-- min:%u", (unsigned)lcd_key_raw_min);
    draw_value_ascii(7, buf);
}

LCDStatus_t LCD_UI_DrawStatic(uint8_t page)
{
    LCDStatus_t status = lcd_ui_begin(250u, 0u);
    if (status != LCD_OK)
        return status;

    /* 整页重画会覆盖/清空整个值区: 必须让值缓存失效,
       否则下一次 UpdateValues() 会因为"内容没变"而跳过, 屏幕就一直空着。 */
    LCD_UI_InvalidateValueCache();

    lcd_page = page % LCD_PAGE_COUNT;

    // 清旧标签列；标题自行铺底色，值区由下一次 UpdateValues() 逐行清理和重画。
    LCD_BufFill(0, 24u, (uint16_t)(COL_VALUE_X - 1u), LCD_H - 1u, BLACK);
    draw_page_title(lcd_page);

    switch (lcd_page)
    {
    case 0:
        draw_page0_static();
        break;
    case 1:
        draw_page1_static();
        break;
    case 2:
        draw_page2_static();
        break;
    default:
        draw_page3_static();
        break;
    }
    status = LCD_InternalCurrentStatus();
    return lcd_ui_end(status);
}

LCDStatus_t LCD_UI_UpdateValues(uint8_t page)
{
    LCDStatus_t status = lcd_ui_begin(20u, 0u);
    if (status != LCD_OK)
        return status;
    lcd_page = page % LCD_PAGE_COUNT;

    switch (lcd_page)
    {
    case 0:
        draw_page0_values();
        break;
    case 1:
        draw_page1_values();
        break;
    case 2:
        draw_page2_values();
        break;
    default:
        draw_page3_values();
        break;
    }
    status = LCD_InternalCurrentStatus();
    return lcd_ui_end(status);
}

LCDStatus_t LCD_UI_Init(void)
{
    LCDStatus_t status = lcd_ui_begin(1000u, 0u);
    if (status != LCD_OK)
        return status;

    lcd_page = 0;
    lcd_frozen = 0;
    lcd_init_stage = 1;
    LCD_UI_InvalidateValueCache();
    status = LCD_Init();
    if (status == LCD_OK)
    {
        lcd_init_stage = 2;
        status = LCD_BufClearPanel();
    }
    if (status == LCD_OK)
    {
        lcd_init_stage = 3;
        status = LCD_BufFill(0, 0, LCD_W - 1u, LCD_H - 1u, BLACK);
    }
    if (status == LCD_OK)
    {
        lcd_init_stage = 4;
        status = LCD_UI_DrawStatic(0u);
    }
    if (status == LCD_OK)
    {
        lcd_init_stage = 5;
        status = LCD_UI_UpdateValues(0u);
    }
    if (status == LCD_OK)
        status = LCD_InternalCommitUI();
    if (status == LCD_OK)
        lcd_init_stage = 6;

    return lcd_ui_end(status);
}

// 面板重新上电后配置可能丢失；确认 SPI 停止后再重置面板并重画。
LCDStatus_t LCD_UI_Recover(void)
{
    LCDStatus_t status = lcd_ui_begin(1000u, 1u);
    if (status != LCD_OK)
        return status;

    status = LCD_InternalStopForRecovery();
    if (status == LCD_OK)
        LCD_UI_InvalidateValueCache();
    if (status == LCD_OK)
        status = LCD_Init();
    if (status == LCD_OK)
        status = LCD_BufClearPanel();
    if (status == LCD_OK)
        status = LCD_BufFill(0, 0, LCD_W - 1u, LCD_H - 1u, BLACK);
    if (status == LCD_OK)
        status = LCD_UI_DrawStatic(lcd_page);
    if (status == LCD_OK)
        status = LCD_UI_UpdateValues(lcd_page);
    if (status == LCD_OK)
        status = LCD_InternalCommitUI();
    return lcd_ui_end(status);
}

LCDStatus_t LCD_UI_SetFrozen(uint8_t frozen)
{
    LCDStatus_t status = lcd_ui_begin(250u, 0u);
    if (status != LCD_OK)
        return status;
    lcd_frozen = frozen ? 1u : 0u;
    // 只重画标题，让冻结标记出现或消失；值区更新由 LCD 任务决定。
    draw_page_title(lcd_page);
    status = LCD_InternalCurrentStatus();
    return lcd_ui_end(status);
}

// 只覆盖页面中间一块，减少标定期间显示所需的发送时间。
LCDStatus_t LCD_UI_ShowCalibMsg(uint8_t msg)
{
    LCDStatus_t status = lcd_ui_begin(250u, 0u);
    if (status != LCD_OK)
        return status;

    if (msg == 0u)
    {
        status = LCD_UI_DrawStatic(lcd_page);
        if (status == LCD_OK)
            status = LCD_UI_UpdateValues(lcd_page);
        return lcd_ui_end(status);
    }

    const Lcd_CnChar_e *title;
    const Lcd_CnChar_e *sub = NULL;
    size_t title_count;
    size_t sub_count = 0u;
    uint16_t color;
    switch (msg)
    {
    case 1u:
        title = L_CAL_RUN;
        sub = L_CAL_HOLD;
        title_count = LCD_CN_LENGTH(L_CAL_RUN);
        sub_count = LCD_CN_LENGTH(L_CAL_HOLD);
        color = YELLOW;
        break;
    case 2u:
        title = L_CAL_OK;
        title_count = LCD_CN_LENGTH(L_CAL_OK);
        color = GREEN;
        break;
    default:
        title = L_CAL_FAIL;
        title_count = LCD_CN_LENGTH(L_CAL_FAIL);
        color = RED;
        break;
    }

    uint16_t y0 = 66u;
    uint16_t y1 = (uint16_t)(y0 + ((sub != NULL) ? 78u : 56u));
    if (y1 > (uint16_t)(LCD_H - 1u))
        y1 = (uint16_t)(LCD_H - 1u);

    /* 底色 + 上下两条彩线, 不用整屏重画 */
    status = LCD_BufFill(0, y0, (uint16_t)(LCD_W - 1u), y1, BLACK);
    if (status == LCD_OK)
        status = LCD_BufFill(0, y0, (uint16_t)(LCD_W - 1u), (uint16_t)(y0 + 1u), color);
    if (status == LCD_OK)
        status = LCD_BufFill(0, (uint16_t)(y1 - 1u), (uint16_t)(LCD_W - 1u), y1, color);

    uint16_t tx = (uint16_t)((LCD_W - title_count * 16u) / 2u);
    if (status == LCD_OK)
        status = LCD_BufShowCnString(tx, (uint16_t)(y0 + 14u), title, title_count, color, BLACK);

    if (status == LCD_OK && sub != NULL)
    {
        tx = (uint16_t)((LCD_W - sub_count * 16u) / 2u);
        status = LCD_BufShowCnString(tx, (uint16_t)(y0 + 44u), sub, sub_count, WHITE, BLACK);
    }
    return lcd_ui_end(status);
}

#if MC02_BENCH_LCD_DIAGNOSTICS
LCDStatus_t LCD_UI_DrawDiagnosticPattern(uint8_t pattern)
{
    LCDStatus_t status = lcd_ui_begin(250u, 0u);
    if (status != LCD_OK)
        return status;
    if (pattern > 3u)
        return lcd_ui_end(LCD_OUT_OF_RANGE);

    status = LCD_BufFill(0u, 0u, LCD_W - 1u, LCD_H - 1u, BLACK);
    if (status == LCD_OK && pattern == 0u)
    {
        status = LCD_BufFill(0u, 0u, LCD_W - 1u, 1u, RED);
        if (status == LCD_OK) status = LCD_BufFill(0u, LCD_H - 2u, LCD_W - 1u, LCD_H - 1u, GREEN);
        if (status == LCD_OK) status = LCD_BufFill(0u, 0u, 1u, LCD_H - 1u, BLUE);
        if (status == LCD_OK) status = LCD_BufFill(LCD_W - 2u, 0u, LCD_W - 1u, LCD_H - 1u, YELLOW);
        if (status == LCD_OK) status = LCD_BufFill(12u, 18u, 58u, 42u, MAGENTA);
        if (status == LCD_OK) status = LCD_BufFill(64u, 18u, 110u, 42u, CYAN);
    }
    else if (status == LCD_OK && pattern == 1u)
    {
        static const char first[] = "ASCII 12px 0123456789";
        static const char second[] = "ASCII 16px AaZz!?";
        status = LCD_BufShowAscii(8u, 72u, first, LCD_ASCII_LENGTH(first), WHITE, BLACK, 12u);
        if (status == LCD_OK)
            status = LCD_BufShowAscii(8u, 100u, second, LCD_ASCII_LENGTH(second), CYAN, BLACK, 16u);
    }
    else if (status == LCD_OK && pattern == 2u)
    {
        status = LCD_BufShowCnString(12u, 72u, L_INIT, LCD_CN_LENGTH(L_INIT), WHITE, BLACK);
        if (status == LCD_OK)
            status = LCD_BufShowCnString(12u, 104u, L_CAL_OK, LCD_CN_LENGTH(L_CAL_OK), GREEN, BLACK);
    }
    else if (status == LCD_OK && pattern == 3u)
    {
        status = LCD_BufShowInt(8u, 44u, INT32_MIN, WHITE, BLACK, 12u);
        if (status == LCD_OK) status = LCD_BufShowInt(8u, 68u, INT32_MAX, CYAN, BLACK, 12u);
        if (status == LCD_OK) status = LCD_BufShowFloat(8u, 96u, 1.25f, 1u, GREEN, BLACK, 16u);
        if (status == LCD_OK) status = LCD_BufShowFloat(8u, 124u, -1.25f, 1u, YELLOW, BLACK, 16u);
        if (status == LCD_OK) status = LCD_BufShowFloat(8u, 152u, 0.0f, 3u, WHITE, BLACK, 12u);
    }

    return lcd_ui_end(status);
}
#endif
