#include "lcd_ui.h"
#include "lcd.h"
#include "lcd_buf.h"
#include "lcd_font_cn.h"
#include "robot_safety.h"
#include "bsp_adc.h"
#include "bsp_watchdog.h"
#include "task_monitor.h"
#include "remote_control.h"
#include "master_process.h"
#include "imu_heater.h"
#include "ins_task.h"
#include "referee_task.h"
#include "rm_referee.h"
#include "chassis.h"
#include "gimbal.h"
#include "shoot.h"
#include "bsp_can.h"
#include "fdcan.h"
#include "bmi088.h"
#include "bsp_crash.h"
#include <stdio.h>
#include <string.h>

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
static const Lcd_CnChar_e L_INVALID[] = {LCD_CN_WU2, LCD_CN_XIAO, LCD_CN_COUNT};
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

static const Lcd_CnChar_e *state_label(Robot_Status_e state)
{
    switch (state)
    {
    case ROBOT_INIT:
        return L_INIT;
    case ROBOT_SAFE:
        return L_SAFE;
    case ROBOT_CALIB:
        return L_CALIB;
    case ROBOT_READY:
        return L_READY;
    case ROBOT_FAULT:
        return L_ERROR;
    case ROBOT_ESTOP:
        return L_ESTOP;
    default:
        return L_NO;
    }
}

static const Lcd_CnChar_e *online_label(uint8_t online)
{
    return online ? L_ONLINE : L_OFFLINE;
}

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

/* 数一个以 LCD_CN_COUNT 结尾的中文字符串有几个字(用于居中) */
static uint8_t cn_len(const Lcd_CnChar_e *str)
{
    uint8_t n = 0;
    while (str[n] != LCD_CN_COUNT && n < 32u)
        ++n;
    return n;
}

static void draw_title(const Lcd_CnChar_e *title_cn, const char *title_ascii)
{
    LCD_BufFill(0, 0, LCD_W - 1u, 23u, BLUE);
    if (title_cn != NULL)
        LCD_BufShowCnString(4, 4, title_cn, WHITE, BLUE);
    else
        LCD_BufShowAscii(4, 4, title_ascii, WHITE, BLUE, 16);

    char page_buf[8];
    snprintf(page_buf, sizeof(page_buf), "%u/%u", (unsigned)(lcd_page + 1u), (unsigned)LCD_PAGE_COUNT);
    LCD_BufShowAscii((uint16_t)(LCD_W - 28u), 4, page_buf, CYAN, BLUE, 16);

    if (lcd_frozen)
        LCD_BufShowCnString((uint16_t)(LCD_W - 68u), 4, L_FREEZE, YELLOW, BLUE);
}

/**
 * @brief 按页码重画标题栏(draw_title 内部会先铺满标题栏底色, 所以是"完整重画")
 * @note  抽出来是为了两个地方能单独刷新标题栏而不必整页重画:
 *          1) 冻结/恢复 —— 只影响右上角那个"冻结"标记;
 *          2) LCD_UI_DrawStatic() —— 统一画标题, 各页的 static 函数就不再重复画了。
 */
static void draw_page_title(uint8_t page)
{
    switch (page)
    {
    case 0u:
#if defined(MC02_PROFILE_BENCH_SAFE)
        draw_title(NULL, "SAFE BENCH");
#else
        draw_title(L_SYS, NULL);
#endif
        break;
    case 1u:
        draw_title(NULL, "IMU");
        break;
    case 2u:
        draw_title(L_POWER, NULL);
        break;
    default:
        draw_title(L_MOTOR, NULL);
        break;
    }
}

static void draw_label(uint8_t row, const Lcd_CnChar_e *label)
{
    LCD_BufShowCnString(COL_LABEL_X, ROW_Y(row), label, WHITE, BLACK);
}

static void draw_label_ascii(uint8_t row, const char *label)
{
    LCD_BufShowAscii(COL_LABEL_X, ROW_Y(row), label, WHITE, BLACK, 16);
}

static void clear_value(uint8_t row)
{
    LCD_BufFill(COL_VALUE_X, ROW_Y(row), LCD_W - 1u, ROW_Y(row) + 15u, BLACK);
}

/* ---------------- 值区重绘缓存(2026-09-26, LCD-12) ----------------
   刷新率 5Hz、每帧无条件重画 9 行值区, 但大部分数值(电压/状态/在线位)根本不变化 ——
   白白吃掉 SPI 带宽和 CPU。这里按"页 × 行"记住上次画的内容, 内容没变就整行跳过
   (连清屏都不做)。
   !! 任何"整页被重画/被破坏"的路径都必须先 LCD_UI_InvalidateValueCache(),
      否则会出现"屏幕被清了但值一直不刷新"。目前只在 LCD_UI_DrawStatic() 里调用,
      而所有整页重画(Init/Recover/SetFrozen/翻页/标定提示消失)都会经过它。 */
#define LCD_ROW_COUNT 9u
#define LCD_TEXT_MAX 48u

/* 行内容类型: 0=空(没画过) 1=ASCII 2=中文标签 3=浮点+单位 */
#define VALUE_KIND_NONE 0u
#define VALUE_KIND_ASCII 1u
#define VALUE_KIND_CN 2u
#define VALUE_KIND_FLOAT 3u

static char s_last_text[LCD_PAGE_COUNT][LCD_ROW_COUNT][LCD_TEXT_MAX];
static const Lcd_CnChar_e *s_last_cn[LCD_PAGE_COUNT][LCD_ROW_COUNT];
static float s_last_float[LCD_PAGE_COUNT][LCD_ROW_COUNT];
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
        return ((s_last_float[lcd_page][row] == *(const float *)data) &&
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
        strncpy(s_last_text[lcd_page][row], (const char *)data, LCD_TEXT_MAX - 1u);
        s_last_text[lcd_page][row][LCD_TEXT_MAX - 1u] = '\0';
        break;
    case VALUE_KIND_CN:
        s_last_cn[lcd_page][row] = (const Lcd_CnChar_e *)data;
        break;
    case VALUE_KIND_FLOAT:
        s_last_float[lcd_page][row] = *(const float *)data;
        s_last_decimals[lcd_page][row] = decimals;
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

static void draw_value_ascii(uint8_t row, const char *str)
{
    /* 可用宽度随 LCD_W 变化(可见区偏移会缩小绘图区)，过长时先降字号，
       仍然放不下就截断，避免整行因为越界校验而什么都不显示。 */
    size_t len = strlen(str);
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

    /* 比较"实际要画出去的文本"而不是原始入参: 原始串可能每次都不同但显示结果相同。
       内容没变就整行跳过(连清屏都不做) —— 这是本次性能优化的主要来源。 */
    if (value_unchanged(row, VALUE_KIND_ASCII, buf, 0u))
    {
        lcd_rows_skipped++;
        return;
    }

    clear_value(row);
    LCD_BufShowAscii(COL_VALUE_X, ROW_Y(row), buf, CYAN, BLACK, sizey);
    value_mark_drawn(row, VALUE_KIND_ASCII, buf, 0u);
}

static void draw_value_cn(uint8_t row, const Lcd_CnChar_e *str)
{
    /* 中文值都是标签数组(如"在线"/"离线"), 比指针就够 —— 同一内容每次传的是同一个数组 */
    if (value_unchanged(row, VALUE_KIND_CN, str, 0u))
    {
        lcd_rows_skipped++;
        return;
    }

    clear_value(row);
    LCD_BufShowCnString(COL_VALUE_X, ROW_Y(row), str, CYAN, BLACK);
    value_mark_drawn(row, VALUE_KIND_CN, str, 0u);
}

static void draw_value_float_unit(uint8_t row, float value, uint8_t decimals, const char *unit)
{
    /* 本工程不用 %f(nano.specs 不支持), 没法先格式化成字符串比较,
       所以直接比较"数值 + 小数位数"。同一行 unit 是固定的, 不参与比较。 */
    if (value_unchanged(row, VALUE_KIND_FLOAT, &value, decimals))
    {
        lcd_rows_skipped++;
        return;
    }

    clear_value(row);
    LCD_BufShowFloat(COL_VALUE_X, ROW_Y(row), value, decimals, CYAN, BLACK, 16);
    LCD_BufShowAscii(COL_VALUE_X + 8u * 8u, ROW_Y(row), unit, GRAY, BLACK, 16);
    value_mark_drawn(row, VALUE_KIND_FLOAT, &value, decimals);
}

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

static void draw_page0_static(void)
{
    draw_label(0, L_STATE);
    draw_label(1, L_FAULT);
    draw_label(2, L_ESTOP);
    draw_label(3, L_VOLT);
    draw_label(4, L_TASK);
    draw_label(5, L_RC);
    draw_label(6, L_VISION);
    draw_label(7, L_WATCHDOG);
    draw_label_ascii(8, "CRASH"); /* 上次崩溃(异常/断言/栈溢出)现场, 没崩过显示 -- */
}

static void draw_page0_values(void)
{
    draw_value_cn(0, state_label(RobotSafetyGetState()));

    char buf[40];
    build_fault_string(RobotSafetyGetFaults(), buf, sizeof(buf));
    draw_value_ascii(1, buf);

    if (EstopIsLatched())
        snprintf(buf, sizeof(buf), "YES %s", (EstopGetReason() == ESTOP_REASON_RC_OFFLINE) ? "RC" : "MAN");
    else
        snprintf(buf, sizeof(buf), "NO");
    draw_value_ascii(2, buf);

    draw_value_float_unit(3, BSP_ADCGetVccIn(), 1, "V");

    TaskMonitorStatus_t st;
    char t[8][4];
    for (uint8_t i = 0; i < TASK_MONITOR_COUNT; ++i)
    {
        TaskMonitorGetStatus((TaskMonitor_Id_e)i, &st);
        snprintf(t[i], sizeof(t[i]), "%s", st.alive ? "OK" : "--");
    }
    snprintf(buf, sizeof(buf), "I:%s M:%s R:%s D:%s", t[0], t[1], t[2], t[3]);
    draw_value_ascii(4, buf);

    draw_value_cn(5, online_label(RemoteControlIsOnline()));

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

static void draw_page1_static(void)
{
    draw_label(0, L_TEMP);
    draw_label(1, L_TARGET);
    draw_label(2, L_DUTY);
    draw_label(3, L_HEAT);
    draw_label(4, L_VALID);
    draw_label_ascii(5, "YAW");
    draw_label_ascii(6, "PIT");
    draw_label_ascii(7, "ROL");
    draw_label_ascii(8, "CAL"); /* 标定来源 + 标定温度 */
}

static void draw_page1_values(void)
{
    IMUHeaterStatus_t hs;
    IMUHeaterGetStatus(&hs);

    /* 标定来源(FLASH=读参数区 / AUTO=首次自动 / MAN=按需标定 / DEF=默认值)
       + 标定时温度。snprintf 在本工程用 nano.specs, 不支持 %f, 所以手算一位小数。 */
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
    if (bmi088_calib_temp > -40.0f && bmi088_calib_temp < 100.0f && bmi088_calib_temp != 0.0f)
    {
        int32_t t10 = (int32_t)(bmi088_calib_temp * 10.0f + 0.5f);
        if (t10 < 0)
            t10 = 0; /* 手算取整在负数上会不对称, 这里温度不会为负, 直接钳到 0 */
        snprintf(cal_buf, sizeof(cal_buf), "%s %ld.%ldC", src,
                 (long)(t10 / 10), (long)(t10 % 10));
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

    draw_value_cn(4, hs.sensor_valid ? L_VALID : L_INVALID);

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

static void draw_page2_static(void)
{
    draw_label(0, L_REFEREE);
    draw_label(1, L_STAGE);
    draw_label(2, L_HP);
    draw_label(3, L_POWER);
    draw_label(4, L_BUFFER);
    draw_label(5, L_HEATQ);
    draw_label(6, L_AMMO);
    draw_label(7, L_BULLET_SPEED);
    draw_label(8, L_COIN);
}

static void draw_page2_values(void)
{
    uint8_t online = RefereeIsOnline();
    referee_info_t *ref = RefereeGetInfo();
    draw_value_cn(0, online_label(online));

    if (!online || ref == NULL)
    {
        for (uint8_t row = 1; row <= 8u; ++row)
            draw_value_ascii(row, "--");
        return;
    }

    char buf[40];
    snprintf(buf, sizeof(buf), "G:%u T:%us", (unsigned)ref->GameState.game_progress,
             (unsigned)ref->GameState.stage_remain_time);
    draw_value_ascii(1, buf);

    snprintf(buf, sizeof(buf), "%u/%u", (unsigned)ref->GameRobotState.current_HP,
             (unsigned)ref->GameRobotState.maximum_HP);
    draw_value_ascii(2, buf);

    float chassis_power = ref->PowerHeatData.reserved_3;
    snprintf(buf, sizeof(buf), "%u/%uW", (unsigned)chassis_power,
             (unsigned)ref->GameRobotState.chassis_power_limit);
    draw_value_ascii(3, buf);

    snprintf(buf, sizeof(buf), "%u", (unsigned)ref->PowerHeatData.buffer_energy);
    draw_value_ascii(4, buf);

    snprintf(buf, sizeof(buf), "%u/%u", (unsigned)ref->PowerHeatData.shooter_17mm_barrel_heat,
             (unsigned)ref->GameRobotState.shooter_barrel_heat_limit);
    draw_value_ascii(5, buf);

    snprintf(buf, sizeof(buf), "%u/%u", (unsigned)ref->ProjectileAllowance.projectile_allowance_17mm,
             (unsigned)ref->ProjectileAllowance.remaining_gold_coin);
    draw_value_ascii(6, buf);

    draw_value_float_unit(7, ref->ShootData.initial_speed, 1, "m/s");

    snprintf(buf, sizeof(buf), "%u", (unsigned)ref->ProjectileAllowance.remaining_gold_coin);
    draw_value_ascii(8, buf);
}

static void draw_page3_static(void)
{
    draw_label(0, L_CHASSIS);
    draw_label(1, L_GIMBAL);
    draw_label(2, L_SHOOT);
    draw_label_ascii(3, "LF");
    draw_label_ascii(4, "YAW");
    draw_label(5, L_BUS);
    draw_label(6, L_LINK);
    draw_label_ascii(7, "KEY");
}

static void draw_page3_values(void)
{
    char buf[48];
    Chassis_Motor_Summary_t cs;
    Gimbal_Motor_Summary_t gs;
    Shoot_Motor_Summary_t ss;
    Chassis_GetMotorSummary(&cs);
    Gimbal_GetMotorSummary(&gs);
    Shoot_GetMotorSummary(&ss);

    snprintf(buf, sizeof(buf), "%s %s %s %s", cs.lf.online ? "1" : "0", cs.rf.online ? "1" : "0",
             cs.lb.online ? "1" : "0", cs.rb.online ? "1" : "0");
    draw_value_ascii(0, buf);

    snprintf(buf, sizeof(buf), "Y:%s P:%s", gs.yaw.online ? "1" : "0", gs.pitch.online ? "1" : "0");
    draw_value_ascii(1, buf);

    snprintf(buf, sizeof(buf), "FL:%s FR:%s LDR:%s", ss.friction_l.online ? "1" : "0",
             ss.friction_r.online ? "1" : "0", ss.loader.online ? "1" : "0");
    draw_value_ascii(2, buf);

    snprintf(buf, sizeof(buf), "spd:%ld cur:%d", (long)cs.lf.speed_aps, (int)cs.lf.current);
    draw_value_ascii(3, buf);

    snprintf(buf, sizeof(buf), "spd:%ld cur:%d", (long)gs.yaw.speed_aps, (int)gs.yaw.current);
    draw_value_ascii(4, buf);

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

void LCD_UI_DrawStatic(uint8_t page)
{
    /* 整页重画会覆盖/清空整个值区: 必须让值缓存失效,
       否则下一次 UpdateValues() 会因为"内容没变"而跳过, 屏幕就一直空着。 */
    LCD_UI_InvalidateValueCache();

    lcd_page = page % LCD_PAGE_COUNT;

    /* 只清"会变的那一块", 不再整屏清黑 ——
       原来这里 LCD_BufFill 整屏刷黑, 于是每次按键(翻页/冻结/重初始化)屏幕都会
       黑一下再刷回来, 用户能看到"黑屏然后重刷"。现在:
         · 标题栏由 draw_page_title() 自己铺底色覆盖;
         · 标签列(x: 0~COL_VALUE_X-1)在这里清掉(新旧页的标签内容不同, 不清会留残影);
         · 值区交给后面的 UpdateValues() 逐行清+画(缓存刚失效, 一定会全部重画)。
       四页都是 9 行值区, 行数一致, 不会出现"上一页多出来的行没被覆盖"。 */
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
}

void LCD_UI_UpdateValues(uint8_t page)
{
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
}

void LCD_UI_Init(void)
{
    lcd_page = 0;
    lcd_frozen = 0;
    lcd_init_stage = 1;
    LCD_Init();
    lcd_init_stage = 2;
    /* 先按玻璃物理范围整屏刷黑，擦掉上一版固件留在内缩区之外的残留像素 */
    LCD_BufClearPanel();
    lcd_init_stage = 3;
    LCD_BufFill(0, 0, LCD_W - 1u, LCD_H - 1u, BLACK);
    lcd_init_stage = 4;
    LCD_UI_DrawStatic(0);
    lcd_init_stage = 5;
    LCD_UI_UpdateValues(0);
    lcd_init_stage = 6;
}

/* 面板掉电(拔插模组/欠压复位)后重新初始化并整屏重画。
   模组重新上电时 ST7789 回到默认状态(休眠、18bit 色深)，不重发初始化序列就会一直黑屏。 */
void LCD_UI_Recover(void)
{
    if (HAL_SPI_GetState(&hspi1) != HAL_SPI_STATE_READY)
    {
        HAL_SPI_Abort(&hspi1);
        MX_SPI1_Init();
    }

    LCD_Init();
    LCD_BufClearPanel();
    LCD_UI_DrawStatic(lcd_page);
    LCD_UI_UpdateValues(lcd_page);
}

void LCD_UI_SetFrozen(uint8_t frozen)
{
    lcd_frozen = frozen ? 1u : 0u;
    /* 冻结/恢复只影响标题栏右上角那个"冻结"标记 —— 只重画标题栏就够,
       不要走 DrawStatic+UpdateValues 那样整页重画(按中键会闪一下)。
       draw_title() 内部会先把整个标题栏铺成底色, 所以标记能正确出现/消失。 */
    draw_page_title(lcd_page);
}

/**
 * @brief 按需标定的屏幕提示(覆盖当前页面中间一块)
 * @param msg 0=清除提示并重画整页, 1=标定中(请保持静止), 2=成功, 3=失败
 * @note  只画中间一块, 不整屏重画 —— 标定期间 CPU 被 INS 任务大量占用,
 *        LCD 任务只能抢到很小的切片, 全屏重画会来不及。
 */
void LCD_UI_ShowCalibMsg(uint8_t msg)
{
    if (msg == 0u)
    {
        LCD_UI_DrawStatic(lcd_page);
        LCD_UI_UpdateValues(lcd_page);
        return;
    }

    const Lcd_CnChar_e *title;
    const Lcd_CnChar_e *sub = NULL;
    uint16_t color;
    switch (msg)
    {
    case 1u:
        title = L_CAL_RUN;
        sub = L_CAL_HOLD;
        color = YELLOW;
        break;
    case 2u:
        title = L_CAL_OK;
        color = GREEN;
        break;
    default:
        title = L_CAL_FAIL;
        color = RED;
        break;
    }

    uint16_t y0 = 66u;
    uint16_t y1 = (uint16_t)(y0 + ((sub != NULL) ? 78u : 56u));
    if (y1 > (uint16_t)(LCD_H - 1u))
        y1 = (uint16_t)(LCD_H - 1u);

    /* 底色 + 上下两条彩线, 不用整屏重画 */
    LCD_BufFill(0, y0, (uint16_t)(LCD_W - 1u), y1, BLACK);
    LCD_BufFill(0, y0, (uint16_t)(LCD_W - 1u), (uint16_t)(y0 + 1u), color);
    LCD_BufFill(0, (uint16_t)(y1 - 1u), (uint16_t)(LCD_W - 1u), y1, color);

    uint8_t n = cn_len(title);
    uint16_t tx = (uint16_t)((LCD_W - (uint16_t)n * 16u) / 2u);
    LCD_BufShowCnString(tx, (uint16_t)(y0 + 14u), title, color, BLACK);

    if (sub != NULL)
    {
        uint8_t m = cn_len(sub);
        tx = (uint16_t)((LCD_W - (uint16_t)m * 16u) / 2u);
        LCD_BufShowCnString(tx, (uint16_t)(y0 + 44u), sub, WHITE, BLACK);
    }
}
