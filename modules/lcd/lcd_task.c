/** @file lcd_task.c
* @brief 连接 ADC 按键、LCD 页面和标定提示，并周期记录任务耗时。
* StartLCDTASK() 先初始化屏幕；循环中先处理按键和拔插，再决定是否更新数值。
 */
#include "lcd_task.h"
#include "lcd_ui.h"
#include "lcd_buf.h"
#include "lcd_key_input.h"
#include "lcd_task_diag_state.h"
#include "bsp_adc.h"
#include "bsp_log.h"
#include "bsp_observation.h"
#include "platform_os.h"
#include "platform_probe.h"
#include "platform_sync.h"
#include "platform_task_stats.h"
#include "bmi088_calibration.h"
#include "robot_safety.h"
#include "config/build_selection.h"
#if defined(MC02_CONFIG_GENERATED)
#include "config/vehicle_selection.h"
#endif
#include "init_diagnostic.h"
#include "bsp_safety.h"
#include <string.h>

/* 五向摇杆为电阻分压, 板级例程给出的阈值是 12bit ADC, 本机 ADC 为 16bit, 故 x16:
       mid  : 0    ~ 200   -> 0     ~ 3200
       right: 700  ~ 1000  -> 11200 ~ 16000
       left : 1500 ~ 1800  -> 24000 ~ 28800
       up   : 2200 ~ 2500  -> 35200 ~ 40000
       down : 2800 ~ 3500  -> 44800 ~ 56000

   LCD 模组相对画面逆时针旋转 90 度安装, 模组自身的上下左右都要跟着转 90 度
   才是用户看到的方向, 因此:
       用户视角 "上" = 模组原始 right
       用户视角 "下" = 模组原始 left

   P4 页 KEY 行会实时显示原始 ADC 值, 实机按下后若与上表不符, 直接改下面的
   阈值宏即可(不需要改逻辑)。 */
#define ADC16(adc12) ((uint16_t)((adc12) * 16u))

#define LCD_KEY_MID_LO ADC16(0u)
#define LCD_KEY_MID_HI ADC16(200u)
#define LCD_KEY_RIGHT_LO ADC16(700u)
#define LCD_KEY_RIGHT_HI ADC16(1000u)
#define LCD_KEY_LEFT_LO ADC16(1500u)
#define LCD_KEY_LEFT_HI ADC16(1800u)
#define LCD_KEY_UP_LO ADC16(2200u)
#define LCD_KEY_UP_HI ADC16(2500u)
#define LCD_KEY_DOWN_LO ADC16(2800u)
#define LCD_KEY_DOWN_HI ADC16(3500u)

// 用按键和空闲读数范围检查模组是否接入；读数落在档位之间时记为拔出。
// 模组重新上电后需要重发初始化命令，连续八次有效读数后再恢复显示。
#define LCD_PRESENCE_WINDOW 10u
#define LCD_PRESENCE_SPREAD_MAX 1500u
#define LCD_IDLE_RAW_MIN 58000u
#define LCD_ABSENT_RAW_HI 65200u
#define LCD_PRESENT_CONFIRM 8u  /* 连续 400ms 正常 -> 判定插回 */

// 动作在松开时处理：短按上下翻页、中键冻结；长按中键恢复面板、左右请求标定。
// 长短按以 1500 ms 区分；标定先请求电机失能，执行过程交给 INS 任务。
#define LCD_KEY_LONG_MIN_MS 1500u
/* 标定结果显示保持时间(ms) */
#define LCD_CALIB_RESULT_MS 3000u

static uint8_t lcd_page = 0;
static uint8_t lcd_frozen = 0;
static LCDKeyInput_t lcd_key_input;
static uint8_t lcd_adc_sample_valid;
/* 去抖状态: 见 lcd_key_update() 的说明 */

/* 按需标定的屏幕提示状态: 0=无提示(正常刷新) 1=显示"标定中" 2=显示结果 */
static uint8_t lcd_calib_msg = 0;
static uint32_t lcd_calib_result_until = 0;

volatile uint8_t lcd_init_done = 0;
volatile uint32_t lcd_heartbeat = 0;
volatile uint16_t lcd_key_raw = 0;
volatile uint16_t lcd_key_raw_min = 0xFFFFu; /* 上电以来见过的最小原始 ADC 值(诊断中键/左右档位) */
volatile uint8_t lcd_key_last = 0;
volatile uint8_t lcd_module_absent = 0;
volatile uint16_t lcd_key_spread = 0;
volatile uint32_t lcd_recover_count = 0;
volatile uint8_t lcd_task_entered = 0; /* 1 = 任务体已开始执行(排查任务是否被创建/调度) */
static volatile LCDTaskDiagnosticState_t lcd_bench_diag_state =
#if MC02_BENCH_LCD_DIAGNOSTICS
    LCD_TASK_DIAG_IDLE;
#else
    LCD_TASK_DIAG_DISABLED;
#endif
#if MC02_BENCH_LCD_DIAGNOSTICS
static uint8_t lcd_bench_diag_pattern;
static uint32_t lcd_bench_diag_started_ms;
static uint8_t lcd_bench_diag_ignore_release;
#endif

/* ---- 性能测量(供 OpenOCD 读取, 单位 us / 字节) ----
   lcd_prof_*_last/max: 单次操作耗时; lcd_prof_busy_us: 累计占用时间;
   lcd_prof_cycles: 统计到的刷新周期数; lcd_spi_bytes: 累计 SPI 字节数 */
volatile uint32_t lcd_prof_sample_last_us = 0;
volatile uint32_t lcd_prof_sample_max_us = 0;
volatile uint32_t lcd_prof_update_last_us = 0;
volatile uint32_t lcd_prof_update_max_us = 0;
volatile uint32_t lcd_prof_draw_last_us = 0;
volatile uint32_t lcd_prof_draw_max_us = 0;
volatile uint32_t lcd_prof_init_us = 0;
volatile uint32_t lcd_prof_recover_us = 0;
volatile uint32_t lcd_prof_busy_us = 0;
volatile uint32_t lcd_prof_cycles = 0;

// 每 25 个刷新周期记录一次任务运行计数，供调试读取；任务耗时比例用前后差值计算。
// 原始计数来自 getRunTimeCounterValue()，当前返回 CYCCNT/1000。
#define LCD_TASK_STATS_PERIOD 25u /* 25 个 200ms 周期 = 5s */
#define LCD_TASK_STATS_MAX PLATFORM_TASK_STATS_MAX_COUNT
#define LCD_TASK_STATS_LAYOUT 0x52543102u
volatile uint32_t lcd_stats_layout = LCD_TASK_STATS_LAYOUT;
volatile uint32_t lcd_stats_generation = 0u;
volatile uint32_t lcd_stats_sample_id = 0u;
volatile uint32_t lcd_stats_captured_ms = 0u;
volatile uint32_t lcd_stats_task_total = 0u;
volatile uint8_t lcd_stats_truncated = 0u;
volatile uint32_t lcd_stats_total = 0;
volatile uint8_t lcd_stats_count = 0;
_Alignas(uint32_t) volatile uint32_t lcd_stats_runtime[LCD_TASK_STATS_MAX] = {0};
_Alignas(uint32_t) volatile uint8_t lcd_stats_prio[LCD_TASK_STATS_MAX] = {0};
_Alignas(uint32_t) volatile char lcd_stats_name[LCD_TASK_STATS_MAX][PLATFORM_TASK_STATS_NAME_SIZE] = {{0}};
/* 各任务"历史最小剩余栈"（word, 4 字节/word；0 = 已经溢出）。
   统计由平台任务接口读取，当前 FreeRTOS 后端映射 usStackHighWaterMark。
   顺序与 lcd_stats_name[] / [cpu] 那一行一致 —— 改动任务栈大小前先看这个数字。 */
_Alignas(uint32_t) volatile uint16_t lcd_stats_stack[LCD_TASK_STATS_MAX] = {0};

/* 其它模块的分段探针(定义见各自的 .c), 用于 RTT 汇总行 */
#if MC02_FEATURE_MOTOR_CONTROL
extern volatile PlatformProbe_t motor_prof_all, motor_prof_dji, motor_prof_lk;
#endif
extern volatile PlatformProbe_t dji_prof_all;
#if MC02_SELECTED_DRIVER_DJI
extern volatile PlatformProbe_t dji_prof_pid, dji_prof_send;
#endif
extern volatile PlatformProbe_t ins_prof_all, ins_prof_read, ins_prof_ekf, ins_prof_temp;
extern volatile PlatformProbe_t can_prof_tx, can_prof_rx_isr;
extern volatile uint32_t can_prof_tx_spins, can_prof_tx_full, can_prof_rx_frames;

/* RTOS 计时单位: getRunTimeCounterValue() 返回 CYCCNT/1000, 即"千周期",
   480MHz 下一单位 = 1/480 ms ≈ 2.083us。换算成 us 用 *1000/480。 */
typedef struct
{
    char name[PLATFORM_TASK_STATS_NAME_SIZE];
    uint32_t runtime;
} LcdTaskPrev_t;

/* 统计快照拆成12条固定事件，每个200ms循环只提交一条，避免一次占满普通事件额度。 */
#define LCD_STATS_LOG_GROUPS 12u
static const BSPLogEventId_t lcd_stats_log_events[LCD_STATS_LOG_GROUPS] = {
    EV_MODULES_LCD_LCD_STATS_C_01, EV_MODULES_LCD_LCD_STATS_C_02,
    EV_MODULES_LCD_LCD_STATS_C_03, EV_MODULES_LCD_LCD_STATS_C_04,
    EV_MODULES_LCD_LCD_STATS_C_05, EV_MODULES_LCD_LCD_STATS_C_06,
    EV_MODULES_LCD_LCD_STATS_C_07, EV_MODULES_LCD_LCD_STATS_C_08,
    EV_MODULES_LCD_LCD_STATS_C_09, EV_MODULES_LCD_LCD_STATS_C_10,
    EV_MODULES_LCD_LCD_STATS_C_11, EV_MODULES_LCD_LCD_STATS_C_12};
static BSPLogField_t lcd_stats_log_fields[LCD_STATS_LOG_GROUPS][8];
static uint8_t lcd_stats_log_next_group;
static uint8_t lcd_stats_log_pending;

static size_t lcd_stats_text_length(const char *text, size_t limit)
{
    size_t length = 0u;
    if (text == NULL)
        return 0u;
    while (length < limit && text[length] != '\0')
        ++length;
    return length;
}

static void lcd_stats_set_pair(uint8_t group, uint8_t pair, const char *name,
                               size_t name_length, uint32_t value)
{
    uint8_t field = (uint8_t)(pair * 2u);
    lcd_stats_log_fields[group][field] = BSP_LOG_FIELD_TEXT(name, name_length);
    lcd_stats_log_fields[group][field + 1u] = BSP_LOG_FIELD_U32(value);
}

static void lcd_stats_set_literal(uint8_t group, uint8_t pair, const char *name, uint32_t value)
{
    lcd_stats_set_pair(group, pair, name, lcd_stats_text_length(name, BSP_LOG_TEXT_MAX), value);
}

static void lcd_stats_prepare_logs(uint32_t delta_total, const uint32_t *deltas)
{
    static const char *const segment_names[10] = {
        "mot_all", "mot_dji", "mot_lk", "dji_pid", "dji_send",
        "ins_all", "ins_read", "ins_ekf", "ins_temp", "lcd_upd"};
    static const char *const can_names[6] = {
        "tx_calls", "tx_full", "tx_spins", "tx_max", "rx_frames", "rx_isr_max"};
    static const char *const lcd_names[4] = {"upd", "clear", "init", "recover"};
    uint32_t segment_values[10] = {
#if MC02_FEATURE_MOTOR_CONTROL
        motor_prof_all.last_us, motor_prof_dji.last_us, motor_prof_lk.last_us,
#else
        0u, 0u, 0u,
#endif
#if MC02_SELECTED_DRIVER_DJI
        dji_prof_pid.last_us, dji_prof_send.last_us, ins_prof_all.last_us,
#else
        0u, 0u, ins_prof_all.last_us,
#endif
        ins_prof_read.last_us, ins_prof_ekf.last_us, ins_prof_temp.last_us,
        lcd_prof_update_last_us};
    uint32_t can_values[6] = {
        can_prof_tx.calls, can_prof_tx_full, can_prof_tx_spins, can_prof_tx.max_us,
        can_prof_rx_frames, can_prof_rx_isr.max_us};
    uint32_t lcd_values[4] = {
        lcd_prof_update_last_us, lcd_prof_clear_us, lcd_prof_init_us, lcd_prof_recover_us};

    for (uint8_t group = 0u; group < LCD_STATS_LOG_GROUPS; ++group)
    {
        for (uint8_t pair = 0u; pair < 4u; ++pair)
        {
            lcd_stats_log_fields[group][pair * 2u] = BSP_LOG_FIELD_TEXT_LITERAL("");
            lcd_stats_log_fields[group][pair * 2u + 1u] = BSP_LOG_FIELD_U32(0u);
        }
    }

    for (uint8_t i = 0u; i < lcd_stats_count; ++i)
    {
        uint32_t permille = (delta_total != 0u)
                                ? (uint32_t)(((uint64_t)deltas[i] * 1000u) / delta_total)
                                : 0u;
        lcd_stats_set_pair((uint8_t)(i / 4u), (uint8_t)(i % 4u),
                           (const char *)lcd_stats_name[i],
                           lcd_stats_text_length((const char *)lcd_stats_name[i],
                                                 PLATFORM_TASK_STATS_NAME_SIZE - 1u), permille);
    }
    for (uint8_t i = 0u; i < 10u; ++i)
        lcd_stats_set_literal((uint8_t)(3u + i / 4u), (uint8_t)(i % 4u),
                              segment_names[i], segment_values[i]);
    for (uint8_t i = 0u; i < 6u; ++i)
        lcd_stats_set_literal((uint8_t)(6u + i / 4u), (uint8_t)(i % 4u),
                              can_names[i], can_values[i]);
    for (uint8_t i = 0u; i < 4u; ++i)
        lcd_stats_set_literal(8u, i, lcd_names[i], lcd_values[i]);
    for (uint8_t i = 0u; i < lcd_stats_count; ++i)
        lcd_stats_set_pair((uint8_t)(9u + i / 4u), (uint8_t)(i % 4u),
                           (const char *)lcd_stats_name[i],
                           lcd_stats_text_length((const char *)lcd_stats_name[i],
                                                 PLATFORM_TASK_STATS_NAME_SIZE - 1u),
                           lcd_stats_stack[i]);

    lcd_stats_log_next_group = 0u;
    lcd_stats_log_pending = LCD_STATS_LOG_GROUPS;
}

static void lcd_stats_log_step(void)
{
    if (lcd_stats_log_pending == 0u)
        return;

    uint8_t group = lcd_stats_log_next_group;
    (void)BSPLogSubmit(lcd_stats_log_events[group], lcd_stats_log_fields[group], 8u);
    ++lcd_stats_log_next_group;
    --lcd_stats_log_pending;
}
// 记录各任务计数与剩余栈；第一次只保存基数，从第二次开始报告这一段时间的占用。
static void lcd_task_stats_snapshot(void)
{
    static LcdTaskPrev_t prev[LCD_TASK_STATS_MAX];
    static uint8_t prev_cnt = 0;
    PlatformTaskStatsInfo_t task_info;

    if (lcd_stats_layout != LCD_TASK_STATS_LAYOUT)
        return; // 调试工具声明了不兼容的布局时，不覆盖它正在读取的数组。

    const uint32_t previous_generation = lcd_stats_generation; // 保存上一次完整版本，失败时恢复。
    uint32_t generation = previous_generation + 1u; // 奇数版本告诉读取者数组正在更新。
    if ((generation & 1u) == 0u)
        ++generation; // 即使输入版本异常为奇数，也选一个奇数作为写入标记。
    lcd_stats_generation = generation; // 先发布写入状态，随后才改统计字段。
    PlatformMemoryBarrier(); // 确保调试读取者先看到奇数版本号，再看到新数组内容。
    if (!PlatformTaskStatsCapture(&task_info))
    {
        // 后端统计暂不可用时还原旧版本号，避免把旧任务列表冒充为新数据。
        lcd_stats_generation = previous_generation; // 捕获失败时数组尚未改写，可继续使用旧版本。
        return; // 没有新的任务记录时结束本轮快照。
    }
    const uint32_t n = task_info.captured_count; // 固定统计数组只处理后端实际捕获的记录。

    // 总量取各任务累计计数之和，计数单位沿用 CYCCNT/1000，下面按同名任务求差。
    uint32_t sum = 0u; // 在逐项复制时累计被记录任务的运行计数。
    lcd_stats_count = (uint8_t)n; // 发布本次数组中实际记录的任务数。
    lcd_stats_task_total = task_info.total_task_count; // 保存捕获前后观察到的任务总数。
    lcd_stats_truncated = (uint8_t)(task_info.total_task_count > n); // 标记固定容量未能容纳全部任务。
    lcd_stats_captured_ms = PlatformTimeNowMs(); // 为本次统计附上平台毫秒时间。
    ++lcd_stats_sample_id; // 每次成功捕获都推进样本编号。
    if (lcd_stats_sample_id == 0u)
        lcd_stats_sample_id = 1u; // 0 保留为尚未发布统计的标记。

    uint32_t deltas[LCD_TASK_STATS_MAX] = {0}; // 任务索引对应的占用差值先清零。
    uint32_t delta_total = 0u; // 汇总本轮能与旧任务名匹配的运行计数差。

    // 按任务逐项保存名称、优先级和剩余栈，再与上次记录配对。
    for (uint32_t i = 0; i < n; ++i)
    {
        PlatformTaskStatsEntry_t entry; // 用短时局部记录承接后端转换结果，避免第二份静态数组。
        if (!PlatformTaskStatsGetEntry(i, &entry))
            return; // 异常索引时保留奇数版本，表示当前数组未完成发布。
        lcd_stats_runtime[i] = entry.runtime_counter; // 保存任务累计运行计数。
        lcd_stats_prio[i] = (uint8_t)entry.priority; // 保留原调试布局中的8位优先级。
        lcd_stats_stack[i] = (uint16_t)entry.stack_high_water_words; // 保留原布局的剩余栈字数。
        memcpy((void *)lcd_stats_name[i], entry.name, sizeof(entry.name)); // 连同结束符复制完整任务名。
        sum += entry.runtime_counter; // 计算被捕获任务的累计运行计数之和。

        // 任务排列可能变化，按名字查找上一次记录，找到后只计算这一对的差值。
        for (uint8_t k = 0; k < prev_cnt; ++k)
        {
            if (strncmp((const char *)lcd_stats_name[i], prev[k].name, sizeof(prev[k].name)) == 0)
            {
                deltas[i] = lcd_stats_runtime[i] - prev[k].runtime;
                delta_total += deltas[i]; // 只累计同名任务两次快照之间的无符号差值。
                break; // 每条当前任务最多匹配一条历史记录。
            }
        }
    }
    lcd_stats_total = sum; // 完成逐条复制后再发布运行计数总量。

    if (prev_cnt != 0u)
        lcd_stats_prepare_logs(delta_total, deltas); // 首次只建立基线，之后才生成差值日志。

    // 本次结果作为下一轮的比较起点，名称连同结尾 0 一起保存。
    for (uint32_t i = 0; i < n; ++i)
    {
        for (uint8_t k = 0; k < PLATFORM_TASK_STATS_NAME_SIZE; ++k)
            prev[i].name[k] = (char)lcd_stats_name[i][k];
        prev[i].runtime = lcd_stats_runtime[i]; // 保存同一任务本次的累计计数基线。
    }
    prev_cnt = (uint8_t)n; // 下一次统计只匹配本次确实捕获到的任务。

    /* 清掉旧任务留下的尾项，避免调试器把缩短后的列表误读成完整快照。 */
    for (uint32_t i = n; i < LCD_TASK_STATS_MAX; ++i)
    {
        lcd_stats_runtime[i] = 0u; // 清除上次比本次更多的任务运行计数。
        lcd_stats_prio[i] = 0u; // 清除对应的旧优先级。
        lcd_stats_stack[i] = 0u; // 清除对应的旧栈余量。
        memset((void *)lcd_stats_name[i], 0, sizeof(lcd_stats_name[i])); // 清除旧名称避免误读。
    }

    PlatformMemoryBarrier(); // 确保数组先写完，再让调试读取者看到偶数版本。
    lcd_stats_generation = generation + 1u; // 偶数版本表示本次统计可以完整读取。
    BSPObservationCapture(); // 同步更新供现有只读工具读取的通用观测记录。
}

// 保存本次耗时及历史最大耗时，并加入 LCD 操作的累计时间。
static void lcd_prof_record(uint32_t t0, volatile uint32_t *last, volatile uint32_t *maxv)
{
    uint32_t dt = PlatformProbeElapsedUs(t0);
    *last = dt;
    if (dt > *maxv)
        *maxv = dt;
    lcd_prof_busy_us += dt;
}

// 读取有效 ADC 样本，按实际安装方向把电压范围换成用户看到的按键方向。
static LcdKey_e lcd_key_read(void)
{
    BSP_ADC_Sample_t sample;
    lcd_adc_sample_valid = (BSP_ADCGetSample(&sample) == BSP_ADC_STATUS_VALID);
    if (!lcd_adc_sample_valid)
    {
        lcd_key_raw = UINT16_MAX;
        return LCD_KEY_NONE;
    }

    uint16_t raw = sample.raw_key;
    lcd_key_raw = raw;

    /* 诊断: 记下上电以来见过的最小原始值。摇杆"中键"按下时 ADC 会趋近 0,
       所以即使某次按键没被识别, 事后读这个变量也能知道实际按下值是多少。 */
    if (raw < lcd_key_raw_min)
        lcd_key_raw_min = raw;

    // 中键按下时接近 0；直接按其范围识别，避免把真实按下误当成模组拔出。
    if (raw < LCD_KEY_MID_HI)
        return LCD_KEY_MID;

    /* 用户视角的上下 = 模组原始 right/left 触点 */
    if (raw >= LCD_KEY_RIGHT_LO && raw < LCD_KEY_RIGHT_HI)
        return LCD_KEY_UP;
    if (raw >= LCD_KEY_LEFT_LO && raw < LCD_KEY_LEFT_HI)
        return LCD_KEY_DOWN;

    // 模组旋转安装，原始 up/down 分别对应用户的左、右。
    if (raw >= LCD_KEY_UP_LO && raw < LCD_KEY_UP_HI)
        return LCD_KEY_LEFT;
    if (raw >= LCD_KEY_DOWN_LO && raw < LCD_KEY_DOWN_HI)
        return LCD_KEY_RIGHT;

    return LCD_KEY_NONE;
}

/* 采样是否落在某个合法档位带内(含空闲带)。落在档位之间的缝隙里 = 模组不在位。 */
static uint8_t lcd_raw_valid_level(uint16_t raw)
{
    if (raw > LCD_ABSENT_RAW_HI)
        return 0u;
    if (raw >= LCD_IDLE_RAW_MIN)
        return 1u;
    if (raw < LCD_KEY_MID_HI)
        return 1u;
    if (raw >= LCD_KEY_RIGHT_LO && raw < LCD_KEY_RIGHT_HI)
        return 1u;
    if (raw >= LCD_KEY_LEFT_LO && raw < LCD_KEY_LEFT_HI)
        return 1u;
    if (raw >= LCD_KEY_UP_LO && raw < LCD_KEY_UP_HI)
        return 1u;
    if (raw >= LCD_KEY_DOWN_LO && raw < LCD_KEY_DOWN_HI)
        return 1u;
    return 0u;
}

LCDStatus_t LCDTaskGetDiagnosticState(LCDTaskDiagnosticState_t *output)
{
    if (output == NULL)
        return LCD_BAD_ARGUMENT;
    *output = lcd_bench_diag_state;
    return LCD_OK;
}

static uint8_t lcd_bench_diag_active(void)
{
#if MC02_BENCH_LCD_DIAGNOSTICS
    return LCDTaskDiagnosticOwnsUi(lcd_bench_diag_state);
#else
    return 0u;
#endif
}

static uint8_t lcd_bench_diag_service(void)
{
#if MC02_BENCH_LCD_DIAGNOSTICS
    // 自测失败后停止测试，让用户仍可用中键恢复面板和继续翻页。
    if (!LCDTaskDiagnosticOwnsUi(lcd_bench_diag_state))
        return 0u;

    LCDState_t state;
    if (LCDGetState(&state) != LCD_OK || !state.ui_usable)
    {
        lcd_bench_diag_state = LCD_TASK_DIAG_FAILED;
        return 1u;
    }

    uint32_t now = PlatformTimeNowMs();
    if ((uint32_t)(now - lcd_bench_diag_started_ms) < 15000u)
        return 1u;

    if (lcd_bench_diag_pattern < 3u)
    {
        lcd_bench_diag_pattern++;
        if (LCD_UI_DrawDiagnosticPattern(lcd_bench_diag_pattern) != LCD_OK)
            lcd_bench_diag_state = LCD_TASK_DIAG_FAILED;
        else
            lcd_bench_diag_started_ms = now;
        return 1u;
    }

    LCDStatus_t status = LCD_UI_DrawStatic(0u);
    if (status == LCD_OK)
        status = LCD_UI_UpdateValues(0u);
    if (status == LCD_OK)
        lcd_bench_diag_state = LCD_TASK_DIAG_COMPLETE;
    else
        lcd_bench_diag_state = LCD_TASK_DIAG_FAILED;
    return 1u;
#else
    return 0u;
#endif
}

// 检查最近采样的电平，记住拔出状态；连续确认接入后重新初始化屏幕。
static void lcd_module_presence_check(void)
{
    static uint16_t hist[LCD_PRESENCE_WINDOW];
    static uint8_t hist_idx = 0;
    static uint8_t hist_filled = 0;
    static uint8_t present_streak = 0;
    static uint8_t invalid_seen = 0;

    if (!lcd_adc_sample_valid)
        return;

    uint16_t raw = lcd_key_raw;

    hist[hist_idx] = raw;
    hist_idx = (uint8_t)((hist_idx + 1u) % LCD_PRESENCE_WINDOW);
    if (hist_idx == 0u)
        hist_filled = 1u;

    uint8_t invalid = 0;
    if (!lcd_raw_valid_level(raw))
    {
        invalid = 1u;
        if (lcd_bench_diag_state == LCD_TASK_DIAG_RUNNING)
            lcd_bench_diag_state = LCD_TASK_DIAG_FAILED;
    }
    else if (hist_filled)
    {
        uint8_t idle_cnt = 0;
        uint16_t idle_min = 0xFFFFu;
        uint16_t idle_max = 0u;
        for (uint8_t i = 0; i < LCD_PRESENCE_WINDOW; ++i)
        {
            uint16_t s = hist[i];
            if (s >= LCD_IDLE_RAW_MIN)
            {
                idle_cnt++;
                if (s < idle_min)
                    idle_min = s;
                if (s > idle_max)
                    idle_max = s;
            }
        }
        lcd_key_spread = (idle_cnt > 0u) ? (uint16_t)(idle_max - idle_min) : 0u;

    // 空闲读数的最大、最小差仅供观察；空闲本身会波动，不据此宣布模组拔出。
    }

    if (invalid)
    {
        // 记住一次不合档位的读数；之后连续八次有效才恢复，避免反复重初始化。
        invalid_seen = 1u;
        present_streak = 0u;
        if (!lcd_module_absent)
        {
            lcd_module_absent = 1u;
            BSP_LOG_EVENT_2(EV_MODULES_LCD_LCD_TASK_C_01, BSP_LOG_FIELD_U32((unsigned)raw), BSP_LOG_FIELD_U32((unsigned)lcd_key_spread));
        }
    }
    else
    {
        if (present_streak < 0xFFu)
            present_streak++;

        if (invalid_seen && present_streak >= LCD_PRESENT_CONFIRM)
        {
            invalid_seen = 0u;
            present_streak = 0u;
            lcd_module_absent = 0u;
            lcd_recover_count++;
            BSP_LOG_EVENT_1(EV_MODULES_LCD_LCD_TASK_C_02, BSP_LOG_FIELD_U32((unsigned long)lcd_recover_count));
            uint32_t t0 = PlatformProbeStart();
            LCDStatus_t status = LCD_UI_Recover();
            lcd_init_done = (uint8_t)(status == LCD_OK);
            lcd_prof_recover_us = PlatformProbeElapsedUs(t0);
        }
    }
}

// 将连续确认后的事件转成页面或标定动作；只有松开事件会执行这些动作。
static void lcd_key_update(void)
{
    LcdKey_e key = lcd_key_read();
    uint32_t now = PlatformTimeNowMs();
    uint8_t key_sample_valid = (uint8_t)(lcd_adc_sample_valid && lcd_raw_valid_level(lcd_key_raw));
    LCDKeyEvent_t key_event;
    LCDKeyInputUpdate(&lcd_key_input, key_sample_valid, key, now, &key_event);

    if (!key_sample_valid)
    {
        lcd_key_last = 0u;
        return;
    }

    if (key_event.type == LCD_KEY_EVENT_DOWN)
    {
        lcd_key_last = (uint8_t)key_event.key;
#if MC02_BENCH_LCD_DIAGNOSTICS
        if (lcd_bench_diag_state == LCD_TASK_DIAG_RUNNING)
            lcd_bench_diag_ignore_release = 1u;
#endif
        BSP_LOG_EVENT_2(EV_MODULES_LCD_LCD_TASK_C_03, BSP_LOG_FIELD_U32((unsigned)key_event.key), BSP_LOG_FIELD_U32((unsigned)lcd_key_raw));
        return;
    }
    if (key_event.type != LCD_KEY_EVENT_RELEASED)
        return;

#if MC02_BENCH_LCD_DIAGNOSTICS
    // 诊断期间开始的按键即使在画面结束后才松开，也不转成翻页或标定请求。
    if (lcd_bench_diag_ignore_release != 0u)
    {
        lcd_bench_diag_ignore_release = 0u;
        lcd_key_last = 0u;
        return;
    }
#endif

    // 诊断画面按固定时长展示；期间按键只采样，不切页或请求标定。
    if (lcd_bench_diag_active())
        return;

    LcdKey_e released = key_event.key;
    uint32_t duration = key_event.duration_ms;
    lcd_key_last = 0u;
    if (released == LCD_KEY_NONE || duration < 30u)
        return;
    if (duration >= LCD_KEY_LONG_MIN_MS)
    {
        /* 长按左/右: 触发一次按需 IMU 标定 */
        if (released == LCD_KEY_LEFT || released == LCD_KEY_RIGHT)
        {
            if (BMI088CalibGetState() == BMI088_RECALIB_BUSY)
                return; /* 已经在标定中, 忽略本次 */

            BSP_LOG_EVENT_2(EV_MODULES_LCD_LCD_TASK_C_04, BSP_LOG_FIELD_CSTR_LIMIT(((released == LCD_KEY_LEFT) ? "L" : "R"), 48u), BSP_LOG_FIELD_U32((unsigned long)duration));
            RobotSafetyRequestCalib(1); /* 先失能: 标定期间电机不能动 */
            if (!BMI088CalibRequest())
            {
                BSP_LOG_EVENT_0(EV_MODULES_LCD_LCD_TASK_C_05);
                RobotSafetyRequestCalib(0);
                return;
            }
            /* 立刻出提示, 不等标定真正跑起来 —— 一开标定 CPU 就被抢走了 */
            (void)LCD_UI_ShowCalibMsg(1u);
            lcd_calib_msg = 1u;
            return;
        }

        /* 长按中键: 重初始化面板 + 顺便解冻 */
        if (released == LCD_KEY_MID)
        {
            lcd_recover_count++;
            // 重置显示时同时解除冻结，使恢复后的数值继续刷新。
            lcd_frozen = 0u;
            (void)LCD_UI_SetFrozen(0u);
            BSP_LOG_EVENT_1(EV_MODULES_LCD_LCD_TASK_C_06, BSP_LOG_FIELD_U32((unsigned long)lcd_recover_count));
            uint32_t t0 = PlatformProbeStart();
            LCDStatus_t status = LCD_UI_Recover();
            lcd_init_done = (uint8_t)(status == LCD_OK);
            lcd_prof_recover_us = PlatformProbeElapsedUs(t0);
        }
        return;
    }

    /* ===== 短按(<1.5s): 上下翻页, 中键冻结/恢复; 左右无短按功能 ===== */
    switch (released)
    {
    case LCD_KEY_UP:
        lcd_page = (uint8_t)((lcd_page + LCD_PAGE_COUNT - 1u) % LCD_PAGE_COUNT);
        break;
    case LCD_KEY_DOWN:
        lcd_page = (uint8_t)((lcd_page + 1u) % LCD_PAGE_COUNT);
        break;
    case LCD_KEY_MID:
        lcd_frozen = lcd_frozen ? 0u : 1u;
        (void)LCD_UI_SetFrozen(lcd_frozen);
        BSP_LOG_EVENT_1(EV_MODULES_LCD_LCD_TASK_C_07, BSP_LOG_FIELD_CSTR_LIMIT((lcd_frozen ? "on" : "off"), 48u));
        return;
    default:
        return;
    }

    uint32_t t0 = PlatformProbeStart();
    LCDStatus_t status = LCD_UI_DrawStatic(lcd_page);
    if (status == LCD_OK)
        status = LCD_UI_UpdateValues(lcd_page);
    if (status != LCD_OK)
        LCD_UI_InvalidateValueCache();
    lcd_prof_record(t0, &lcd_prof_draw_last_us, &lcd_prof_draw_max_us);
    BSP_LOG_EVENT_1(EV_MODULES_LCD_LCD_TASK_C_08, BSP_LOG_FIELD_U32((unsigned)(lcd_page + 1u)));
}

/**
 * @brief 按需标定的"提示/结果/收尾"状态机(由 LCD 任务周期调用)
 * @note  标定本体在 INS 任务里跑(BMI088CalibService), 这里只负责显示和
 *        把标定结果呈现 3s, 然后确认结果并回到 SAFE(必须重新走使能流程)。
 */
static void lcd_calib_service(void)
{
    uint8_t st = BMI088CalibGetState();

    if (st == BMI088_RECALIB_BUSY)
    {
        if (lcd_calib_msg != 1u)
        {
            (void)LCD_UI_ShowCalibMsg(1u); /* 标定被别的路径触发时也能补上提示 */
            lcd_calib_msg = 1u;
        }
        return;
    }

    if (st == BMI088_RECALIB_OK || st == BMI088_RECALIB_FAIL)
    {
        if (lcd_calib_msg != 2u)
        {
            (void)LCD_UI_ShowCalibMsg((st == BMI088_RECALIB_OK) ? 2u : 3u);
            lcd_calib_msg = 2u;
            lcd_calib_result_until = PlatformTimeNowMs() + LCD_CALIB_RESULT_MS;
            BSP_LOG_EVENT_2(EV_MODULES_LCD_LCD_TASK_C_09, BSP_LOG_FIELD_CSTR_LIMIT(((st == BMI088_RECALIB_OK) ? "OK" : "FAIL"), 48u), BSP_LOG_FIELD_U32((unsigned)bmi088_calib_attempts));
        }
        if ((int32_t)(PlatformTimeNowMs() - lcd_calib_result_until) >= 0)
        {
            BMI088CalibAckResult();
            RobotSafetyRequestCalib(0); /* 回 SAFE: 必须重新走正常使能流程 */
            lcd_calib_msg = 0u;
            (void)LCD_UI_ShowCalibMsg(0u); /* 清提示并重画整页 */
            BSP_LOG_EVENT_0(EV_MODULES_LCD_LCD_TASK_C_10);
        }
        return;
    }

    /* IDLE: 如果之前显示过提示(比如被异常路径打断), 这里收尾 */
    if (lcd_calib_msg != 0u)
    {
        lcd_calib_msg = 0u;
        RobotSafetyRequestCalib(0);
        (void)LCD_UI_ShowCalibMsg(0u);
    }
}

void StartLCDTASK(void const *argument)
{
    (void)argument;

    lcd_task_entered = 1;
    uint32_t t0 = PlatformProbeStart();
    LCDStatus_t init_status = LCD_UI_Init();
    lcd_prof_init_us = PlatformProbeElapsedUs(t0);
    lcd_init_done = (uint8_t)(init_status == LCD_OK);
    if (init_status != LCD_OK)
    {
#if defined(MC02_CONFIG_GENERATED)
        if (RobotBuildMarkFailure(ROBOT_MODULE_LCD) != 0u)
        {
            InitDiagnosticReportFailure(INIT_STAGE_APPLICATION, INIT_MODULE_TASKS,
                                        INIT_FAILURE_RESOURCE, INIT_RESOURCE_TASK_LCD);
            RobotFaultReport(ROBOT_FAULT_INIT_FAILURE);
            RobotSafetyRequestDisable();
            BSP_SafetyStartupLatchOutputsOff();
        }
        else
            InitDiagnosticReportDegraded(INIT_STAGE_APPLICATION, INIT_MODULE_TASKS,
                                         INIT_FAILURE_RESOURCE, INIT_RESOURCE_TASK_LCD);
#else
        InitDiagnosticReportFailure(INIT_STAGE_APPLICATION, INIT_MODULE_TASKS,
                                    INIT_FAILURE_RESOURCE, INIT_RESOURCE_TASK_LCD);
        RobotFaultReport(ROBOT_FAULT_INIT_FAILURE);
        RobotSafetyRequestDisable();
        BSP_SafetyStartupLatchOutputsOff();
#endif
        for (;;)
        {
            ++lcd_heartbeat;
            PlatformTaskDelayMs(100u);
        }
    }
    if (init_status == LCD_OK)
        BSP_LOG_EVENT_1(EV_MODULES_LCD_LCD_TASK_C_11, BSP_LOG_FIELD_U32((unsigned)(lcd_page + 1u)));
#if MC02_BENCH_LCD_DIAGNOSTICS
    if (init_status == LCD_OK && LCD_UI_DrawDiagnosticPattern(0u) == LCD_OK)
    {
        lcd_bench_diag_pattern = 0u;
        lcd_bench_diag_started_ms = PlatformTimeNowMs();
        lcd_bench_diag_ignore_release = 0u;
        lcd_bench_diag_state = LCD_TASK_DIAG_RUNNING;
    }
    else
    {
        lcd_bench_diag_state = LCD_TASK_DIAG_FAILED;
    }
#endif

    for (;;)
    {
        lcd_heartbeat++;
        /* 50ms 采样按键与模组在位状态, 200ms 刷新一次数值(5Hz) */
        for (uint8_t i = 0; i < 4u; ++i)
        {
            uint32_t ts = PlatformProbeStart();
            lcd_key_update();
            lcd_module_presence_check();
            lcd_prof_record(ts, &lcd_prof_sample_last_us, &lcd_prof_sample_max_us);
            PlatformTaskDelayMs(50u);
        }

        if (!lcd_bench_diag_active())
            lcd_calib_service();
        uint8_t diagnostic_active = lcd_bench_diag_service();

        /* 标定提示正在显示时不刷新数值区, 否则会把提示盖掉 */
        LCDState_t lcd_state;
        uint8_t lcd_usable = (uint8_t)(LCDGetState(&lcd_state) == LCD_OK && lcd_state.ui_usable);
        if (!diagnostic_active && lcd_usable && !lcd_frozen && !lcd_module_absent && lcd_calib_msg == 0u)
        {
            uint32_t tu = PlatformProbeStart();
            LCDStatus_t update_status = LCD_UI_UpdateValues(lcd_page);
            if (update_status != LCD_OK)
                LCD_UI_InvalidateValueCache();
            lcd_prof_record(tu, &lcd_prof_update_last_us, &lcd_prof_update_max_us);
            lcd_prof_cycles++;
        }

        if ((lcd_heartbeat % LCD_TASK_STATS_PERIOD) == 0u)
            lcd_task_stats_snapshot();

        lcd_stats_log_step();
    }
}
