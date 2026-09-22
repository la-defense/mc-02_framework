#include "lcd_task.h"
#include "lcd_ui.h"
#include "lcd_buf.h"
#include "bsp_adc.h"
#include "bsp_log.h"
#include "bsp_dwt.h"
#include "cmsis_os.h"
#include "main.h"
#include "task.h"
#include "bmi088.h"
#include "robot_safety.h"
#include <string.h>

typedef enum
{
    LCD_KEY_NONE = 0,
    LCD_KEY_MID,
    LCD_KEY_UP,    /* 用户视角"上" = 模组原始 right 触点 */
    LCD_KEY_DOWN,  /* 用户视角"下" = 模组原始 left  触点 */
    LCD_KEY_LEFT,  /* 用户视角"左" = 模组原始 up    触点 */
    LCD_KEY_RIGHT, /* 用户视角"右" = 模组原始 down  触点 */
} LcdKey_e;

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

/* 模组在位检测(热插拔恢复):
   模组拔掉后 PA5(按键分压输入)没有驱动, 重新插上时模组已经掉电复位,
   必须重发初始化序列才可能重新显示。
   实测(2026-09-21, 拔掉模组): PA5 稳定在 ~5900(0.29V), 既不越界也不跳动,
   所以"越界/极差"判据抓不到, 必须按"极性档位带"判断:
     合法档位 = mid(0~3200) / right(11200~16000) / left(24000~28800)
                / up(35200~40000) / down(44800~56000) / 空闲(>=58000)
   落在档位之间的缝隙(如拔掉时的 5900)即视为不在位。 */
#define LCD_PRESENCE_WINDOW 10u
#define LCD_PRESENCE_SPREAD_MAX 1500u
#define LCD_IDLE_RAW_MIN 58000u
#define LCD_ABSENT_RAW_HI 65200u
#define LCD_PRESENT_CONFIRM 8u  /* 连续 400ms 正常 -> 判定插回 */

/* 按键定义(全部在"松开"时判定, 以 1.5s 为界分成"短按/长按", 没有中间的空白区):
     < 1.5s  上/下       : 翻页
     < 1.5s  中键        : 冻结 / 恢复
     < 1.5s  左/右       : (无功能)
     >= 1.5s 中键        : 手动重初始化面板(拔插后没自动恢复时的兜底), 并解除冻结
     >= 1.5s 左/右       : 进入按需 IMU 标定(进 CALIB, 电机失能, 保持静止)
   为什么把标定从中键挪到左右:
     中键的长按已经被"面板重初始化"占用, 再塞"标定"会让人分不清按了多久会发生什么;
     而且用户最常用的就是中键, 误触标定会让机器人突然失能。
     左右键原本没有任何功能(而且之前根本没被识别), 拿来做"破坏性动作"最合适。 */
#define LCD_KEY_LONG_MIN_MS 1500u
/* 按键去抖: 连续多少次采样一致才认账(采样周期 50ms) */
#define LCD_KEY_DEBOUNCE_CNT 3u

/* 标定结果显示保持时间(ms) */
#define LCD_CALIB_RESULT_MS 3000u

static uint8_t lcd_page = 0;
static uint8_t lcd_frozen = 0;
static LcdKey_e key_active = LCD_KEY_NONE;
static uint32_t key_press_ms = 0;
/* 去抖状态: 见 lcd_key_update() 的说明 */
static LcdKey_e key_cand = LCD_KEY_NONE;
static uint8_t key_cand_cnt = 0;
static uint8_t key_release_cnt = 0;

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

/* ---- 全系统 CPU 占用快照(每 5s 采一次, 供 OpenOCD 读取) ----
   RTOS 运行时间统计依赖 Core/Src/freertos.c 里 getRunTimeCounterValue() = DWT->CYCCNT。
   任务占用率 = Δlcd_stats_runtime[i] / Δlcd_stats_total。 */
#define LCD_TASK_STATS_PERIOD 25u /* 25 个 200ms 周期 = 5s */
#define LCD_TASK_STATS_MAX 12u
volatile uint32_t lcd_stats_total = 0;
volatile uint8_t lcd_stats_count = 0;
volatile uint32_t lcd_stats_runtime[LCD_TASK_STATS_MAX] = {0};
volatile uint8_t lcd_stats_prio[LCD_TASK_STATS_MAX] = {0};
volatile char lcd_stats_name[LCD_TASK_STATS_MAX][12] = {{0}};

/* 其它模块的分段探针(定义见各自的 .c), 用于 RTT 汇总行 */
extern volatile DWT_Probe_t motor_prof_all, motor_prof_dji, motor_prof_lk;
extern volatile DWT_Probe_t dji_prof_all, dji_prof_pid, dji_prof_send;
extern volatile DWT_Probe_t ins_prof_all, ins_prof_read, ins_prof_ekf, ins_prof_temp;
extern volatile DWT_Probe_t can_prof_tx, can_prof_rx_isr;
extern volatile uint32_t can_prof_tx_spins, can_prof_tx_full, can_prof_rx_frames;

/* RTOS 计时单位: getRunTimeCounterValue() 返回 CYCCNT/1000, 即"千周期",
   480MHz 下一单位 = 1/480 ms ≈ 2.083us。换算成 us 用 *1000/480。 */
#define LCD_STATS_UNIT_TO_US(x) ((uint32_t)(((uint64_t)(x) * 1000u) / 480u))

typedef struct
{
    char name[12];
    uint32_t runtime;
} LcdTaskPrev_t;

/* 每 5s 打印一次: 第一行各任务占用率(千分比, RTT 不支持浮点), 第二行分段耗时 */
static void lcd_cpu_report(uint32_t delta_total, const uint32_t *deltas)
{
    char line[256];
    int off = snprintf(line, sizeof(line), "[cpu] win=%lums ",
                       (unsigned long)LCD_STATS_UNIT_TO_US(delta_total) / 1000u);

    for (uint8_t i = 0; i < lcd_stats_count && off > 0 && off < (int)sizeof(line); ++i)
    {
        uint32_t permille = (delta_total != 0u)
                                ? (uint32_t)(((uint64_t)deltas[i] * 1000u) / delta_total)
                                : 0u;
        int w = snprintf(line + off, sizeof(line) - (size_t)off, "%s=%lu ",
                         (const char *)lcd_stats_name[i], (unsigned long)permille);
        if (w < 0)
            break;
        off += w;
    }
    LOGINFO("%s (千分比, 1000=100%%)", line);

    LOGINFO("[seg] mot_all=%lu mot_dji=%lu mot_lk=%lu dji_pid=%lu dji_send=%lu ins_all=%lu ins_read=%lu ins_ekf=%lu ins_temp=%lu lcd_upd=%lu (us, last)",
            (unsigned long)motor_prof_all.last_us, (unsigned long)motor_prof_dji.last_us,
            (unsigned long)motor_prof_lk.last_us, (unsigned long)dji_prof_pid.last_us,
            (unsigned long)dji_prof_send.last_us, (unsigned long)ins_prof_all.last_us,
            (unsigned long)ins_prof_read.last_us, (unsigned long)ins_prof_ekf.last_us,
            (unsigned long)ins_prof_temp.last_us, (unsigned long)lcd_prof_update_last_us);
    LOGINFO("[can] tx_calls=%lu tx_full=%lu tx_spins=%lu tx_max=%lu rx_frames=%lu rx_isr_max=%lu (us)",
            (unsigned long)can_prof_tx.calls, (unsigned long)can_prof_tx_full,
            (unsigned long)can_prof_tx_spins, (unsigned long)can_prof_tx.max_us,
            (unsigned long)can_prof_rx_frames, (unsigned long)can_prof_rx_isr.max_us);
    LOGINFO("[lcd] upd=%lu clr=%lu init=%lu recover=%lu (us)",
            (unsigned long)lcd_prof_update_last_us, (unsigned long)lcd_prof_clear_us,
            (unsigned long)lcd_prof_init_us, (unsigned long)lcd_prof_recover_us);
}

static void lcd_task_stats_snapshot(void)
{
    static TaskStatus_t st[LCD_TASK_STATS_MAX];
    static LcdTaskPrev_t prev[LCD_TASK_STATS_MAX];
    static uint8_t prev_cnt = 0;
    uint32_t total = 0;

    UBaseType_t n = uxTaskGetSystemState(st, LCD_TASK_STATS_MAX, &total);
    if (n > LCD_TASK_STATS_MAX)
        n = LCD_TASK_STATS_MAX;

    /* 用各任务累计运行时间之和作为总时间: 计数器单位是 us(见 freertos.c),
       单个任务要 4295s 才回绕, 比直接读 portGET_RUN_TIME_COUNTER_VALUE() 稳。 */
    (void)total;
    uint32_t sum = 0;
    for (UBaseType_t i = 0; i < n; ++i)
        sum += st[i].ulRunTimeCounter;
    lcd_stats_total = sum;
    lcd_stats_count = (uint8_t)n;

    uint32_t deltas[LCD_TASK_STATS_MAX] = {0};
    uint32_t delta_total = 0;

    for (UBaseType_t i = 0; i < n; ++i)
    {
        lcd_stats_runtime[i] = st[i].ulRunTimeCounter;
        lcd_stats_prio[i] = (uint8_t)st[i].uxCurrentPriority;
        for (uint8_t k = 0; k < 12u; ++k)
            lcd_stats_name[i][k] = '\0';
        for (uint8_t k = 0; k < 11u; ++k)
        {
            char c = st[i].pcTaskName[k];
            lcd_stats_name[i][k] = c;
            if (c == '\0')
                break;
        }

        /* 与上一次快照按任务名配对, 求增量; 无符号差值天然兼容计数器回绕 */
        for (uint8_t k = 0; k < prev_cnt; ++k)
        {
            if (strncmp((const char *)lcd_stats_name[i], prev[k].name, sizeof(prev[k].name)) == 0)
            {
                deltas[i] = lcd_stats_runtime[i] - prev[k].runtime;
                delta_total += deltas[i];
                break;
            }
        }
    }

    if (prev_cnt != 0u)
        lcd_cpu_report(delta_total, deltas);

    for (uint8_t i = 0; i < n; ++i)
    {
        for (uint8_t k = 0; k < 12u; ++k)
            prev[i].name[k] = (char)lcd_stats_name[i][k];
        prev[i].runtime = lcd_stats_runtime[i];
    }
    prev_cnt = (uint8_t)n;
}

static void lcd_prof_record(uint32_t t0, volatile uint32_t *last, volatile uint32_t *maxv)
{
    uint32_t dt = DWT_ProbeElapsedUs(t0); /* 探针统一直接读 CYCCNT */
    *last = dt;
    if (dt > *maxv)
        *maxv = dt;
    lcd_prof_busy_us += dt;
}

static LcdKey_e lcd_key_read(void)
{
    uint16_t raw = BSP_ADCGetRawKey();
    lcd_key_raw = raw;

    /* 诊断: 记下上电以来见过的最小原始值。摇杆"中键"按下时 ADC 会趋近 0,
       所以即使某次按键没被识别, 事后读这个变量也能知道实际按下值是多少。 */
    if (raw < lcd_key_raw_min)
        lcd_key_raw_min = raw;

    /* 中键: 按下时 ADC 趋近 0。
       原来这里还有一条 "raw < LCD_ABSENT_RAW_LO(100) 判为悬空" 的保护, 那是
       为了防"模组拔掉后引脚浮空" —— 但实测拔掉模组时读数稳定在 ~5900(落在
       档位缝隙里, 本来就会被判成 NONE), 根本不会出现在 0 附近; 那条保护反而把
       中键按下的真实值吃掉了 → 短按/长按中键时灵时不灵。这里去掉它,
       也**不能**用 lcd_module_absent 来兜底: 按住按键时读数本来就会大幅变化,
       在位检测偶尔会误判成"模组拔出", 那样会把按键整个屏蔽掉(用户体感:
       按什么都没反应)。 */
    if (raw < LCD_KEY_MID_HI)
        return LCD_KEY_MID;

    /* 用户视角的上下 = 模组原始 right/left 触点 */
    if (raw >= LCD_KEY_RIGHT_LO && raw < LCD_KEY_RIGHT_HI)
        return LCD_KEY_UP;
    if (raw >= LCD_KEY_LEFT_LO && raw < LCD_KEY_LEFT_HI)
        return LCD_KEY_DOWN;

    /* 用户视角的左右 = 模组原始 up/down 触点。
       模组逆时针旋转 90° 安装: 局部 +y(up) → 用户 -x(左), 局部 -y(down) → 用户 +x(右)。
       之前这里漏了这两档, 所以摇杆左右按下去没有任何反应(直接落到 LCD_KEY_NONE)。 */
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

static void lcd_module_presence_check(void)
{
    static uint16_t hist[LCD_PRESENCE_WINDOW];
    static uint8_t hist_idx = 0;
    static uint8_t hist_filled = 0;
    static uint8_t present_streak = 0;
    static uint8_t invalid_seen = 0;

    uint16_t raw = lcd_key_raw;

    hist[hist_idx] = raw;
    hist_idx = (uint8_t)((hist_idx + 1u) % LCD_PRESENCE_WINDOW);
    if (hist_idx == 0u)
        hist_filled = 1u;

    uint8_t invalid = 0;
    if (!lcd_raw_valid_level(raw))
    {
        invalid = 1u;
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

        /* 这里原来还有一条"空闲电平极差过大 → 判模组不在位"的判据(spread > 1500)。
           实测本模组的空闲电平本身就会在 58000~65000 之间抖动(极差 6000+),
           于是好端端的模组被反复判成"不在位" → 数值区停止刷新 + 误触发重初始化。
           插拔检测其实只靠"读数落在档位缝隙里"(拔掉时稳定在 ~5900)就够了,
           所以这条判据去掉, lcd_key_spread 仍保留仅供观察。 */
    }

    if (invalid)
    {
        /* 只要出现过一次"缝隙档位/剧烈跳动", 就记下这次拔插事件,
           之后电平回到合法档位并稳定 400ms 就重初始化面板。
           这样即使拔插很快(只抓到一两帧), 也能恢复。 */
        invalid_seen = 1u;
        present_streak = 0u;
        if (!lcd_module_absent)
        {
            lcd_module_absent = 1u;
            LOGINFO("[lcd] module absent? raw=%u spread=%u", (unsigned)raw, (unsigned)lcd_key_spread);
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
            LOGINFO("[lcd] module back, re-init panel (recover=%lu)", (unsigned long)lcd_recover_count);
            uint32_t t0 = DWT_ProbeStart();
            LCD_UI_Recover();
            lcd_prof_recover_us = DWT_ProbeElapsedUs(t0);
        }
    }
}

static void lcd_key_update(void)
{
    LcdKey_e key = lcd_key_read();
    uint32_t now = HAL_GetTick();

    /* ---- 去抖(2026-09-22 加) ----
       摇杆的 ADC 读数本身抖动很大(实测空闲时极差可达 6000+), 单次采样可能瞬间
       落进相邻档位或缝隙。不做去抖会有两个后果:
         ① 长按过程中偶发一次"读不到键", 长按就被截断成好几次短按 → 长按时灵时不灵;
         ② 空闲抖动可能被当成按键, 甚至误触发"长按左右 = 进标定失能"。
       规则: 按下要连续 DEBOUNCE_CNT 次采到同一个键; 松开要连续 DEBOUNCE_CNT 次
       采到"无按键"。采样周期 50ms, 所以确认时间是 150ms。 */
    if (key_active == LCD_KEY_NONE)
    {
        if (key == LCD_KEY_NONE)
        {
            key_cand = LCD_KEY_NONE;
            key_cand_cnt = 0u;
            return;
        }
        if (key != key_cand)
        {
            key_cand = key;
            key_cand_cnt = 1u;
            return;
        }
        if (++key_cand_cnt < LCD_KEY_DEBOUNCE_CNT)
            return;

        key_active = key;
        key_press_ms = now;
        lcd_key_last = (uint8_t)key;
        key_release_cnt = 0u;
        LOGINFO("[lcd] key down %u raw=%u", (unsigned)key, (unsigned)lcd_key_raw);
        return;
    }

    /* 已按下: 按下期间的偶发抖动一律忽略, 只有连续 DEBOUNCE_CNT 次"无按键"才算松开 */
    if (key != LCD_KEY_NONE)
    {
        key_release_cnt = 0u;
        return;
    }
    if (++key_release_cnt < LCD_KEY_DEBOUNCE_CNT)
        return;

    key_cand = LCD_KEY_NONE;
    key_cand_cnt = 0u;
    key_release_cnt = 0u;

    /* 确认松开: 短按 = 翻页/冻结; 长按中键 = 重初始化面板; 长按左右 = 按需标定 */
    LcdKey_e released = key_active;
    uint32_t duration = now - key_press_ms;
    key_active = LCD_KEY_NONE;
    lcd_key_last = 0u;

    if (released == LCD_KEY_NONE || duration < 30u)
        return;

    /* ===== 长按(>=1.5s) =====
       注: 原来这里分了 [1s,1.5s) 和 [1.5s,3s) 两段, 其中 [1s,1.5s) 是"什么都不做"的
       黑洞 —— 用户按 1 秒多钟松手时既不翻页也不冻结, 看起来就是"按键没反应"。
       现在统一成"<1.5s 一律当短按, >=1.5s 一律当长按", 没有空洞区间。 */
    if (duration >= LCD_KEY_LONG_MIN_MS)
    {
        /* 长按左/右: 触发一次按需 IMU 标定 */
        if (released == LCD_KEY_LEFT || released == LCD_KEY_RIGHT)
        {
            if (BMI088CalibGetState() == BMI088_RECALIB_BUSY)
                return; /* 已经在标定中, 忽略本次 */

            LOGINFO("[lcd] long press %s %lums -> request IMU calibration",
                    (released == LCD_KEY_LEFT) ? "L" : "R", (unsigned long)duration);
            RobotSafetyRequestCalib(1); /* 先失能: 标定期间电机不能动 */
            if (!BMI088CalibRequest())
            {
                LOGWARNING("[lcd] calibration request rejected");
                RobotSafetyRequestCalib(0);
                return;
            }
            /* 立刻出提示, 不等标定真正跑起来 —— 一开标定 CPU 就被抢走了 */
            LCD_UI_ShowCalibMsg(1u);
            lcd_calib_msg = 1u;
            return;
        }

        /* 长按中键: 重初始化面板 + 顺便解冻 */
        if (released == LCD_KEY_MID)
        {
            lcd_recover_count++;
            /* 长按中键的语义是"重置显示", 顺便解冻 —— 否则用户习惯了长按,
               冻结之后怎么长按都解不开(长按被"重初始化"吃掉了)。 */
            lcd_frozen = 0u;
            LCD_UI_SetFrozen(0u);
            LOGINFO("[lcd] manual re-init (recover=%lu)", (unsigned long)lcd_recover_count);
            uint32_t t0 = DWT_ProbeStart();
            LCD_UI_Recover();
            lcd_prof_recover_us = DWT_ProbeElapsedUs(t0);
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
        LCD_UI_SetFrozen(lcd_frozen);
        LOGINFO("[lcd] freeze %s", lcd_frozen ? "on" : "off");
        return;
    default:
        return;
    }

    uint32_t t0 = DWT_ProbeStart();
    LCD_UI_DrawStatic(lcd_page);
    LCD_UI_UpdateValues(lcd_page);
    lcd_prof_record(t0, &lcd_prof_draw_last_us, &lcd_prof_draw_max_us);
    LOGINFO("[lcd] page %u", (unsigned)(lcd_page + 1u));
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
            LCD_UI_ShowCalibMsg(1u); /* 标定被别的路径触发时也能补上提示 */
            lcd_calib_msg = 1u;
        }
        return;
    }

    if (st == BMI088_RECALIB_OK || st == BMI088_RECALIB_FAIL)
    {
        if (lcd_calib_msg != 2u)
        {
            LCD_UI_ShowCalibMsg((st == BMI088_RECALIB_OK) ? 2u : 3u);
            lcd_calib_msg = 2u;
            lcd_calib_result_until = HAL_GetTick() + LCD_CALIB_RESULT_MS;
            LOGINFO("[lcd] calib result: %s (rounds=%u)",
                    (st == BMI088_RECALIB_OK) ? "OK" : "FAIL",
                    (unsigned)bmi088_calib_attempts);
        }
        if ((int32_t)(HAL_GetTick() - lcd_calib_result_until) >= 0)
        {
            BMI088CalibAckResult();
            RobotSafetyRequestCalib(0); /* 回 SAFE: 必须重新走正常使能流程 */
            lcd_calib_msg = 0u;
            LCD_UI_ShowCalibMsg(0u); /* 清提示并重画整页 */
            LOGINFO("[lcd] calib UI done, back to normal page");
        }
        return;
    }

    /* IDLE: 如果之前显示过提示(比如被异常路径打断), 这里收尾 */
    if (lcd_calib_msg != 0u)
    {
        lcd_calib_msg = 0u;
        RobotSafetyRequestCalib(0);
        LCD_UI_ShowCalibMsg(0u);
    }
}

void StartLCDTASK(void const *argument)
{
    (void)argument;

    lcd_task_entered = 1;
    uint32_t t0 = DWT_ProbeStart();
    LCD_UI_Init();
    lcd_prof_init_us = DWT_ProbeElapsedUs(t0);
    lcd_init_done = 1;
    LOGINFO("[lcd] init done, page=%u", (unsigned)(lcd_page + 1u));

    for (;;)
    {
        lcd_heartbeat++;
        /* 50ms 采样按键与模组在位状态, 200ms 刷新一次数值(5Hz) */
        for (uint8_t i = 0; i < 4u; ++i)
        {
            uint32_t ts = DWT_ProbeStart();
            lcd_key_update();
            lcd_module_presence_check();
            lcd_prof_record(ts, &lcd_prof_sample_last_us, &lcd_prof_sample_max_us);
            osDelay(50);
        }

        lcd_calib_service();

        /* 标定提示正在显示时不刷新数值区, 否则会把提示盖掉 */
        if (!lcd_frozen && !lcd_module_absent && lcd_calib_msg == 0u)
        {
            uint32_t tu = DWT_ProbeStart();
            LCD_UI_UpdateValues(lcd_page);
            lcd_prof_record(tu, &lcd_prof_update_last_us, &lcd_prof_update_max_us);
            lcd_prof_cycles++;
        }

        if ((lcd_heartbeat % LCD_TASK_STATS_PERIOD) == 0u)
            lcd_task_stats_snapshot();
    }
}
