#ifndef _BSP_LOG_H
#define _BSP_LOG_H

#include "SEGGER_RTT.h"
#include "SEGGER_RTT_Conf.h"
#include <stdio.h>
#include <stdint.h>
#include <string.h> /* LOG_PROTO 里用 memcpy 拼接结尾的换行+复位色码 */

#define BUFFER_INDEX 0

/**
 * @brief 日志系统初始化
 *
 */
void BSPLogInit();

/**
 * @brief 日志功能原型,供下面的LOGI,LOGW,LOGE等使用
 *
 */
/* ---------------- 日志格式化的实现方式 (2026-09-26, LOG-02) ----------------
   以前: LOG_PROTO 把"颜色 + 等级 + format + 复位码"四段拼成一个格式串, 直接交给
         SEGGER 的 RTT printf, 而且 **format 后面还挂着一个 %s**(复位色码)。
         参数由"谁认识格式符谁消费"决定, 于是 format 里出现 SEGGER 不认识的 %f 时,
         它连参数都不消费, 后面那个 %s 就取到了 float 的位模式当指针解引用
         -> BusFault -> HardFault(完整现场见 LOG-01)。
   现在(三层保险):
     ① 先用 newlib 的 snprintf 把整行格式化到"栈上的缓冲区", 再用
        SEGGER_RTT_WriteString 输出纯字符串 —— 参数由 newlib 按标准 C 规则消费;
     ② 结尾的 "\r\n" + 复位色码 **不再作为格式串里的 %s**, 而是格式化完之后用
        memcpy 拼到缓冲区尾部 —— 这样"用户自己的 format"后面永远不再有别人的
        格式符, 就算真写了 newlib 也不认识的格式符, 受影响的也只有那一项,
        不可能再拖累后面的参数, 更不可能出现"读非法指针"。
         (这一条才是 LOG-01 那类崩溃的结构性根因: 错不在 %f, 而在我们自己在
          用户格式串后面又追加了一个 %s。)
   其它收益:
     - 缓冲区在栈上, 每个日志调用点天然独立, 不需要锁/关中断;
     - snprintf 自带截断, 超长日志不越界;
     - 宏展开后 snprintf 的格式串确实是一个字符串字面量, 所以 GCC 的 -Wformat
       能真正检查每个 LOGINFO/LOGWARNING/LOGERROR 的格式符与参数是否匹配
       (以前直接喂 SEGGER 私有 printf 是查不出来的)。
   代价: 每个日志点占 LOG_BUF_SIZE(256) 字节栈空间 —— 各任务栈要留够余量
         (实测 motortask 因此从 256 加到 512 words, daemon 从 128 加到 512 words;
          每个任务够不够看 RTT 的 `[stk]` 行)。*/
/* 缓冲大小按"本工程最长的日志行"定: 最长的是 lcd_task.c 里的
   `[seg] ...`(10 个 %lu, 最坏 ~230 字节) 与 `[cpu] ...`(最多 12 个任务)。
   128 字节会把这些诊断行截断(改造后 GCC 的 -Wformat-truncation 会直接报出来),
   所以取 256。代价是每个日志调用点占 256 字节栈 —— 各任务栈要留够余量。 */
#define LOG_BUF_SIZE 256u /* 整行日志缓冲(放在栈上) */
#define LOG_BODY_TAIL 8u  /* 给结尾的 "\r\n" + 复位色码预留的位置 */

/* 缓冲区尾部预留量必须放得下 "\r\n" + RTT_CTRL_RESET(+结束符) */
_Static_assert(sizeof("\r\n" RTT_CTRL_RESET) <= LOG_BODY_TAIL,
               "LOG_BODY_TAIL 放不下结尾的换行+复位色码");

#define LOG_PROTO(type, color, format, ...)                             \
        do {                                                            \
            char _log_buf[LOG_BUF_SIZE];                                \
            int _log_n = snprintf(_log_buf,                             \
                                  LOG_BUF_SIZE - LOG_BODY_TAIL,         \
                                  "  %s%s" format,                      \
                                  color, type, ##__VA_ARGS__);          \
            if (_log_n < 0)                                             \
                _log_n = 0; /* 格式化出错: 只留结尾的复位码 */           \
            else if ((unsigned)_log_n > LOG_BUF_SIZE - LOG_BODY_TAIL - 1u) \
                _log_n = LOG_BUF_SIZE - LOG_BODY_TAIL - 1u; /* 被截断 */  \
            memcpy(_log_buf + _log_n, "\r\n" RTT_CTRL_RESET,            \
                   sizeof("\r\n" RTT_CTRL_RESET));                      \
            SEGGER_RTT_WriteString(BUFFER_INDEX, _log_buf);             \
        } while (0)

/*----------------------------------------下面是日志输出的接口-------------------------------------------------*/

/* 清屏 */
#define LOG_CLEAR() SEGGER_RTT_WriteString(0, "  " RTT_CTRL_CLEAR)

/* 无颜色日志输出 */
#define LOG(format, ...) LOG_PROTO("", "", format, ##__VA_ARGS__)

/**
 *  有颜色格式日志输出,建议使用这些宏来输出日志
 *  @attention newlib-nano 默认**不带**浮点格式化(库里的 `_printf_float` 不会被链进来),
 *             本工程已在 CMake 里加了 `-u _printf_float` 把它显式链上, 所以 %f 能正常打印;
 *             但浮点格式化比较慢(几十 µs 量级), 热路径上仍建议放大成整数打印(如 temp_x100)。
 *  @note 在release版本上车使用时,与makefile中添加的宏DISABLE_LOG_SYSTEM一起使用,可以关闭日志系统
 */
#if DISABLE_LOG_SYSTEM
#define LOGINFO(format, ...) 
#define LOGWARNING(format, ...) 
#define LOGERROR(format, ...) 
#else
// information level
#define LOGINFO(format, ...) LOG_PROTO("I:", RTT_CTRL_TEXT_BRIGHT_GREEN, format, ##__VA_ARGS__)
// warning level
#define LOGWARNING(format, ...) LOG_PROTO("W:", RTT_CTRL_TEXT_BRIGHT_YELLOW, format, ##__VA_ARGS__)
// error level
#define LOGERROR(format, ...) LOG_PROTO("E:", RTT_CTRL_TEXT_BRIGHT_RED, format, ##__VA_ARGS__)
#endif //  DISABLE_LOG_SYSTEM

/* ---------------- 热路径日志限速(2026-09) ----------------
   高频路径(任务超时、电机离线、CAN 丢帧、串口错误…)如果不限速, 会把 CPU 和 1KB 的 RTT
   缓冲一起刷爆(本工程实测过: 每秒上千条, 连占用率汇总行都被挤掉)。

   用法:
       static LogRateLimit_t rl = {0};
       if (LogRateLimitAllow(&rl, 1000u))   // 同一类消息最多 1 条/秒
           LOGWARNING("[xxx] 出问题了 (累计 %lu 次, 期间限速 %lu 条)",
                      (unsigned long)rl.total, (unsigned long)rl.dropped);

   注意: 该组件用 HAL_GetTick() 计时; 调度器启动前 TIM23 时间基准还没跑起来(HAL_GetTick
   不递增), 启动阶段的限速日志会只打第一条 —— 这是可接受的(启动阶段不是热点)。 */
typedef struct
{
    uint32_t last_ms; /* 上次真正打印的时间 */
    uint32_t total;   /* 累计事件数(每次调用 +1) */
    uint32_t printed; /* 上次打印时的累计数 */
    uint32_t dropped; /* 上次打印至今被限速掉的条数(打印时读取) */
} LogRateLimit_t;

/**
 * @brief 热路径日志限速: 返回 1 表示这次允许打印(调用方随后自己 LOGxxx)
 */
uint8_t LogRateLimitAllow(LogRateLimit_t *rl, uint32_t period_ms);

/**
 * @brief 通过segger RTT打印日志,支持格式化输出,格式化输出的实现参考printf.
 * @attention !! 与 LOGINFO 等同: 先由 newlib 格式化到栈缓冲再写 RTT,
 *            参数消费按标准 C 规则, 不会错位; %f 已可用(见 CMake 的 -u _printf_float) !!
 *
 * @param fmt 格式字符串
 * @param ... 参数列表
 * @return int 打印的log字符数
 */
int PrintLog(const char *fmt, ...);

/**
 * @brief 利用sprintf(),将float转换为字符串进行打印
 * @attention 浮点数需要转换为字符串后才能通过RTT打印
 *
 * @param str 转换后的字符串
 * @param va 待转换的float
 */
void Float2Str(char *str, float va);

/* ---------------- 日志格式串安全自测 (2026-09-26, 默认关闭) ----------------
   置 1 后, daemon 任务会**每 2 秒**打一组混合格式日志(而不是只在启动时打一次:
   RTT 只有 1KB 环形缓冲, 启动那几秒的日志会把自测内容顶掉, 临时接上 RTT 抓不到)。
   其中**故意**留一条 %f 后面紧跟 %s 的写法 —— 用来验证"参数一律按标准 C 规则消费,
   不会错位"。改造前这样的写法会直接 BusFault -> HardFault; 现在是安全的自测。*/
#ifndef LOG_TEST
#define LOG_TEST 0
#endif

#if LOG_TEST
void LogSelfTest(void);
#endif

#endif
