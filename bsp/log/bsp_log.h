/** @file bsp_log.h
* @brief 先用标准格式化函数准备短文本，再通过 RTT 输出；支持同类日志限速。
 */
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
// 先由 snprintf 处理全部参数，再写出纯文本；结尾换行和颜色码另行追加，避免混用格式化规则。
// 每次调用在任务栈上使用 256 字节，超长内容只保留数组放得下的部分。
// 日志数组还须容纳诊断汇总行，保留 256 字节，并为末尾换行及颜色复原码留空。
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
                _log_n = LOG_BUF_SIZE - LOG_BODY_TAIL - 1u; /* 只保留数组放得下的内容。 */  \
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

// 同类事件共用一个清零的计数对象，HAL 毫秒时间达到 period 后才允许报告。
// 初始 last_ms 为 0；上电未满 period 时首次也不报告，计时不前进时可能一直不报告。
typedef struct
{
    uint32_t last_ms; /* 上次真正打印的时间 */
    uint32_t total;   /* 累计事件数(每次调用 +1) */
    uint32_t printed; /* 上次打印时的累计数 */
    uint32_t dropped; /* 允许报告时保存的本段事件数，包含本次，不等于实际省略的条数。 */
} LogRateLimit_t;

/** @brief 计数一次事件，按时间决定本次是否允许调用方打印。
* @param[in,out] rl 首次使用前清零；NULL 时始终返回 1，不计数。
* @param[in] period_ms 最短报告间隔，单位 ms；为 0 时每次允许。
* @return 达到间隔返回 1 并更新时间，否则为 0；不负责实际输出。
 */
uint8_t LogRateLimitAllow(LogRateLimit_t *rl, uint32_t period_ms);

/** @brief 在本地数组中格式化并尝试写 RTT。
* @param[in] fmt 有效 printf 格式串，不能为 NULL。
* @param[in] ... 与格式串对应的参数。
* @return 格式化所需字节数，不含结尾 0；负值表示格式化失败且不输出。
* @note 文本超过 255 字节时只输出前面放得下的部分，返回值仍为完整所需长度，不代表实际写出数量。
 */
int PrintLog(const char *fmt, ...);

/** @brief 旧浮点转文字入口，保留原有小数写法，不是日志输出的必要步骤。
* @param[out] str 足够大的可写数组，不检查 NULL 或容量；本函数不提供长度参数。
* @param[in] va 待转换值，须能以 int 保存整数部分和小数乘 1000 的部分；不接受 NaN 或无穷大。
* @note 不补足小数前面的 0，不做四舍五入；日志宏已由 newlib 支持浮点格式化。
 */
void Float2Str(char *str, float va);

// 默认关闭；开启后由 Daemon 周期打印多种参数和超长行，检查格式化结果及长度处理。
#ifndef LOG_TEST
#define LOG_TEST 0
#endif

#if LOG_TEST
void LogSelfTest(void);
#endif

#endif
