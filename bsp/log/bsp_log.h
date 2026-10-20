/** @file bsp_log.h
* @brief 声明 BSPLogInit、PrintLog 和日志宏，支持同类日志限速，并引入 Float2Str 声明。
* @note RTT 日志先调用 BSPLogInit；独立的 Float2Str 不需要日志初始化或硬件。
 */
#ifndef _BSP_LOG_H
#define _BSP_LOG_H

#include "SEGGER_RTT.h"
#include "SEGGER_RTT_Conf.h"
#include <stdio.h>
#include <stdint.h>
#include <string.h> /* LOG_PROTO 里用 memcpy 拼接结尾的换行+复位色码 */
#include "float_to_string.h"
#include "format_check.h"

#define BUFFER_INDEX 0

/** @brief 初始化默认RTT通道；在启动阶段串行调用，先于普通日志输出。
 * @note 无返回状态；调用完成不证明探针或查看器已经连接。
 */
void BSPLogInit();

// 先由 snprintf 处理全部参数，再写出纯文本；结尾换行和颜色码另行追加，避免混用格式化规则。
// 每次调用在任务栈上使用 256 字节，超长内容只保留数组放得下的部分。
// 日志数组还须容纳诊断汇总行，保留 256 字节，并为末尾换行及颜色复原码留空。
#define LOG_BUF_SIZE 256u /* 整行日志缓冲(放在栈上) */
#define LOG_BODY_TAIL 8u  /* 给结尾的 "\r\n" + 复位色码预留的位置 */

/* 缓冲区尾部预留量必须放得下 "\r\n" + RTT_CTRL_RESET(+结束符) */
_Static_assert(sizeof("\r\n" RTT_CTRL_RESET) <= LOG_BODY_TAIL,
               "LOG_BODY_TAIL 放不下结尾的换行+复位色码");

/** @brief 在局部数组格式化一行并追加换行、颜色复原码，再尝试交给RTT。
 * @param[in] type 可读的零结尾类型文字，生命周期至少覆盖本次调用。
 * @param[in] color 可读的零结尾颜色码，生命周期至少覆盖本次调用。
 * @param[in] format printf格式字面量；后续实参类型和数量必须匹配。
 * @note 先BSPLogInit；每次使用256字节局部数组，无返回状态，超长时只交付能放下的部分。
 *       格式化失败仅写换行和颜色复原码。编译检查不能替代动态长度检查，不证明查看器已显示。
 */
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
 *             本轮没有测量板上格式化耗时，实时路径须按实际预算选择格式。
 *  @note DISABLE_LOG_SYSTEM仅关闭下面三个彩色宏；LOG和PrintLog仍可直接输出。
 *        关闭宏仍编译检查格式，但不求值参数、不执行发送。
 */
#if DISABLE_LOG_SYSTEM
// 保留可被编译器检查的调用；常假分支不求值参数，也不生成发送代码。
#define LOGINFO(format, ...) do { if (0) { LOG_PROTO("I:", RTT_CTRL_TEXT_BRIGHT_GREEN, format, ##__VA_ARGS__); } } while (0)
#define LOGWARNING(format, ...) do { if (0) { LOG_PROTO("W:", RTT_CTRL_TEXT_BRIGHT_YELLOW, format, ##__VA_ARGS__); } } while (0)
#define LOGERROR(format, ...) do { if (0) { LOG_PROTO("E:", RTT_CTRL_TEXT_BRIGHT_RED, format, ##__VA_ARGS__); } } while (0)
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
* @note 先成功BSPLogInit，仅从任务调用；格式及所引用对象须在调用期间有效。
*       文本超过255字节时只输出前面放得下的部分，返回值仍为完整所需长度，不代表实际写出数量。
*       编译属性检查已知格式的参数类型，不能证明动态文字完整；不得把外部文字直接用作格式串。
 */
int PrintLog(const char *fmt, ...) MC02_PRINTF_FORMAT(1, 2);

// 默认关闭；开启后由Daemon打印合法混排自测，超长输入由主机测试验证。
#ifndef LOG_TEST
#define LOG_TEST 0
#endif

#if LOG_TEST
/** @brief 初始化RTT后从Daemon任务输出合法混排自测；无返回状态，不证明查看器显示。 */
void LogSelfTest(void);
#endif

#endif
