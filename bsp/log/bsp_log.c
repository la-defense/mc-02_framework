/** @file bsp_log.c
* @brief 初始化 RTT、决定同类日志能否打印，并由 PrintLog 格式化和提交文字。
*/
#include "bsp_log.h"
#include "main.h"

#include "SEGGER_RTT.h"
#include "SEGGER_RTT_Conf.h"
#include <stdio.h>


void BSPLogInit()
{
    SEGGER_RTT_Init();
}

uint8_t LogRateLimitAllow(LogRateLimit_t *rl, uint32_t period_ms)
{
    uint32_t now;
    // 未提供计数对象时让调用方直接打印，不保存间隔或次数。
    if (rl == NULL)
        return 1u;

    now = HAL_GetTick();
    rl->total++;
    // 等到报告间隔才更新基准和这一段的计数；未到时只增加 total。
    if ((uint32_t)(now - rl->last_ms) >= period_ms)
    {
        rl->last_ms = now;
        rl->dropped = rl->total - rl->printed; /* 距离上次打印期间又发生了多少条 */
        rl->printed = rl->total;
        return 1u;
    }
    return 0u;
}

int PrintLog(const char *fmt, ...)
{
    char buf[LOG_BUF_SIZE];
    va_list args;
    va_start(args, fmt);
    /* 与 LOG_PROTO 同一套机制: 先由 newlib 格式化到栈缓冲, 再整体写成字符串。
       这样参数消费完全按标准 C 规则走, 不会因为某个格式符"没人认识"而错位。*/
    int n = vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);
    if (n < 0)
        return n; /* 格式化失败: 宁可什么都不输出, 也不输出半截内容 */
    SEGGER_RTT_WriteString(BUFFER_INDEX, buf);
    return n;
}

#if LOG_TEST
// 默认关闭的格式化检查：混排整数、文字和浮点，再检查超长行只保留可容纳部分。
void LogSelfTest(void)
{
    const char *s = "OK";
    unsigned int u = 1234567u;
    unsigned long lu = 0xDEADBEEFu;
    int d = -42;

    LOGINFO("[logtest] 1/5 字符串/整数: s=%s u=%u d=%d", s, u, d);
    LOGINFO("[logtest] 2/5 长整型: lu=%lu hex=%08lX", lu, lu);
    LOGWARNING("[logtest] 3/5 核心项(%%f 后面紧跟 %%s): f=%f tail_s=%s", 3.14159, s);
    LOGINFO("[logtest] 4/5 多参数混排: %s|%u|%d|%s|%lu", s, u, d, "end", lu);
    // 故意提供超长行来检查长度处理；启用后编译器会对这行报告格式化长度告警。
    LOGWARNING("[logtest] 5/5 超长行截断验证: "
               "0123456789abcdef0123456789abcdef0123456789abcdef0123456789abcdef"
               "0123456789abcdef0123456789abcdef0123456789abcdef0123456789abcdef"
               "0123456789abcdef0123456789abcdef0123456789abcdef0123456789abcdef"
               "0123456789abcdef0123456789abcdef0123456789abcdef0123456789abcdef");
    LOGINFO("[logtest] done: 5 条都在、且第 3 条的 tail_s=OK 即说明格式串免疫生效");
}
#endif

