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
    if (rl == NULL)
        return 1u;

    now = HAL_GetTick();
    rl->total++;
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

void Float2Str(char *str, float va)
{
    int flag = va < 0;
    int head = (int)va;
    int point = (int)((va - head) * 1000);
    head = abs(head);
    point = abs(point);
    if (flag)
        sprintf(str, "-%d.%d", head, point);
    else
        sprintf(str, "%d.%d", head, point);
}

#if LOG_TEST
/* 日志格式串安全自测(默认关闭, 见 bsp_log.h 里的 LOG_TEST)
   一次性覆盖: 字符串 / 整数 / 长整型 / 十六进制 / 浮点 / 多参数混排 / 超长行截断。
   重点是第 3 条 —— "%f" 后面紧跟着一个 "%s":
     改造前, 日志宏拼出的格式串在用户 format 之后还挂着一个 %s(复位色码),
     而 SEGGER 的精简 printf 不消费 %f 的参数, 于是那个 %s 会取到 float 的位模式
     当指针解引用 -> BusFault -> HardFault(LOG-01 的真凶);
     改造后, 参数由 newlib 按标准 C 规则消费, 这一行必须完整打印、后面的 %s 正确。*/
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
    /* 下面是**故意**超过缓冲区的长行: 用来验证 snprintf 的截断行为(不越界、不崩)。
       注意打开 LOG_TEST 后编译会有一条 `-Wformat-truncation` 告警指向这一行 ——
       这是预期结果, 不是新 bug。 */
    LOGWARNING("[logtest] 5/5 超长行截断验证: "
               "0123456789abcdef0123456789abcdef0123456789abcdef0123456789abcdef"
               "0123456789abcdef0123456789abcdef0123456789abcdef0123456789abcdef"
               "0123456789abcdef0123456789abcdef0123456789abcdef0123456789abcdef"
               "0123456789abcdef0123456789abcdef0123456789abcdef0123456789abcdef");
    LOGINFO("[logtest] done: 5 条都在、且第 3 条的 tail_s=OK 即说明格式串免疫生效");
}
#endif

