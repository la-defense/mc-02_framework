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
    va_list args;
    va_start(args, fmt);
    int n = SEGGER_RTT_vprintf(BUFFER_INDEX, fmt, &args); // 一次可以开启多个buffer(多个终端),我们只用一个
    va_end(args);
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

