/**
 * @file bsp_crash_guard.c
 * @brief 实现崩溃记录的进入检查，接口用法见 bsp_crash_guard.h。
 * bsp_crash.c 首次进入后继续保存现场，重复进入时在这里请求复位。
 */
#include "bsp_crash_guard.h"

#include "main.h"

#include <stddef.h>

void CrashLogGuardEnter(volatile uint32_t *active)
{
    // 缺少标志或已经开始记录时，立即请求复位，避免重复记录覆盖已有内容。
    if (active == NULL || *active != 0u)
    {
        NVIC_SystemReset();
        // 正常的复位函数不会返回；若意外返回，就停在这里，不再继续执行。
        for (;;)
            ;
    }

    // 在调用方开始写现场前记下已进入，供后续异常识别。
    *active = 1u;
}
