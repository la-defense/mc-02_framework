#include "bsp_crash_guard.h"

#include "main.h"

#include <stddef.h>

void CrashLogGuardEnter(volatile uint32_t *active)
{
    if (active == NULL || *active != 0u)
    {
        NVIC_SystemReset();
        for (;;)
            ;
    }

    *active = 1u;
}
