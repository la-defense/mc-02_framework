#include <setjmp.h>
#include <stdint.h>
#include <stdio.h>

#include "bsp_crash_guard.h"

#define CHECK(condition)                                                        \
    do                                                                          \
    {                                                                           \
        if (!(condition))                                                       \
        {                                                                       \
            fprintf(stderr, "FAIL:%d: %s\n", __LINE__, #condition);          \
            return 1;                                                           \
        }                                                                       \
    } while (0)

static jmp_buf reset_target;
static uint32_t reset_count;

void NVIC_SystemReset(void)
{
    reset_count++;
    longjmp(reset_target, 1);
}

int main(void)
{
    volatile uint32_t active = 0u;

    CrashLogGuardEnter(&active);
    CHECK(active == 1u);
    CHECK(reset_count == 0u);

    if (setjmp(reset_target) == 0)
    {
        CrashLogGuardEnter(&active);
        CHECK(0 && "a repeated crash entry must reset immediately");
    }

    CHECK(reset_count == 1u);
    CHECK(active == 1u);
    puts("PASS: repeated crash entry resets without re-claiming the guard");
    return 0;
}
