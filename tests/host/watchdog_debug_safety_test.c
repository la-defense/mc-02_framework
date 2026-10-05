#include <stdint.h>
#include <stdio.h>

#include "bsp_watchdog.h"
#include "main.h"

#define CHECK(condition)                                                        \
    do                                                                          \
    {                                                                           \
        if (!(condition))                                                       \
        {                                                                       \
            fprintf(stderr, "FAIL:%d: %s\n", __LINE__, #condition);           \
            return 1;                                                           \
        }                                                                       \
    } while (0)

DWT_TypeDef mc02_test_dwt;
IWDG_TypeDef mc02_test_iwdg;
DBGMCU_TypeDef mc02_test_dbgmcu;
RCC_TypeDef mc02_test_rcc;
uint32_t mc02_test_tick;
uint32_t mc02_test_primask;
volatile uint32_t dwt_cpu_freq_mhz = 1u;

static uint32_t dwt_cycle_step;
static uint32_t error_log_count;
static uint32_t info_log_count;

DWT_TypeDef *MC02TestDWT(void)
{
    mc02_test_dwt.CYCCNT += dwt_cycle_step;
    return &mc02_test_dwt;
}

void MC02TestLogError(void)
{
    error_log_count++;
}

void MC02TestLogInfo(void)
{
    info_log_count++;
}

int main(void)
{
    mc02_test_tick = 0u;
    mc02_test_primask = 1u;
    dwt_cycle_step = 0u;

    /* A running debugger must not freeze the watchdog counter. */
    BSP_WatchdogInit(200u);
    CHECK((mc02_test_dbgmcu.APB4FZ1 & DBGMCU_APB4FZ1_DBG_IWDG1) == 0u);
    CHECK(info_log_count == 1u);

    /* A stopped HAL tick (as with interrupts masked) must not make flag polling infinite. */
    mc02_test_iwdg.SR = IWDG_SR_PVU | IWDG_SR_RVU | IWDG_SR_WVU;
    dwt_cycle_step = 10000u;
    BSP_WatchdogInit(200u);

    CHECK(mc02_test_tick == 0u);
    CHECK(mc02_test_primask == 1u);
    CHECK(error_log_count == 2u);
    CHECK(mc02_test_iwdg.KR == 0x0000AAAAu);
    CHECK(info_log_count == 2u);
    puts("PASS: watchdog remains active in debug and flag polling is bounded without HAL tick");
    return 0;
}
