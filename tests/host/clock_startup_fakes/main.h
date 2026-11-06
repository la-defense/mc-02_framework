/** @file main.h
 * @brief 用于主机测试的 H723 周期计数、电压与中断替身。
 */
#ifndef MC02_CLOCK_STARTUP_FAKE_MAIN_H
#define MC02_CLOCK_STARTUP_FAKE_MAIN_H
#include <stdint.h>

typedef struct { uint32_t DEMCR; } ClockTestDebug_t;
typedef struct { uint32_t CTRL; uint32_t CYCCNT; } ClockTestDwt_t;
extern ClockTestDebug_t clock_test_debug;
extern ClockTestDwt_t clock_test_dwt;
extern volatile uint32_t uwTick;
extern uint32_t clock_test_irq;
extern uint8_t clock_test_vos;
uint32_t ClockTestReadTick(void);
#define CLOCK_STARTUP_READ_RAW_TICK() ClockTestReadTick()
#define CoreDebug (&clock_test_debug)
#define DWT (&clock_test_dwt)
#define CoreDebug_DEMCR_TRCENA_Msk 1u
#define DWT_CTRL_CYCCNTENA_Msk 1u
#define PWR_FLAG_VOSRDY 1u
#define __HAL_PWR_GET_FLAG(flag) ((void)(flag), clock_test_vos)
#define __DMB() ((void)0)
#define __get_PRIMASK() (clock_test_irq)
#define __disable_irq() (clock_test_irq = 1u)
#define __set_PRIMASK(value) (clock_test_irq = (value))
#define CLOCKSWITCH_TIMEOUT_VALUE 5000u
#define HSE_TIMEOUT_VALUE 100u
#define HSI_TIMEOUT_VALUE 2u
#define HSI48_TIMEOUT_VALUE 2u
#define PLL_TIMEOUT_VALUE 2u
void NVIC_SystemReset(void);
void ClockTestNop(void);
void ClockTestClearResetFlags(void);
#define __HAL_RCC_CLEAR_RESET_FLAGS() ClockTestClearResetFlags()
#define __NOP() ClockTestNop()
#endif
