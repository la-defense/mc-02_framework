/** @file main.h
 * @brief 致命启动失败测试的中断、复位和看门狗寄存器替身。
 */
#ifndef MC02_FATAL_STARTUP_FAKE_MAIN_H
#define MC02_FATAL_STARTUP_FAKE_MAIN_H
#include <stdint.h>

typedef struct { uint32_t KR; } FatalTestWatchdog_t;
extern FatalTestWatchdog_t fatal_test_watchdog;
extern uint32_t fatal_test_irq;
#define IWDG1 (&fatal_test_watchdog)
#define __disable_irq() (fatal_test_irq = 1u)
void FatalTestClearResetFlags(void);
void FatalTestNop(void);
void NVIC_SystemReset(void);
#define __HAL_RCC_CLEAR_RESET_FLAGS() FatalTestClearResetFlags()
#define __NOP() FatalTestNop()
void Error_Handler(void);
#endif
