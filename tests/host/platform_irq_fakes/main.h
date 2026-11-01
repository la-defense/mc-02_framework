/** @file main.h
 * @brief 使用可控 PRIMASK 替身测试 STM32 中断屏蔽适配器。
 */
#ifndef MC02_PLATFORM_IRQ_TEST_MAIN_H
#define MC02_PLATFORM_IRQ_TEST_MAIN_H
#include <stdint.h>
uint32_t TestIrqGetMask(void);
void TestIrqDisable(void);
void TestIrqSetMask(uint32_t mask);
#define __get_PRIMASK() TestIrqGetMask()
#define __disable_irq() TestIrqDisable()
#define __set_PRIMASK(mask_) TestIrqSetMask(mask_)
#endif
