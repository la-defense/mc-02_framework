/** @file stm32h7xx_hal.h
 * @brief LK 主机测试使用的毫秒时钟声明。
 * @note 测试程序提供 HAL_GetTick()，不模拟真实外设。
 * @author MC-02 project contributors
 * @copyright Copyright (c) HNU YueLu EC 2022 all rights reserved
 */
#ifndef MC02_LK_TEST_HAL_H
#define MC02_LK_TEST_HAL_H
#include <stdint.h>
uint32_t HAL_GetTick(void);
#endif
