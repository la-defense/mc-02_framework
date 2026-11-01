/** @file stm32h7xx_hal.h
 * @brief 主机 DJI 电机测试所需的毫秒时钟声明。
 * @note 不替代 HAL 外设实现；测试程序提供确定性的 HAL_GetTick()。
 * @author MC-02 project contributors
 * @copyright Copyright (c) HNU YueLu EC 2022 all rights reserved
 */
#ifndef MC02_MOTOR_PID_TEST_HAL_H
#define MC02_MOTOR_PID_TEST_HAL_H
#include <stdint.h>
typedef struct { unsigned unused; } UART_HandleTypeDef;
uint32_t HAL_GetTick(void);
#endif
