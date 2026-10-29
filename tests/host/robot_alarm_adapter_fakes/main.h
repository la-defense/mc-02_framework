/** @file main.h
 * @brief 告警适配层主机测试用时钟与临界区声明。
 * @note 只供主机测试替换 CMSIS/HAL；不参与固件构建。
 */
#ifndef MC02_ROBOT_ALARM_TEST_MAIN_H
#define MC02_ROBOT_ALARM_TEST_MAIN_H

#include <stdint.h>

uint32_t HAL_GetTick(void);
uint32_t __get_PRIMASK(void);
void __disable_irq(void);
void __set_PRIMASK(uint32_t previous);

#endif
