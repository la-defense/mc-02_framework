/** @file main.h
 * @brief 为板间桥接主机测试提供可控毫秒时钟与中断屏蔽替身。
 * @note 不模拟 MCU 中断，仅支持验证桥接撤销与 Safety 接口的确定性调用顺序。
 */
#ifndef MC02_CAN_BRIDGE_TEST_MAIN_H
#define MC02_CAN_BRIDGE_TEST_MAIN_H
#include <stdint.h>

uint32_t HAL_GetTick(void);
#define __get_PRIMASK() (0u)
#define __disable_irq() ((void)0)
#define __set_PRIMASK(value) ((void)(value))

#endif
