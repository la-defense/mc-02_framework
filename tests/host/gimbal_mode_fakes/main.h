/** @file main.h
 * @brief 为生产云台模块提供当前 CAN 句柄和毫秒时钟声明。
 * @note 时钟由测试固定，不访问 HAL 或板级寄存器。
 */
#ifndef MC02_GIMBAL_MODE_TEST_MAIN_H
#define MC02_GIMBAL_MODE_TEST_MAIN_H

#include <stdint.h>
#include "bsp_can.h"

uint32_t HAL_GetTick(void);

#endif
