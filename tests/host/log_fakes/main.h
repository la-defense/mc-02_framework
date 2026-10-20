/** @file main.h
 * @brief 日志测试只提供毫秒时钟声明，不包含HAL。
 */
#ifndef MC02_LOG_FAKE_MAIN_H
#define MC02_LOG_FAKE_MAIN_H
#include <stdint.h>
uint32_t HAL_GetTick(void);
#endif
