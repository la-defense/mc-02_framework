/** @file cmsis_os.h
 * @brief 模拟日志端口使用的调度状态和优先级。
 */
#ifndef MC02_LOG_PORT_FAKE_CMSIS_H
#define MC02_LOG_PORT_FAKE_CMSIS_H
#include <stdint.h>
enum { osPriorityIdle = -3, osPriorityBelowNormal = -1 };
int32_t osKernelRunning(void);
void osDelay(uint32_t milliseconds);
#endif
