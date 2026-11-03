/** @file cmsis_os.h @brief 任务信号适配测试使用的最小 CMSIS 替身。 */
#ifndef MC02_TEST_SIGNAL_CMSIS_OS_H
#define MC02_TEST_SIGNAL_CMSIS_OS_H
#include <stdint.h>
typedef void *osThreadId;
typedef enum { osOK = 0, osEventSignal = 0x08, osEventTimeout = 0x40, osErrorISR = 0x82 } osStatus;
typedef struct { osStatus status; union { int32_t signals; } value; } osEvent;
int32_t osSignalSet(osThreadId thread, int32_t signals);
osEvent osSignalWait(int32_t signals, uint32_t milliseconds);
#endif
