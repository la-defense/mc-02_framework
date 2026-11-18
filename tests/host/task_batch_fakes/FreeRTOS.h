/** @file FreeRTOS.h
 * @brief 为任务创建批次测试提供最小调度状态类型。
 */
#ifndef MC02_TEST_TASK_BATCH_FREERTOS_H
#define MC02_TEST_TASK_BATCH_FREERTOS_H
#include <stdint.h>
typedef int BaseType_t;
typedef uint32_t TickType_t; // 与固件相同的计数宽度。
#define configTICK_RATE_HZ 1000u // 模拟固件已有的每毫秒一刻度。
#endif
