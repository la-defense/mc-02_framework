/** @file FreeRTOS.h
 * @brief 日志端口创建任务测试使用的最小 FreeRTOS 类型。
 */
#ifndef MC02_LOG_PORT_FAKE_FREERTOS_H
#define MC02_LOG_PORT_FAKE_FREERTOS_H
#include <stdint.h>
typedef uint32_t StackType_t; // 测试仅检查栈指针，不模拟实际调度栈。
typedef struct { uint32_t value; } StaticTask_t; // 模拟静态任务控制块。
typedef void *TaskHandle_t; // 模拟不透明任务句柄。
#define tskIDLE_PRIORITY 0u
#define configMAX_PRIORITIES 8u
void LogTestEnterTaskCritical(void);
void LogTestExitTaskCritical(void);
#define taskENTER_CRITICAL() LogTestEnterTaskCritical()
#define taskEXIT_CRITICAL() LogTestExitTaskCritical()
#define taskYIELD() ((void)0)
#endif
