/** @file FreeRTOS.h
 * @brief 为 TIM5 测试提供有限等待所需的 FreeRTOS 类型。
 */
#ifndef MC02_TEST_PERIODIC_TIM5_FREERTOS_H
#define MC02_TEST_PERIODIC_TIM5_FREERTOS_H
#include <stdint.h>
typedef uint32_t TickType_t;
typedef void *TaskHandle_t;
typedef int BaseType_t;
typedef struct { uint8_t value; } StaticSemaphore_t; // 只保存是否已收到通知。
typedef StaticSemaphore_t *SemaphoreHandle_t;
#define configTICK_RATE_HZ 1000u // 与当前固件一致。
#define configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY 5u // 检查平台选择的 ISR 优先级。
#define portMAX_DELAY UINT32_MAX // 用于区分有限与无限等待。
#define pdFALSE 0
#define pdTRUE 1
void taskENTER_CRITICAL(void);
void taskEXIT_CRITICAL(void);
void portYIELD_FROM_ISR(BaseType_t reschedule);
#endif
