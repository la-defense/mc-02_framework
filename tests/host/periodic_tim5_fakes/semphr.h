/** @file semphr.h
 * @brief 模拟单个静态信号量及中断通知。
 */
#ifndef MC02_TEST_PERIODIC_TIM5_SEMPHR_H
#define MC02_TEST_PERIODIC_TIM5_SEMPHR_H
#include "FreeRTOS.h"
SemaphoreHandle_t xSemaphoreCreateBinaryStatic(StaticSemaphore_t *storage);
BaseType_t xSemaphoreTake(SemaphoreHandle_t signal, TickType_t ticks);
BaseType_t xSemaphoreGiveFromISR(SemaphoreHandle_t signal, BaseType_t *reschedule);
void vSemaphoreDelete(SemaphoreHandle_t signal);
#endif
