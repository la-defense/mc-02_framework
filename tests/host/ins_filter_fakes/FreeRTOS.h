/** @file FreeRTOS.h
 * @brief INS 主机测试的临界区替身；当前测试单线程运行，不模拟调度或中断。
 */
#ifndef INS_FILTER_FAKE_FREERTOS_H
#define INS_FILTER_FAKE_FREERTOS_H
#define taskENTER_CRITICAL() ((void)0)
#define taskEXIT_CRITICAL() ((void)0)
#endif
