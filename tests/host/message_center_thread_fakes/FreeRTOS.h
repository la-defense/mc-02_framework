/** @file FreeRTOS.h
 * @brief 用互斥锁模拟FreeRTOS临界区，测试消息复制的一致性。
 */
#ifndef MC02_MESSAGE_CENTER_THREAD_FREERTOS_H
#define MC02_MESSAGE_CENTER_THREAD_FREERTOS_H

void HostCriticalEnter(void);
void HostCriticalExit(void);
#define taskENTER_CRITICAL() HostCriticalEnter()
#define taskEXIT_CRITICAL() HostCriticalExit()

#endif
