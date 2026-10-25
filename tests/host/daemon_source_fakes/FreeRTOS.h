/** @file FreeRTOS.h
 * @brief 为 Daemon 来源测试映射可并发临界区。
 * @note 实际互斥由 host_critical_model.c 提供；不参与固件构建。
 * @author MC-02 project contributors
 * @copyright Copyright (c) HNU YueLu EC 2022 all rights reserved
 */
#ifndef MC02_DAEMON_TEST_FREERTOS_H
#define MC02_DAEMON_TEST_FREERTOS_H
void HostTaskCriticalEnter(void);
void HostTaskCriticalExit(void);
#define taskENTER_CRITICAL() HostTaskCriticalEnter()
#define taskEXIT_CRITICAL() HostTaskCriticalExit()
#endif
