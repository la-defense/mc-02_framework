/** @file FreeRTOS.h
 * @brief 主机裁判快照并发测试用互斥临界区替身。
 * Linux线程测试用同一互斥锁包住生产发布和读取；Windows按单线程交错验证。
 * @author kidneygood (you@domain.com)
 * @copyright Copyright (c) HNU YueLu EC 2022 all rights reserved
 */
#ifndef MC02_REFEREE_TEST_FREERTOS_H
#define MC02_REFEREE_TEST_FREERTOS_H
#if defined(REFEREE_TEST_THREADS)
#include <pthread.h>
extern pthread_mutex_t referee_test_critical_mutex;
#define taskENTER_CRITICAL() ((void)pthread_mutex_lock(&referee_test_critical_mutex))
#define taskEXIT_CRITICAL() ((void)pthread_mutex_unlock(&referee_test_critical_mutex))
#else
#define taskENTER_CRITICAL() ((void)0)
#define taskEXIT_CRITICAL() ((void)0)
#endif
#endif
