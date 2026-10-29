/** @file FreeRTOS.h
 * @brief 主机测试使用互斥锁模拟超级电容数据复制的短临界区。
 */
#ifndef SUPER_CAP_TEST_FREERTOS_H
#define SUPER_CAP_TEST_FREERTOS_H
#include <pthread.h>
extern pthread_mutex_t super_cap_test_mutex;
#define taskENTER_CRITICAL() ((void)pthread_mutex_lock(&super_cap_test_mutex))
#define taskEXIT_CRITICAL() ((void)pthread_mutex_unlock(&super_cap_test_mutex))
#endif
