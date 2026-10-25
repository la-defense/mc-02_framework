/** @file cmsis_os.h
 * @brief 为 Daemon 生产主机测试提供 CMSIS 任务创建替身。
 * @note 测试可记录任务配置并注入创建失败；不参与固件构建。
 * @author MC-02 project contributors
 * @copyright Copyright (c) HNU YueLu EC 2022 all rights reserved
 */
#ifndef MC02_DAEMON_TEST_CMSIS_OS_H
#define MC02_DAEMON_TEST_CMSIS_OS_H
#include <stdint.h>
typedef void (*os_pthread)(void const *argument);
typedef void *osThreadId;
typedef struct {os_pthread entry; uint32_t priority; uint32_t stack_size;} osThreadDef_t;
#define osPriorityBelowNormal 1u
#define osThreadDef(name, thread, priority, instances, stacksz) \
    const osThreadDef_t os_thread_def_##name = {thread, priority, stacksz}
#define osThread(name) (&os_thread_def_##name)
osThreadId osThreadCreate(const osThreadDef_t *definition, void *argument);
void osDelay(uint32_t milliseconds);
#endif
