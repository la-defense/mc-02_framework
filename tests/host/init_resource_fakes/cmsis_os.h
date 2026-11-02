/** @file cmsis_os.h
 * @brief 为初始化资源主机测试声明 CMSIS 任务接口替身。
 * @note 测试只用于记录或注入任务创建结果；不参与固件构建。
 * @author MC-02 project contributors
 * @copyright Copyright (c) HNU YueLu EC 2022 all rights reserved
 */
#ifndef MC02_INIT_RESOURCE_CMSIS_OS_H
#define MC02_INIT_RESOURCE_CMSIS_OS_H
#include <stdint.h>
typedef void (*os_pthread)(void const *argument);
typedef void *osThreadId;
typedef struct
{
    const char *name;
    os_pthread pthread;
    uint32_t priority;
    uint32_t instances;
    uint32_t stack_size;
} osThreadDef_t;
#define osPriorityBelowNormal 1u
#define osPriorityLow 0u
#define osThreadDef(name, thread, priority, instances, stacksz) \
    const osThreadDef_t os_thread_def_##name = {#name, thread, priority, instances, stacksz}
#define osThread(name) (&os_thread_def_##name)
osThreadId osThreadCreate(const osThreadDef_t *definition, void *argument);
void osDelay(uint32_t milliseconds);
#endif
