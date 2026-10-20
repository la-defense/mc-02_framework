/**
 * @file cmsis_os.h
 * @brief 捕获 DM 任务入口，并为主机测试提供短临界区。
 */
#ifndef MC02_DM_MOTOR_TEST_CMSIS_OS_H
#define MC02_DM_MOTOR_TEST_CMSIS_OS_H

#include <stdint.h>
typedef void *osThreadId;
typedef void (*os_pthread)(void const *argument);
typedef enum { osOK = 0, osErrorOS = -1 } osStatus;
typedef enum { osPriorityNormal = 0 } osPriority;
typedef struct
{
    const char *name;
    os_pthread entry;
    osPriority priority;
    uint32_t instances;
    uint32_t stack_size;
} osThreadDef_t;

#define osThreadDef(name, fn, priority, count, stack) \
    const osThreadDef_t os_def_##name = {#name, (fn), (priority), (count), (stack)}
#define osThread(name) (&os_def_##name)

osThreadId osThreadCreate(const osThreadDef_t *definition, void *argument);
osStatus osThreadTerminate(osThreadId thread_id);
void osDelay(uint32_t milliseconds);
void dm_test_critical_enter(void);
void dm_test_critical_exit(void);
#define taskENTER_CRITICAL() dm_test_critical_enter()
#define taskEXIT_CRITICAL() dm_test_critical_exit()

#endif
