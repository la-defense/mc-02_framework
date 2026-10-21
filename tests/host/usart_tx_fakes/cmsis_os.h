/** @file cmsis_os.h
 * @brief UART任务测试记录真实创建定义；不会在主机创建RTOS任务。
 */
#ifndef MC02_UART_TEST_CMSIS_OS_H
#define MC02_UART_TEST_CMSIS_OS_H
#include <stdint.h>
typedef void *osThreadId;
typedef void (*os_pthread)(void const *);
typedef enum
{
    osPriorityNormal = 0
} osPriority;
typedef struct
{
    const char *name;
    os_pthread entry;
    osPriority priority;
    uint32_t instances, stack_size;
} osThreadDef_t;
#define osThreadDef(name, fn, priority, count, stack)                                                        \
    const osThreadDef_t os_def_##name = {#name, fn, priority, count, stack}
#define osThread(name) (&os_def_##name)
osThreadId osThreadCreate(const osThreadDef_t *definition, void *argument);
int32_t osKernelRunning(void);
void osDelay(uint32_t ms);
#endif
