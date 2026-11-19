/** @file chassis_task_config.h
 * @brief 车型提供的独立底盘任务时间和栈配置。
 */
#ifndef MC02_CHASSIS_TASK_CONFIG_H
#define MC02_CHASSIS_TASK_CONFIG_H
#include "../../bsp/os/platform_periodic.h"

typedef struct {
    PlatformPeriodicConfig_t timing; /**< 固定周期与单轮预算，单位 μs。 */
    PlatformPeriodicSource_t source; /**< 车型允许的时钟来源；启动通过后转为实际使用的单一来源。 */
    uint32_t deadline_ms; /**< 监控相邻工作或完成报告的期限，单位 ms。 */
    uint32_t startup_grace_ms; /**< 放行后首次工作的宽限，单位 ms。 */
    uint32_t stack_words; /**< RTOS 栈容量，单位为字。 */
} RobotChassisTaskConfig_t;
#endif
