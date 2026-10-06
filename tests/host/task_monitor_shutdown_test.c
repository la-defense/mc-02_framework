#include <stdint.h>
#include <stdio.h>

#include "main.h"
#include "task_monitor.h"

#define CHECK(condition)                                                         \
    do                                                                           \
    {                                                                            \
        if (!(condition))                                                        \
        {                                                                        \
            fprintf(stderr, "FAIL:%d: %s\n", __LINE__, #condition);             \
            return 1;                                                            \
        }                                                                        \
    } while (0)

GPIO_TypeDef mc02_test_gpiob;
GPIO_TypeDef mc02_test_gpioc;
TIM_TypeDef mc02_test_tim3;
RCC_TypeDef mc02_test_rcc;
uint32_t mc02_test_irq_disabled;
uint32_t mc02_test_primask;
uint32_t mc02_test_tick;

static uint32_t watchdog_feed_count;
static uint32_t logged_after_shutdown;
static uint32_t log_count;
static uint8_t reported_task_health = 1u;

void BSP_WatchdogFeed(void)
{
    watchdog_feed_count++;
}

void RobotSafetySetTaskHealthy(uint8_t healthy)
{
    reported_task_health = healthy;
}

void MC02TestLogError(void)
{
    const uint32_t power_pins = POWER_24V_1_Pin | POWER_24V_2_Pin | POWER_5V_Pin;
    uint8_t heater_is_low_gpio =
        (mc02_test_gpiob.BSRR == (1u << 17u)) &&
        ((mc02_test_gpiob.MODER & (3u << 2u)) == (1u << 2u));
    uint8_t rails_are_low_gpio =
        (mc02_test_gpioc.BSRR == (power_pins << 16u)) &&
        ((mc02_test_gpioc.MODER & (3u << 26u)) == (1u << 26u)) &&
        ((mc02_test_gpioc.MODER & (3u << 28u)) == (1u << 28u)) &&
        ((mc02_test_gpioc.MODER & (3u << 30u)) == (1u << 30u));

    if (mc02_test_irq_disabled && heater_is_low_gpio && rails_are_low_gpio &&
        mc02_test_tim3.CCR4 == 0u &&
        (mc02_test_tim3.CCER & TIM_CCER_CC4E) == 0u &&
        (mc02_test_tim3.CR1 & TIM_CR1_CEN) == 0u)
        logged_after_shutdown++;
    log_count++;
}

int main(void)
{
    mc02_test_gpiob = (GPIO_TypeDef){0};
    mc02_test_gpioc = (GPIO_TypeDef){0};
    mc02_test_tim3 = (TIM_TypeDef){0};
    mc02_test_rcc = (RCC_TypeDef){0};
    mc02_test_irq_disabled = 0u;
    mc02_test_tick = 100u;

    mc02_test_gpiob.ODR = 1u << 1u;
    mc02_test_gpioc.ODR = POWER_24V_1_Pin | POWER_24V_2_Pin | POWER_5V_Pin;
    mc02_test_tim3.CCR4 = 25u;
    mc02_test_tim3.CCER = TIM_CCER_CC4E;
    mc02_test_tim3.CR1 = TIM_CR1_CEN;

    TaskMonitorInit();
    mc02_test_tick += 6u;
    TaskMonitorTick();

    CHECK(log_count >= 1u);
    CHECK(logged_after_shutdown == log_count);
    CHECK(reported_task_health == 0u);
    CHECK(watchdog_feed_count == 0u);
    CHECK(TaskMonitorAllAlive() == 0u);
    CHECK(TaskMonitorBeginLongOperation(TASK_MONITOR_INS, TASK_MONITOR_LONG_OPERATION_CALIBRATION) == 0u);
    CHECK(TaskMonitorBeginLongOperation(TASK_MONITOR_INS, TASK_MONITOR_LONG_OPERATION_FLASH) == 0u);

    /* Calibration may exceed INS's normal deadline while other tasks stay
       monitored, and an operation overrun must permanently stop watchdog feeds. */
    mc02_test_gpiob = (GPIO_TypeDef){0};
    mc02_test_gpioc = (GPIO_TypeDef){0};
    mc02_test_tim3 = (TIM_TypeDef){0};
    mc02_test_rcc = (RCC_TypeDef){0};
    mc02_test_tick = 200u;
    watchdog_feed_count = 0u;
    logged_after_shutdown = 0u;
    log_count = 0u;
    reported_task_health = 1u;
    mc02_test_irq_disabled = 0u;
    mc02_test_primask = 0u;
    TaskMonitorInit();
    CHECK(TaskMonitorBeginLongOperation(TASK_MONITOR_INS, TASK_MONITOR_LONG_OPERATION_CALIBRATION) == 1u);
    mc02_test_tick += 6u;
    TaskMonitorFeed(TASK_MONITOR_MOTOR);
    TaskMonitorFeed(TASK_MONITOR_ROBOT);
    TaskMonitorFeed(TASK_MONITOR_DAEMON);
    TaskMonitorTick();

    CHECK(reported_task_health == 1u);
    CHECK(watchdog_feed_count == 1u);
    CHECK(log_count == 0u);
    CHECK(TaskMonitorAllAlive() == 1u);

    mc02_test_tick += 6u;
    TaskMonitorFeed(TASK_MONITOR_ROBOT);
    TaskMonitorFeed(TASK_MONITOR_DAEMON);
    TaskMonitorTick();

    CHECK(reported_task_health == 0u);
    CHECK(watchdog_feed_count == 1u);
    CHECK(log_count >= 1u);
    CHECK(TaskMonitorAllAlive() == 0u);
    CHECK(TaskMonitorEndLongOperation(TASK_MONITOR_INS, TASK_MONITOR_LONG_OPERATION_CALIBRATION) == 0u);

    /* A calibration deadline remains wrap-safe and latched after a late return. */
    mc02_test_gpiob = (GPIO_TypeDef){0};
    mc02_test_gpioc = (GPIO_TypeDef){0};
    mc02_test_tim3 = (TIM_TypeDef){0};
    mc02_test_rcc = (RCC_TypeDef){0};
    mc02_test_tick = UINT32_MAX - 100u;
    watchdog_feed_count = 0u;
    logged_after_shutdown = 0u;
    log_count = 0u;
    reported_task_health = 1u;
    mc02_test_irq_disabled = 0u;
    mc02_test_primask = 0u;
    TaskMonitorInit();
    CHECK(TaskMonitorBeginLongOperation(TASK_MONITOR_INS, TASK_MONITOR_LONG_OPERATION_CALIBRATION) == 1u);
    mc02_test_tick += 15000u;
    TaskMonitorFeed(TASK_MONITOR_MOTOR);
    TaskMonitorFeed(TASK_MONITOR_ROBOT);
    TaskMonitorFeed(TASK_MONITOR_DAEMON);
    TaskMonitorTick();

    CHECK(reported_task_health == 0u);
    CHECK(watchdog_feed_count == 0u);
    CHECK(log_count >= 1u);
    CHECK(TaskMonitorEndLongOperation(TASK_MONITOR_INS, TASK_MONITOR_LONG_OPERATION_CALIBRATION) == 0u);
    TaskMonitorFeed(TASK_MONITOR_INS);
    TaskMonitorFeed(TASK_MONITOR_MOTOR);
    TaskMonitorFeed(TASK_MONITOR_ROBOT);
    TaskMonitorFeed(TASK_MONITOR_DAEMON);
    TaskMonitorTick();
    CHECK(watchdog_feed_count == 0u);
    CHECK(TaskMonitorAllAlive() == 0u);

    /* Startup grace is only for calibration before the first monitor tick;
       Flash must never use it to bypass the all-task health check. */
    mc02_test_tick = 450u;
    watchdog_feed_count = 0u;
    reported_task_health = 1u;
    mc02_test_irq_disabled = 0u;
    mc02_test_primask = 0u;
    TaskMonitorInit();
    mc02_test_tick += 40u;
    CHECK(TaskMonitorBeginLongOperation(TASK_MONITOR_INS, TASK_MONITOR_LONG_OPERATION_FLASH) == 0u);
    CHECK(reported_task_health == 0u);

    /* Calibration may begin during startup before other tasks have first fed;
       their first monitor sample still requires every task to have fed. */
    mc02_test_tick = 500u;
    watchdog_feed_count = 0u;
    reported_task_health = 1u;
    mc02_test_irq_disabled = 0u;
    mc02_test_primask = 0u;
    TaskMonitorInit();
    TaskMonitorFeed(TASK_MONITOR_INS);
    mc02_test_tick += 100u;
    CHECK(TaskMonitorBeginLongOperation(TASK_MONITOR_INS, TASK_MONITOR_LONG_OPERATION_CALIBRATION) == 1u);
    TaskMonitorFeed(TASK_MONITOR_MOTOR);
    TaskMonitorFeed(TASK_MONITOR_ROBOT);
    TaskMonitorFeed(TASK_MONITOR_DAEMON);
    TaskMonitorTick();
    CHECK(reported_task_health == 1u);
    CHECK(watchdog_feed_count == 1u);
    CHECK(TaskMonitorEndLongOperation(TASK_MONITOR_INS, TASK_MONITOR_LONG_OPERATION_CALIBRATION) == 1u);

    /* Flash stalls the whole H723 core, so a bounded window covers all task
       deadlines only after the begin call has verified them healthy. */
    mc02_test_tick = 1000u;
    watchdog_feed_count = 0u;
    reported_task_health = 1u;
    mc02_test_irq_disabled = 0u;
    mc02_test_primask = 0u;
    TaskMonitorInit();
    TaskMonitorFeed(TASK_MONITOR_INS);
    TaskMonitorFeed(TASK_MONITOR_MOTOR);
    TaskMonitorFeed(TASK_MONITOR_ROBOT);
    TaskMonitorFeed(TASK_MONITOR_DAEMON);
    CHECK(TaskMonitorBeginLongOperation(TASK_MONITOR_INS, TASK_MONITOR_LONG_OPERATION_FLASH) == 1u);
    mc02_test_tick += 40u;
    TaskMonitorTick();
    CHECK(reported_task_health == 1u);
    CHECK(watchdog_feed_count == 1u);
    CHECK(TaskMonitorEndLongOperation(TASK_MONITOR_INS, TASK_MONITOR_LONG_OPERATION_FLASH) == 1u);
    CHECK(mc02_test_primask == 0u);
    CHECK(TaskMonitorAllAlive() == 1u);

    /* The shorter Flash window cannot reset the enclosing calibration budget. */
    mc02_test_tick = 2000u;
    watchdog_feed_count = 0u;
    reported_task_health = 1u;
    mc02_test_irq_disabled = 0u;
    mc02_test_primask = 0u;
    TaskMonitorInit();
    TaskMonitorFeed(TASK_MONITOR_INS);
    TaskMonitorFeed(TASK_MONITOR_MOTOR);
    TaskMonitorFeed(TASK_MONITOR_ROBOT);
    TaskMonitorFeed(TASK_MONITOR_DAEMON);
    CHECK(TaskMonitorBeginLongOperation(TASK_MONITOR_INS, TASK_MONITOR_LONG_OPERATION_CALIBRATION) == 1u);
    mc02_test_tick += 14900u;
    TaskMonitorFeed(TASK_MONITOR_MOTOR);
    TaskMonitorFeed(TASK_MONITOR_ROBOT);
    TaskMonitorFeed(TASK_MONITOR_DAEMON);
    CHECK(TaskMonitorBeginLongOperation(TASK_MONITOR_INS, TASK_MONITOR_LONG_OPERATION_FLASH) == 1u);
    mc02_test_tick += 200u;
    CHECK(TaskMonitorEndLongOperation(TASK_MONITOR_INS, TASK_MONITOR_LONG_OPERATION_FLASH) == 0u);
    CHECK(TaskMonitorAllAlive() == 0u);

    /* An overlong Flash stall latches the task monitor and stops watchdog feeds. */
    mc02_test_tick = 3000u;
    watchdog_feed_count = 0u;
    reported_task_health = 1u;
    mc02_test_irq_disabled = 0u;
    mc02_test_primask = 0u;
    TaskMonitorInit();
    TaskMonitorFeed(TASK_MONITOR_INS);
    TaskMonitorFeed(TASK_MONITOR_MOTOR);
    TaskMonitorFeed(TASK_MONITOR_ROBOT);
    TaskMonitorFeed(TASK_MONITOR_DAEMON);
    CHECK(TaskMonitorBeginLongOperation(TASK_MONITOR_INS, TASK_MONITOR_LONG_OPERATION_FLASH) == 1u);
    mc02_test_tick += 8000u;
    TaskMonitorTick();
    CHECK(reported_task_health == 0u);
    CHECK(watchdog_feed_count == 0u);
    CHECK(TaskMonitorEndLongOperation(TASK_MONITOR_INS, TASK_MONITOR_LONG_OPERATION_FLASH) == 0u);
    CHECK(TaskMonitorAllAlive() == 0u);

    puts("PASS: task-monitor fault latches outputs off before logging and watchdog reset");
    return 0;
}
