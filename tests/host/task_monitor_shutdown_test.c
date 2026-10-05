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
    puts("PASS: task-monitor fault latches outputs off before logging and watchdog reset");
    return 0;
}
