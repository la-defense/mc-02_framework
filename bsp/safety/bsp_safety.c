/**
 * @file bsp_safety.c
 * @brief 直接设置 GPIO 和 TIM3 寄存器，供启动及故障入口关闭已知输出。
 * 两个入口共用关闭步骤，对普通中断状态的处理不同，用法见 bsp_safety.h。
 */
#include "bsp_safety.h"

#include "main.h"

#if defined(MC02_SAFETY_TRACE_TEST)
// 主机测试先记录关键写入的顺序，再写模拟寄存器；do...while(0) 将这些操作组成一条语句。
extern void BSP_SafetyTestRecordWrite(const volatile uint32_t *reg, uint32_t value);
#define BSP_SAFETY_REG_WRITE(reg, value)              \
    do                                                \
    {                                                 \
        const uint32_t write_value_ = (value);        \
        BSP_SafetyTestRecordWrite(&(reg), write_value_); \
        (reg) = write_value_;                         \
    } while (0)
#else
// 常规编译直接赋值，不调用测试记录函数。
#define BSP_SAFETY_REG_WRITE(reg, value) ((reg) = (value))
#endif

void BSP_SafetyLatchOutputsOff(void)
{
    const uint32_t heater_pin = 1u << 1u; /* PB1 / TIM3_CH4 */
    // PC13、PC14、PC15 分别控制两路 24V 和一路 5V 电源使能。
    const uint32_t controlled_power_pins = POWER_24V_2_Pin |
                                           POWER_24V_1_Pin |
                                           POWER_5V_Pin;

    // 修改寄存器期间关闭普通中断，返回前也不重新开启。
    __disable_irq();
    // 先开启 GPIOB、GPIOC 和 TIM3 的时钟，并读回时钟寄存器，再访问这些外设。
    RCC->AHB4ENR |= RCC_AHB4ENR_GPIOBEN | RCC_AHB4ENR_GPIOCEN;
    (void)RCC->AHB4ENR;
    RCC->APB1LENR |= RCC_APB1LENR_TIM3EN;
    (void)RCC->APB1LENR;

    // 先把 PB1 的 GPIO 输出值设为 0，再改为普通输出，避免切换时沿用旧的高电平。
    BSP_SAFETY_REG_WRITE(GPIOB->BSRR, heater_pin << 16u);
    GPIOB->OTYPER &= ~heater_pin;
    GPIOB->OSPEEDR &= ~(3u << (1u * 2u));
    GPIOB->PUPDR &= ~(3u << (1u * 2u));
    BSP_SAFETY_REG_WRITE(GPIOB->MODER,
                         (GPIOB->MODER & ~(3u << (1u * 2u))) |
                             (1u << (1u * 2u)));

    // 三路电源使能引脚也先设置低输出值，再切换为普通输出。
    BSP_SAFETY_REG_WRITE(GPIOC->BSRR, controlled_power_pins << 16u);
    GPIOC->OTYPER &= ~controlled_power_pins;
    GPIOC->OSPEEDR &= ~((3u << (13u * 2u)) |
                        (3u << (14u * 2u)) |
                        (3u << (15u * 2u)));
    GPIOC->PUPDR &= ~((3u << (13u * 2u)) |
                      (3u << (14u * 2u)) |
                      (3u << (15u * 2u)));
    BSP_SAFETY_REG_WRITE(GPIOC->MODER,
                         (GPIOC->MODER & ~((3u << (13u * 2u)) |
                                          (3u << (14u * 2u)) |
                                          (3u << (15u * 2u)))) |
                             (1u << (13u * 2u)) |
                             (1u << (14u * 2u)) |
                             (1u << (15u * 2u)));

    // PB1 已由 GPIO 输出低电平，再清零比较值、关闭通道 4 并停止 TIM3，停下原来的 PWM 来源。
    TIM3->CCR4 = 0u;
    TIM3->CCER &= ~TIM_CCER_CC4E;
    TIM3->CR1 &= ~TIM_CR1_CEN;

    // 等待前面的寄存器访问完成后再返回。
    __DSB();
}

void BSP_SafetyStartupLatchOutputsOff(void)
{
    // 启动流程还要继续运行，先保存原中断状态，关断后再恢复。
    const uint32_t previous_primask = __get_PRIMASK();

    BSP_SafetyLatchOutputsOff();
    // 恢复原值，调用前已关中断时继续保持关闭。
    __set_PRIMASK(previous_primask);
}
