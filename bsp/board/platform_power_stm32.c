/** @file platform_power_stm32.c
 * @brief MC-02 三路电源使能引脚；裸板固件不提供开启能力。
 */
#include "platform_power.h"
#include "main.h"

#if defined(MC02_PROFILE_ROBOT)
// 逻辑编号只绑定本板三条已声明引脚。
static uint8_t PlatformPowerPin(PlatformPowerRail_t rail, uint16_t *pin, unsigned *shift)
{
    switch (rail)
    {
    case PLATFORM_POWER_24V_1: *pin = POWER_24V_1_Pin; *shift = 14u; return 1u; // PC14。
    case PLATFORM_POWER_24V_2: *pin = POWER_24V_2_Pin; *shift = 13u; return 1u; // PC13。
    case PLATFORM_POWER_5V: *pin = POWER_5V_Pin; *shift = 15u; return 1u; // PC15。
    default: return 0u; // 未声明编号不能访问引脚。
    }
}
#endif

uint8_t PlatformPowerEnable(PlatformPowerRail_t rail)
{
#if defined(MC02_PROFILE_ROBOT)
    uint16_t pin;
    unsigned shift;
    if (!PlatformPowerPin(rail, &pin, &shift)) return 0u; // 拒绝未声明编号。
    if (((GPIOC->MODER >> (shift * 2u)) & 3u) != 1u) return 0u; // 只有已配置的普通输出可开启。
    HAL_GPIO_WritePin(GPIOC, pin, GPIO_PIN_SET); // 调用方已在短临界区内确认安全状态。
    return PlatformPowerIsEnabled(rail); // 读回输出锁存；仍不能推断负载电压。
#else
    (void)rail; // 裸板固件始终拒绝打开外接供电。
    return 0u;
#endif
}

uint8_t PlatformPowerIsEnabled(PlatformPowerRail_t rail)
{
#if defined(MC02_PROFILE_ROBOT)
    uint16_t pin;
    unsigned shift;
    if (!PlatformPowerPin(rail, &pin, &shift)) return 0u; // 未声明编号不能访问引脚。
    return (uint8_t)((((GPIOC->MODER >> (shift * 2u)) & 3u) == 1u) &&
                     ((GPIOC->ODR & pin) != 0u)); // 同时检查输出模式和锁存值。
#else
    (void)rail; // 裸板不提供外接电源的开启状态。
    return 0u;
#endif
}
