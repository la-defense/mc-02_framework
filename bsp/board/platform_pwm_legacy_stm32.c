/** @file platform_pwm_legacy_stm32.c
 * @brief 旧 PWM 配置留在 MC-02 板级后端，复用原有注册与释放。
 */
#include "../pwm/platform_pwm_legacy.h"
#include "bsp_pwm.h"
#include <stddef.h>

void *PlatformLegacyPwmRegister(const void *platform_config, void *owner)
{
    if (platform_config == NULL) return NULL; // 空配置不能启动 PWM。
    PWM_Init_Config_s config = *(const PWM_Init_Config_s *)platform_config; // 在本地副本设置设备上下文。
    config.id = owner; // 保留旧设备回调访问的上下文。
    return PWMRegister(&config); // 延续原注册、占空比和失败处理。
}

uint8_t PlatformLegacyPwmUnregister(void *device)
{
    return PWMUnregister(device); // 延续置零、停止与释放行为。
}
