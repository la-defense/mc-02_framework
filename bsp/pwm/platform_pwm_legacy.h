/** @file platform_pwm_legacy.h
 * @brief 旧 PWM 设备注册适配；新温控使用逻辑 PWM 通道。
 */
#ifndef MC02_PLATFORM_PWM_LEGACY_H
#define MC02_PLATFORM_PWM_LEGACY_H
#include <stdint.h>

/** @brief 用平台私有描述注册旧 PWM 设备。
 * @param[in] platform_config 平台后端定义的只读配置，不能为 NULL。
 * @param[in] owner 设备上下文，可为 NULL，设备存续时须有效。
 * @return 成功返回设备句柄；配置或资源错误返回 NULL。
 * @note 仅启动阶段串行调用；当前 MC-02 后端会沿用旧 PWM 的启动行为。
 */
void *PlatformLegacyPwmRegister(const void *platform_config, void *owner);

/** @brief 撤销先前注册的旧 PWM 设备。
 * @param[in] device 设备句柄；允许为 NULL。
 * @return 成功返回 1，其他情况返回 0；成功后句柄失效。
 */
uint8_t PlatformLegacyPwmUnregister(void *device);
#endif
