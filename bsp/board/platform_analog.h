/** @file platform_analog.h
 * @brief 逻辑 PWM 和模拟电压的框架接口，不暴露芯片句柄。
 */
#ifndef MC02_PLATFORM_ANALOG_H
#define MC02_PLATFORM_ANALOG_H
#include "platform_resources.h"
#include <stdint.h>

/** @brief 查询逻辑 PWM 是否存在板级绑定。
 * @param[in] channel 逻辑通道。
 * @return 1 表示已绑定，0 表示未绑定；不启动输出。
 */
uint8_t PlatformPwmIsBound(PlatformPwmChannel_t channel);

/** @brief 查询逻辑模拟输入是否存在板级绑定。
 * @param[in] channel 逻辑模拟输入。
 * @return 1 表示已绑定，0 表示未绑定；不表示采样有效。
 */
uint8_t PlatformAdcIsBound(PlatformAdcChannel_t channel);

/** @brief 设置已绑定 PWM 通道的原始比较值。
 * @param[in] channel 逻辑通道。
 * @param[in] compare 计数器比较值；有效范围由使用模块和板级周期共同确定。
 * @return 1 为已写入，0 为通道未绑定；不启动 PWM。
 */
uint8_t PlatformPwmSetCompare(PlatformPwmChannel_t channel, uint32_t compare);

/** @brief 启动已绑定 PWM 通道。
 * @param[in] channel 逻辑通道。
 * @return 1 表示后端接受启动，0 表示未绑定或后端拒绝。
 * @note 调用者须先设置安全输出；异常直接关断路径不依赖此接口。
 */
uint8_t PlatformPwmStart(PlatformPwmChannel_t channel);

/** @brief 获取已校验且未过期的模拟电压。
 * @param[in] channel 逻辑模拟输入。
 * @param[out] volts 成功时写入电压，单位 V；失败时保持原值。
 * @return 1 表示可使用，0 表示未绑定、采样或换算不可用。
 */
uint8_t PlatformAdcReadVolts(PlatformAdcChannel_t channel, float *volts);
#endif
