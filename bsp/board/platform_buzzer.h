/** @file platform_buzzer.h
 * @brief 板载蜂鸣器 PWM 的平台接口，不暴露定时器句柄。
 */
#ifndef MC02_PLATFORM_BUZZER_H
#define MC02_PLATFORM_BUZZER_H
#include <stdint.h>

/** @brief 初始化蜂鸣器输出，初始占空比为零。
 * @param[in] initial_frequency_hz 初始频率，单位 Hz，必须大于零。
 * @return 1 为可用或已初始化，0 为资源不可用或参数错误。
 * @note 在任务启动前调用；后续由唯一告警任务串行使用。
 */
uint8_t PlatformBuzzerInit(uint16_t initial_frequency_hz);

/** @brief 设置已初始化输出的频率与占空比。
 * @param[in] frequency_hz 频率，单位 Hz，必须大于零。
 * @param[in] duty_permille 占空比，范围 1～1000，1000 表示 100%。
 * @note 调用方须先检查参数并成功初始化；当前底层不报告运行时写入失败。
 */
void PlatformBuzzerSetTone(uint16_t frequency_hz, uint16_t duty_permille);

/** @brief 将已初始化输出的占空比设为零。
 * @note 调用方须先成功初始化；当前底层不报告运行时写入失败。
 */
void PlatformBuzzerSilence(void);
#endif
