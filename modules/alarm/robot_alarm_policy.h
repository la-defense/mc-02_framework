/** @file robot_alarm_policy.h
 * @brief 告警仲裁和按实际毫秒推进的纯策略接口，供运行模块及主机回归使用。
 * @note 本文件不访问 HAL；同一策略对象由调用者串行操作。
 * @author MC-02 project contributors
 * @copyright Copyright (c) HNU YueLu EC 2022 all rights reserved
 */
#ifndef ROBOT_ALARM_POLICY_H
#define ROBOT_ALARM_POLICY_H

#include "robot_alarm.h"

#define ROBOT_ALARM_KNOWN_FAULT_MASK 0x000001FFu

typedef struct
{
    RobotAlarmRobotState_t state;
    uint32_t fault_flags;
    uint8_t estop_latched;
} RobotAlarmInput_t;

typedef struct
{
    RobotAlarmWriteLedFn write_led;
    RobotAlarmWriteBuzzerFn write_buzzer;
    void *context;
} RobotAlarmOutputOps_t;

typedef struct
{
    const RobotAlarmConfig_t *config;
    RobotAlarmOutputOps_t output;
    RobotAlarmState_t state;
    uint8_t initialized;
    uint8_t led_output_valid;
    uint8_t led_r;
    uint8_t led_g;
    uint8_t led_b;
    uint8_t buzzer_output_valid;
    BuzzerNote_t buzzer_note;
    uint16_t buzzer_duty_permille;
    uint8_t mute_allowed;
    uint32_t mute_started_ms;
    uint32_t mute_duration_ms;
} RobotAlarmPolicy_t;

/** @brief 检查车型表、模式、段数、单段时长和周期总时长。
 * @param[in] config 待检查的只读配置。
 * @return 配置完整且在上限内为 OK，否则返回 CONFIG 或 ARGUMENT。
 */
RobotAlarmStatus_t RobotAlarmConfigValidate(const RobotAlarmConfig_t *config);
/** @brief 初始化策略并记录可用输出通道。
 * @param[out] policy 调用者长期保存的策略对象。
 * @param[in] config 长期只读车型配置。
 * @param[in] output 输出函数；不可用通道可将对应函数设为 NULL。
 * @param[in] led_available LED 驱动已初始化时非零。
 * @param[in] buzzer_available 蜂鸣器驱动已初始化时非零。
 * @return 成功为 OK 或 DEGRADED；失败不覆盖原策略对象。
 */
RobotAlarmStatus_t RobotAlarmPolicyInit(RobotAlarmPolicy_t *policy,
                                        const RobotAlarmConfig_t *config,
                                        const RobotAlarmOutputOps_t *output,
                                        uint8_t led_available,
                                        uint8_t buzzer_available);
/** @brief 选择最高告警并按 now_ms 计算两个输出，成功后提交新状态。
 * @param[in,out] policy 已初始化策略对象。
 * @param[in] input 同一 safety 临界区复制的状态、fault 和急停标记。
 * @param[in] now_ms 当前本机毫秒计数，可从零开始并允许 uint32_t 回绕。
 * @return 完整输出为 OK；通道忙、故障或已禁用为 DEGRADED。
 */
RobotAlarmStatus_t RobotAlarmPolicyUpdate(RobotAlarmPolicy_t *policy,
                                          const RobotAlarmInput_t *input,
                                          uint32_t now_ms);
/** @brief 请求当前故障静音；仅接受车型允许的等级和有限时长。
 * @param[in,out] policy 已初始化策略对象。
 * @param[in] now_ms 当前本机毫秒计数。
 * @param[in] duration_ms 静音时长，范围 1～配置上限。
 * @return 成功为 OK；当前告警不可静音或时长越界时返回明确错误。
 */
RobotAlarmStatus_t RobotAlarmPolicyMuteFor(RobotAlarmPolicy_t *policy,
                                           uint32_t now_ms,
                                           uint32_t duration_ms);
/** @brief 复制策略状态；查询不推进 pattern。
 * @param[in] policy 已初始化策略对象。
 * @param[out] state 输出对象，失败时不改写。
 * @return 成功为 OK，参数或初始化错误时返回错误。
 */
RobotAlarmStatus_t RobotAlarmPolicyGetState(const RobotAlarmPolicy_t *policy,
                                            RobotAlarmState_t *state);

#endif /* ROBOT_ALARM_POLICY_H */
