/** @file robot_alarm.h
 * @brief 声明基于 safety 快照选择 RGB 和蜂鸣器输出的车型告警配置与运行入口。
 * @note 运行入口由普通任务串行调用；配置及其中的表在运行期间保持只读。
 * @author MC-02 project contributors
 * @copyright Copyright (c) HNU YueLu EC 2022 all rights reserved
 */
#ifndef ROBOT_ALARM_H
#define ROBOT_ALARM_H

#include <stdint.h>

#define ROBOT_ALARM_MAX_SEGMENTS 8u
#define ROBOT_ALARM_MAX_PATTERN_MS 10000u
#define ROBOT_ALARM_MAX_MUTE_LEVELS 8u

typedef enum
{
    ROBOT_ALARM_OK = 0,
    ROBOT_ALARM_DEGRADED,
    ROBOT_ALARM_QUEUED,
    ROBOT_ALARM_ERROR_ARGUMENT,
    ROBOT_ALARM_ERROR_CONFIG,
    ROBOT_ALARM_ERROR_UNINITIALIZED,
    ROBOT_ALARM_ERROR_RANGE,
    ROBOT_ALARM_ERROR_MUTE_DENIED,
    ROBOT_ALARM_ERROR_BUSY,
    ROBOT_ALARM_ERROR_EVENT_CHANGED,
} RobotAlarmStatus_t;

typedef enum
{
    ROBOT_ALARM_STATE_STOP = 0,
    ROBOT_ALARM_STATE_INIT,
    ROBOT_ALARM_STATE_SAFE,
    ROBOT_ALARM_STATE_CALIB,
    ROBOT_ALARM_STATE_READY,
    ROBOT_ALARM_STATE_FAULT,
    ROBOT_ALARM_STATE_ESTOP,
    ROBOT_ALARM_STATE_COUNT,
} RobotAlarmRobotState_t;

typedef enum
{
    ROBOT_ALARM_LED_OFF = 0,
    ROBOT_ALARM_LED_SOLID,
    ROBOT_ALARM_LED_BLINK,
    ROBOT_ALARM_LED_BREATHE,
    ROBOT_ALARM_LED_SEGMENTED,
} RobotAlarmLedMode_t;

/** @brief 分段灯效结束后的处理；单次播放结束时保持最后一段颜色。 */
typedef enum
{
    ROBOT_ALARM_REPEAT_LOOP = 0,
    ROBOT_ALARM_REPEAT_ONCE_HOLD,
} RobotAlarmRepeat_t;

typedef enum
{
    BUZZER_SILENT = 0,
    OCTAVE_1,
    OCTAVE_2,
    OCTAVE_3,
    OCTAVE_4,
    OCTAVE_5,
    OCTAVE_6,
    OCTAVE_7,
    OCTAVE_8,
    BUZZER_NOTE_COUNT,
} BuzzerNote_t;

typedef struct
{
    uint16_t duration_ms; /**< 本段持续毫秒数，必须大于0。 */
    uint8_t r;            /**< 红色分量，范围0～255。 */
    uint8_t g;            /**< 绿色分量，范围0～255。 */
    uint8_t b;            /**< 蓝色分量，范围0～255。 */
} RobotAlarmLedSegment_t;

typedef struct
{
    uint16_t duration_ms; /**< 本段持续毫秒数，必须大于0。 */
    BuzzerNote_t note;    /**< BUZZER_SILENT 或 OCTAVE_1～OCTAVE_8。 */
    uint16_t duty_permille; /**< 发声占空比1～1000；静音时必须为0。 */
} RobotAlarmSoundSegment_t;

/** @brief 一种灯光时间线；字段不用的模式可为0，分段数组需长期只读。 */
typedef struct
{
    RobotAlarmLedMode_t mode; /**< 关、常亮、闪烁、呼吸或分段。 */
    uint16_t period_ms; /**< 闪烁／呼吸周期，范围2～max_pattern_ms。 */
    uint8_t r; /**< 非分段模式的红色，范围0～255。 */
    uint8_t g; /**< 非分段模式的绿色，范围0～255。 */
    uint8_t b; /**< 非分段模式的蓝色，范围0～255。 */
    RobotAlarmRepeat_t repeat; /**< 分段循环或单次保持末段；其他模式填默认0。 */
    uint8_t segment_count; /**< 分段数量，范围0～ROBOT_ALARM_MAX_SEGMENTS。 */
    const RobotAlarmLedSegment_t *segments; /**< 分段模式的只读数组；周期总长不得超限。 */
} RobotAlarmLedPattern_t;

/** @brief 蜂鸣器分段时间线；各段从当前告警共同起点计时并独立循环。 */
typedef struct
{
    uint8_t segment_count; /**< 分段数量，范围0～ROBOT_ALARM_MAX_SEGMENTS。 */
    const RobotAlarmSoundSegment_t *segments; /**< 长期只读数组；总时长不得超限。 */
} RobotAlarmSoundPattern_t;

/** @brief 单种状态或故障的灯声组合。 */
typedef struct
{
    RobotAlarmLedPattern_t led;
    RobotAlarmSoundPattern_t sound;
} RobotAlarmStyle_t;

typedef struct
{
    uint32_t fault_bit; /**< 一个已知故障位；配置表需覆盖全部九个已知位。 */
    uint8_t priority; /**< 数值越小优先级越高；相同值按数组顺序仲裁。 */
    uint8_t mute_level; /**< 静音等级0～7，由配置位图单独授权。 */
    RobotAlarmStyle_t style; /**< 此故障持续期间采用的灯声。 */
} RobotAlarmFaultRule_t;

typedef struct
{
    RobotAlarmRobotState_t state; /**< 必须恰好配置每个已知状态一次。 */
    RobotAlarmStyle_t style; /**< 无活动故障时采用的灯声。 */
} RobotAlarmStateRule_t;

/** @brief 车型告警表；本体和所有数组须在 RobotAlarmInit 至停机期间保持只读。 */
typedef struct
{
    uint8_t fault_rule_count; /**< 本版本必须为9。 */
    const RobotAlarmFaultRule_t *fault_rules; /**< 长期只读故障规则数组。 */
    uint8_t state_rule_count; /**< 必须等于 ROBOT_ALARM_STATE_COUNT。 */
    const RobotAlarmStateRule_t *state_rules; /**< 长期只读状态规则数组。 */
    RobotAlarmStyle_t unknown_fault_style; /**< 仅未知故障位使用的保底样式。 */
    RobotAlarmStyle_t unknown_state_style; /**< 仅非法状态值使用的保底样式。 */
    uint8_t mute_level_mask; /**< bit n 为1时允许静音等级n。 */
    uint32_t max_mute_ms; /**< 最大静音时长，范围1～60000ms。 */
    uint16_t max_pattern_ms; /**< 单个灯或声循环的最大总长，范围1～10000ms。 */
} RobotAlarmConfig_t;

typedef enum
{
    ROBOT_ALARM_EVENT_NONE = 0,
    ROBOT_ALARM_EVENT_ESTOP,
    ROBOT_ALARM_EVENT_FAULT,
    ROBOT_ALARM_EVENT_UNKNOWN_FAULT,
    ROBOT_ALARM_EVENT_STATE,
    ROBOT_ALARM_EVENT_UNKNOWN_STATE,
} RobotAlarmEventKind_t;

typedef struct
{
    RobotAlarmEventKind_t event_kind;
    uint32_t event_id;
    uint32_t event_started_ms;
    uint32_t event_generation;
    uint8_t led_available;
    uint8_t buzzer_available;
    uint8_t mute_active;
    uint8_t mute_level;
    uint8_t mute_allowed;
    RobotAlarmStatus_t last_status;
    RobotAlarmStatus_t last_mute_status;
    uint32_t led_busy_count;
    uint32_t buzzer_busy_count;
    uint32_t led_error_count;
    uint32_t buzzer_error_count;
} RobotAlarmState_t;

/** @brief 一次底层输出的提交结果；ERROR 会禁用该告警通道，BUSY 保留重试。 */
typedef enum
{
    ROBOT_ALARM_WRITE_APPLIED = 0,
    ROBOT_ALARM_WRITE_BUSY,
    ROBOT_ALARM_WRITE_ERROR,
} RobotAlarmWriteResult_t;

/** @brief 提交 RGB 请求；成功仅表示驱动接受，不能证明灯珠已显示。
 * @param[in] context 初始化时绑定的长期上下文，可为 NULL。
 * @param[in] r,g,b RGB 各分量，范围0～255。
 * @return APPLIED、BUSY或ERROR；策略会记录结果并处理对应通道。
 */
typedef RobotAlarmWriteResult_t (*RobotAlarmWriteLedFn)(void *context,
                                                         uint8_t r,
                                                         uint8_t g,
                                                         uint8_t b);
/** @brief 提交蜂鸣器音符和占空比；静音使用 BUZZER_SILENT 与零占空比。
 * @param[in] context 长期有效的驱动上下文；可为 NULL。
 * @param[in] note Do～高音 Do 的 OCTAVE_1～OCTAVE_8，或 BUZZER_SILENT。
 * @param[in] duty_permille 发声占空比，范围 1～1000；静音时必须为 0。
 * @return APPLIED、BUSY或ERROR；成功仅表示本机驱动已接受请求。
 */
typedef RobotAlarmWriteResult_t (*RobotAlarmWriteBuzzerFn)(void *context,
                                                            BuzzerNote_t note,
                                                            uint16_t duty_permille);

/** @brief 初始化板载告警输出；通道失败只会使对应告警降级。
 * @param[in] config 长期只读车型配置，必须覆盖全部已知 fault 和 robot state。
 * @return 两个通道可用时为 OK；部分或全部辅助通道不可用时为 DEGRADED；配置错误时失败。
 * @note 在 RobotSafetyInit() 之后、任务启动前调用。失败不设置 safety fault。
 */
RobotAlarmStatus_t RobotAlarmInit(const RobotAlarmConfig_t *config);
/** @brief 读取一致 safety 快照并推进当前告警一次。
 * @return 两个通道均可用为 OK；通道忙或不可用为 DEGRADED；未初始化时返回错误。
 * @note 在普通任务中调用，时间使用 HAL 毫秒计数；LED 发送异步推进，故障恢复按任务轮次检查。
 */
RobotAlarmStatus_t RobotAlarmTask(void);
/** @brief 为当前允许静音的故障提交一次限时静音请求。
 * @param[in] duration_ms 静音时长，范围 1～车型配置的最大值。
 * @return 成功排队为 QUEUED；请求会在下一次 RobotAlarmTask() 中处理。
 * @note 新告警切换会取消先前静音；静音不改变 LED、fault 或 safety 许可。
 */
RobotAlarmStatus_t RobotAlarmMuteFor(uint32_t duration_ms);
/** @brief 复制最近一次告警状态；读取不推进节奏。
 * @param[out] state 输出对象，不能为空；失败时保留原内容。
 * @return 成功为 OK，空参数或未初始化时返回错误。
 */
RobotAlarmStatus_t RobotAlarmGetState(RobotAlarmState_t *state);

#endif /* ROBOT_ALARM_H */
