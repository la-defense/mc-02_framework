/** @file can_recovery.h
 * @brief 按总线、必需反馈和新命令判断 CAN 故障后的恢复条件。
 */
#ifndef MC02_CAN_RECOVERY_H
#define MC02_CAN_RECOVERY_H

#include <stdint.h>

#define CAN_RECOVERY_MAX_FEEDBACK 16u

typedef enum {
    CAN_RECOVERY_WAIT_BUS = 0, /**< 等待总线和外部许可可用。 */
    CAN_RECOVERY_WAIT_FEEDBACK, /**< 等待每个必需反馈更新。 */
    CAN_RECOVERY_STABILIZING, /**< 必需反馈持续有效，计时未结束。 */
    CAN_RECOVERY_WAIT_COMMAND, /**< 反馈已稳定，等待新的控制命令。 */
    CAN_RECOVERY_READY /**< 条件仍成立，允许调用方继续检查其他许可。 */
} CanRecoveryPhase_t;

typedef struct {
    uint32_t required_feedback_mask; /**< 必需反馈位图，最多 16 路，不能为零。 */
    uint32_t stable_ms; /**< 反馈连续有效时间，单位 ms，范围 1～INT32_MAX。 */
} CanRecoveryConfig_t;

typedef struct {
    uint32_t now_ms; /**< 本机时刻，单位 ms，允许 32 位回绕。 */
    uint32_t valid_feedback_mask; /**< 调用方已检查新鲜度的反馈位图。 */
    uint32_t feedback_sequence[CAN_RECOVERY_MAX_FEEDBACK]; /**< 每路采样序号，0 表示尚无样本。 */
    uint32_t command_sequence; /**< 当前合法且未过期的命令序号；无有效命令时填 0。 */
    uint32_t inhibit_generation; /**< 总线或核心禁止事件的代际；每次发生时递增。 */
    uint8_t bus_ok; /**< 总线实际可收发且旧硬件帧已处置时为 1。 */
    uint8_t permit_ok; /**< 外部使能及核心检查当前成立时为 1。 */
} CanRecoveryInput_t;

typedef struct {
    CanRecoveryConfig_t config; /**< 初始化时复制，运行中保持不变。 */
    uint32_t baseline[CAN_RECOVERY_MAX_FEEDBACK]; /**< 每路禁止时刻的反馈序号。 */
    uint32_t last_feedback[CAN_RECOVERY_MAX_FEEDBACK]; /**< 最近观察到的各路非零反馈序号。 */
    uint32_t seen_mask; /**< 禁止后已经更新过的反馈位图。 */
    uint32_t stable_start_ms; /**< 所有反馈连续有效的开始时刻。 */
    uint32_t command_baseline; /**< 反馈稳定时的命令序号。 */
    uint32_t inhibit_generation; /**< 上次观察到的禁止事件代际。 */
    CanRecoveryPhase_t phase; /**< 当前恢复阶段。 */
    uint8_t initialized; /**< 防止未初始化实例取得许可。 */
    uint8_t baseline_captured; /**< 尚未采集首次禁止时刻时为 0。 */
} CanRecovery_t;

/** @brief 设置必需反馈及连续有效时长，初始状态不允许控制。
 * @param[out] state 调用者持有的实例；失败时保持原值。
 * @param[in] config 配置；不能为 NULL，反馈位图和时长须在允许范围。
 * @return 1 为初始化成功，0 为参数错误。
 */
uint8_t CanRecoveryInit(CanRecovery_t *state, const CanRecoveryConfig_t *config);

/** @brief 按当前总线、反馈、外部许可和命令推进恢复阶段。
 * @param[in,out] state 已初始化实例，由单一任务持有。
 * @param[in] input 同一轮读取的输入；反馈有效性和命令合法性由来源检查。
 * @return 1 为恢复条件成立；参数错误或任一条件未满足返回 0。
 * @note 本函数不操作总线、队列或执行器，也不代替整车安全管理。调用方须先撤销故障前排队数据，
 *       确认硬件旧帧处置，按来源原始时间检查反馈和命令期限；代际须记录两次调用之间的短故障。
 *       来源重启或切换需重新初始化本实例；来源采集与最终许可发布须由调用方保证一致。
 */
uint8_t CanRecoveryStep(CanRecovery_t *state, const CanRecoveryInput_t *input);

/** @brief 带原始时间和来源切换检查的恢复状态；由单一协调任务持有。 */
typedef struct {
    CanRecovery_t recovery; /**< 原有总线和逐台反馈恢复状态。 */
    uint32_t source_id; /**< 最近采用的非零命令来源编号。 */
    uint32_t source_epoch; /**< 来源选择变化代次，允许自然回绕。 */
    uint32_t command_barrier_ms; /**< 反馈稳定检查完成时的本机时间。 */
    uint8_t barrier_valid; /**< 区分尚未建立起点和起点恰好为零。 */
} CanRecoveryTimed_t;

/** @brief 含真实来源时间的恢复输入。 */
typedef struct {
    CanRecoveryInput_t core; /**< 总线、反馈、资格和命令序号。 */
    uint32_t command_source_id; /**< 来源编号，0 表示没有有效来源。 */
    uint32_t command_source_epoch; /**< 来源选择变化时递增，防止周期之间的短暂切换被漏掉。 */
    uint32_t command_source_ms; /**< 原始来源接收时间，单位本机 ms。 */
    uint32_t command_max_age_ms; /**< 来源允许的最大年龄，范围 1～INT32_MAX ms。 */
} CanRecoveryTimedInput_t;

/** @brief 初始化带来源时间检查的恢复实例。
 * @param[out] state 单一任务持有的状态，失败不修改。
 * @param[in] config 逐台反馈位图和稳定时间。
 * @return 1 为成功，0 为参数或配置错误。
 */
uint8_t CanRecoveryTimedInit(CanRecoveryTimed_t *state, const CanRecoveryConfig_t *config);

/** @brief 检查新命令的来源、原始时间和原有恢复条件。
 * @param[in,out] state 已初始化的单一任务状态。
 * @param[in] input 完整输入；原始时间必须已转换为本机时钟。
 * @return 1 为反馈稳定后收到对应来源的新命令，0 为不允许恢复或输入错误。
 * @note 来源切换重新验证反馈；稳定前产生但延迟交付的命令不能接管。此函数不解除故障。
 */
uint8_t CanRecoveryTimedStep(CanRecoveryTimed_t *state, const CanRecoveryTimedInput_t *input);

#endif
