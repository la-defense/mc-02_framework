/** @file platform_periodic.h
 * @brief 固定周期的时钟计算和任务等待入口，不暴露 RTOS 类型。
 */
#ifndef MC02_PLATFORM_PERIODIC_H
#define MC02_PLATFORM_PERIODIC_H
#include <stdint.h>

typedef enum {
    PLATFORM_PERIODIC_READY = 0, /**< 返回最近到期的一拍。 */
    PLATFORM_PERIODIC_WAIT, /**< 下一拍尚未到期。 */
    PLATFORM_PERIODIC_CONFIG, /**< 周期、预算或计数分辨率无效。 */
    PLATFORM_PERIODIC_CONTEXT, /**< 当前无法在任务中等待。 */
    PLATFORM_PERIODIC_CLOCK /**< 计数倒退或后端提前结束等待。 */
} PlatformPeriodicResult_t;

typedef struct {
    uint32_t period_us; /**< 从一拍到下一拍的计划间隔，单位 μs，须可精确换算。 */
    uint32_t budget_us; /**< 从实际开始到完成的允许耗时，单位 μs，须可精确换算。 */
} PlatformPeriodicConfig_t;

/** @brief 单一任务持有的固定周期状态，初始化后由接口更新。 */
typedef struct {
    uint32_t period_ticks; /**< 每拍间隔，单位为计数刻度。 */
    uint32_t budget_ticks; /**< 单轮执行时间预算，单位为计数刻度。 */
    uint32_t next_tick; /**< 下一计划时刻，允许 32 位回绕。 */
    uint32_t last_tick; /**< 最近一次观察时刻，用于拒绝逆序读数。 */
} PlatformPeriodicClock_t;

typedef struct {
    uint32_t scheduled_tick; /**< 本拍计划时刻；等待时为下一拍。 */
    uint32_t observed_tick; /**< 确认本拍可运行时的实际计数。 */
    uint32_t skipped; /**< 本轮放弃的过时拍数。 */
    uint32_t wait_ticks; /**< 下一拍之前的剩余刻度数；就绪时为 0。 */
} PlatformPeriodicRelease_t;

typedef struct {
    uint32_t work_ticks; /**< 从就绪到完成经过的刻度数。 */
    uint8_t over_budget; /**< 执行时间严格超过预算时为 1。 */
    uint8_t missed_deadline; /**< 完成时刻严格晚于下一计划拍时为 1。 */
} PlatformPeriodicTiming_t;

/** @brief 从给定时刻开始安排固定周期，第一拍立即到期。
 * @param[out] clock 成功才改写，失败保留原值。
 * @param[in] config 周期和预算，二者均为正且预算不超过周期。
 * @param[in] quantum_us 后端的一个计数刻度时长，单位 μs，非零。
 * @param[in] start_tick 第一拍在本计数器中的时刻。
 * @return READY 为成功，CONFIG 为参数错误或数值无法用此时基准确表示。
 */
PlatformPeriodicResult_t PlatformPeriodicInit(PlatformPeriodicClock_t *clock,
    const PlatformPeriodicConfig_t *config, uint32_t quantum_us, uint32_t start_tick);

/** @brief 判断一拍是否可运行，跳过已错过的拍，只返回最近到期的一拍。
 * @param[in,out] clock 当前周期状态；单一任务使用，不可为空。
 * @param[in] now_tick 当前时刻，相邻观察的间隔最多 INT32_MAX 个刻度，允许回绕。
 * @param[out] release READY 或 WAIT 时写入，错误时保留原值。
 * @return READY、WAIT、CONFIG 或 CLOCK；不会睡眠或调用任务代码。
 */
PlatformPeriodicResult_t PlatformPeriodicPoll(PlatformPeriodicClock_t *clock, uint32_t now_tick,
    PlatformPeriodicRelease_t *release);

/** @brief 检查刚运行的一拍是否超出执行预算或计划截止时间。
 * @param[in] clock 就绪后尚未领取下一拍的状态。
 * @param[in] release 最近一次 READY 的结果。
 * @param[in] finish_tick 本拍完成时刻，使用同一计数器。
 * @param[out] timing 成功时写入，失败保留原值。
 * @return READY 为已计算，CONFIG 或 CLOCK 为参数或时刻错误。
 */
PlatformPeriodicResult_t PlatformPeriodicFinish(const PlatformPeriodicClock_t *clock,
    const PlatformPeriodicRelease_t *release, uint32_t finish_tick, PlatformPeriodicTiming_t *timing);

/** @brief 用任务调度器的刻度初始化周期状态，要求调度器正在运行。
 * @param[out] clock 成功时写入，以当前调度器时刻为第一拍。
 * @param[in] config 周期与执行预算；低于当前 RTOS 分辨率时拒绝。
 * @return READY 为成功，CONFIG 或 CONTEXT 为配置或调用位置错误。
 */
PlatformPeriodicResult_t PlatformTaskPeriodicInit(PlatformPeriodicClock_t *clock,
    const PlatformPeriodicConfig_t *config);

/** @brief 等待下一拍或领取最近到期的一拍，只在正常任务上下文使用。
 * @param[in,out] clock 由 PlatformTaskPeriodicInit 初始化的任务私有状态。
 * @param[out] release READY 时写入，失败时保留原值。
 * @return READY 或具体错误；不会补跑已错过的旧拍。
 * @note 调用前须退出临界区，并解除调用者设置的中断屏蔽。
 */
PlatformPeriodicResult_t PlatformTaskPeriodicWait(PlatformPeriodicClock_t *clock,
    PlatformPeriodicRelease_t *release);
#endif
