/** @file platform_clock_guard.h
 * @brief 限制启动时 HAL 时钟配置的总等待次数和周期数。
 */
#ifndef MC02_PLATFORM_CLOCK_GUARD_H
#define MC02_PLATFORM_CLOCK_GUARD_H
#include <stdint.h>

/* 必须大于所接入 HAL 主时钟等待的最大超时，单位 ms。 */
#define CLOCK_GUARD_EXPIRED_STEP_MS 6001u

/** @brief 一次时钟配置期间的固定期限和计数。 */
typedef struct {
    uint32_t started_cycles; /**< 本次配置开始时的周期计数，允许回绕。 */
    uint32_t cycle_limit; /**< 固定周期上限，小于半个计数周期。 */
    uint32_t poll_limit; /**< HAL 查询时间的最大次数。 */
    uint32_t polls; /**< 自开始以来的查询次数，不随时钟变化清零。 */
    uint32_t forced_offset; /**< 超时后让 HAL 退出的临时毫秒偏移。 */
    uint8_t expired; /**< 周期或查询次数达到上限后保持为 1。 */
} PlatformClockGuard_t;

/** @brief 开始一次有界时钟配置。
 * @param[out] guard 非空状态对象，由启动阶段独占。
 * @param[in] cycles 启动时读取的 32 位 CPU 周期计数。
 * @param[in] cycle_limit 固定周期上限，范围 1～INT32_MAX。
 * @param[in] poll_limit 最大时间查询次数，必须大于零。
 * @return 参数有效返回 1；无效返回 0 且不修改状态。
 */
uint8_t PlatformClockGuardBegin(PlatformClockGuard_t *guard, uint32_t cycles,
                                uint32_t cycle_limit, uint32_t poll_limit);

/** @brief 读取供 HAL 本次时钟配置使用的毫秒值。
 * @param[in,out] guard 已初始化且非空的状态对象。
 * @param[in] raw_tick 未经修改的 HAL 毫秒计数。
 * @param[in] cycles 同一周期计数器的当前值。
 * @return 正常时返回 raw_tick；到期后返回带临时偏移的值，让 HAL 固定超时检查退出。
 * @note 外层必须检查 expired 并拒绝成功；此值只供启动配置使用，不写回真实 HAL 时钟。
 */
uint32_t PlatformClockGuardTick(PlatformClockGuard_t *guard, uint32_t raw_tick,
                                uint32_t cycles);
/** @brief 以指定步长推进到期后的临时 HAL 时间。
 * @param[in,out] guard 已初始化的状态；NULL 时原样返回 raw_tick。
 * @param[in] raw_tick 实际 HAL 毫秒计数。
 * @param[in] cycles 当前 CPU 周期计数。
 * @param[in] expired_step 到期后每次查询增加的毫秒数，必须大于调用方所用的 HAL 超时。
 * @return 到期前返回 raw_tick，到期后返回带临时偏移的时间。
 * @note 外层仍须检查 expired；本接口不修改真实 HAL 时钟。
 */
uint32_t PlatformClockGuardTickStep(PlatformClockGuard_t *guard, uint32_t raw_tick,
                                    uint32_t cycles, uint32_t expired_step);
#endif
