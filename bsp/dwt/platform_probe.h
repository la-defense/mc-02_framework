/** @file platform_probe.h
 * @brief 短时耗时记录和原有周期差值的框架接口。
 */
#ifndef MC02_PLATFORM_PROBE_H
#define MC02_PLATFORM_PROBE_H
#include <stdint.h>

/** @brief 单一执行者更新的微秒统计，跨字段读取不保证同时一致。 */
typedef struct {
    volatile uint32_t last_us; /**< 最近一次耗时，单位微秒。 */
    volatile uint32_t max_us; /**< 记录以来的最大耗时，单位微秒。 */
    volatile uint32_t total_us; /**< 累计微秒，32 位回绕。 */
    volatile uint32_t calls; /**< 完成次数，32 位回绕。 */
} PlatformProbe_t;

/** @brief 读取当前周期计数，供短时探针使用。
 * @return 原始 32 位周期计数；一次测量不能跨越完整回绕周期。
 */
uint32_t PlatformProbeStart(void);

/** @brief 更新一次短时耗时记录。
 * @param[in,out] probe 非空、初始化为零且不并发更新的统计对象。
 * @param[in] start 本次开始时的周期计数。
 */
void PlatformProbeDone(volatile PlatformProbe_t *probe, uint32_t start);

/** @brief 返回距上次调用的秒数并更新调用者的周期计数。
 * @param[in,out] previous 非空的上次周期计数，首次使用前置零。
 * @return 沿用底层计时器计算的秒数。
 */
float PlatformProbeDeltaSeconds(uint32_t *previous);
#endif
