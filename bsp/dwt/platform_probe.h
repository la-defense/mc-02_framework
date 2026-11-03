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

/** @brief 在读取 BMI088 标定样本之间忙等 500 微秒。
 * @note 只保留现有短时等待，调用前须初始化平台计数器；不用于长时间延时。
 */
void PlatformProbeDelay500Us(void);

/** @brief 保留 HT 驱动初始化／置零中的旧忙等。
 * @param[in] milliseconds 等待时间，单位毫秒；当前调用为 5 或 50。
 * @note 调用前须初始化平台计数器；等待期间不让出处理器，不用于新的周期任务。
 */
void PlatformHtLegacyDelayMs(uint32_t milliseconds);

/** @brief 读取原有的浮点毫秒运行时间，供低频诊断使用。
 * @return 自平台原有时基初始化起的毫秒数，精度随浮点数值增大而降低。
 * @note H723 后端保留原有 DWT 时间计算；不用于跨设备时钟比较。
 */
float PlatformProbeTimelineMs(void);

/** @brief 将一次短时测量的周期差换算为微秒。
 * @param[in] start PlatformProbeStart 返回的起点。
 * @return 沿用平台原有的 32 位微秒耗时结果。
 * @note 测量间隔不能跨越完整的周期计数回绕；时钟须先完成初始化。
 */
uint32_t PlatformProbeElapsedUs(uint32_t start);

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

/** @brief 将两个原始周期计数之间的差值换算为秒。
 * @param[in] start,end 同一平台时钟的周期计数，间隔必须小于一个计数回绕周期。
 * @param[out] seconds 成功时写入秒数；失败时保持原值。
 * @return 1 表示换算成功，0 表示输出为空或平台频率尚未就绪。
 * @note 无符号差值允许计数跨零一次；调用者另用较粗时间检查长时间间隔。
 */
uint8_t PlatformProbeElapsedSeconds(uint32_t start, uint32_t end, float *seconds);
#endif
