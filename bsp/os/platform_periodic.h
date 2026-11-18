/** @file platform_periodic.h
 * @brief 固定周期的时钟计算和任务等待入口，不暴露 RTOS 类型。
 */
#ifndef MC02_PLATFORM_PERIODIC_H
#define MC02_PLATFORM_PERIODIC_H
#include <stddef.h>
#include <stdint.h>

typedef enum {
    PLATFORM_PERIODIC_READY = 0, /**< 返回最近到期的一拍。 */
    PLATFORM_PERIODIC_WAIT, /**< 下一拍尚未到期。 */
    PLATFORM_PERIODIC_CONFIG, /**< 周期、预算或计数分辨率无效。 */
    PLATFORM_PERIODIC_CONTEXT, /**< 当前无法在任务中等待。 */
    PLATFORM_PERIODIC_CLOCK, /**< 计数倒退、后端提前结束等待或唤醒超时。 */
    PLATFORM_PERIODIC_RESOURCE /**< 计时资源已占用或无法建立等待。 */
} PlatformPeriodicResult_t;

typedef struct {
    uint32_t period_us; /**< 从一拍到下一拍的计划间隔，单位 μs，须可精确换算。 */
    uint32_t budget_us; /**< 从实际开始到完成的允许耗时，单位 μs，须可精确换算。 */
} PlatformPeriodicConfig_t;

typedef enum {
    PLATFORM_PERIODIC_SOURCE_AUTO = 0, /**< 优先用能精确表示周期和预算的 RTOS 刻度。 */
    PLATFORM_PERIODIC_SOURCE_RTOS, /**< 必须使用 RTOS 刻度。 */
    PLATFORM_PERIODIC_SOURCE_HIGH_RES /**< 必须使用硬件比较通道。 */
} PlatformPeriodicSource_t;

typedef struct {
    PlatformPeriodicConfig_t timing; /**< 周期和预算均以 μs 给出。 */
    PlatformPeriodicSource_t source; /**< 启动时允许选择的计数来源。 */
} PlatformPeriodicRequest_t;

typedef struct {
    uint32_t rtos_quantum_us; /**< RTOS 每刻度的 μs 数，0 表示不可用。 */
    uint32_t high_res_quantum_us; /**< 硬件每刻度的 μs 数，0 表示后端未接入。 */
    uint32_t high_res_channels; /**< 当前可供独立任务使用的硬件通道数。 */
} PlatformPeriodicCapabilities_t;

/** @brief 查询周期来源的实际分辨率和可用硬件通道数。
 * @return 当前平台可用于启动检查的能力。
 * @note 板级时钟初始化后、周期任务创建前调用；只读，不占用通道。
 */
PlatformPeriodicCapabilities_t PlatformTaskPeriodicCapabilities(void);

/** @brief 预检整组已启用任务是否可按要求建立固定周期。
 * @param[in] requests 按任务顺序提供配置，空计划可传 NULL。
 * @param[in] count 请求数量，最多 32 项。
 * @param[in] capabilities 各来源的实际刻度与可用通道数，不可为空。
 * @param[out] hardware_mask 成功时写入需要硬件通道的任务位图，失败保留原值。
 * @param[out] failed_index 可选。单项出错时写入其索引，调用参数有误或成功时保持原值。
 * @return READY 为整组可运行；CONFIG 为配置无法精确表示；RESOURCE 为通道不足。
 */
PlatformPeriodicResult_t PlatformPeriodicPlan(const PlatformPeriodicRequest_t *requests, size_t count,
    const PlatformPeriodicCapabilities_t *capabilities, uint32_t *hardware_mask, size_t *failed_index);

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

/** @brief 用当前 RTOS 刻度检查刚完成的一拍。
 * @param[in] clock 由 PlatformTaskPeriodicInit 建立并已领取一拍的状态。
 * @param[in] release 最近一次成功等待返回的拍点。
 * @param[out] timing 成功时写入耗时和是否超期，失败保持原值。
 * @return READY 为成功，或返回上下文、配置及计数错误。
 */
PlatformPeriodicResult_t PlatformTaskPeriodicFinish(const PlatformPeriodicClock_t *clock,
    const PlatformPeriodicRelease_t *release, PlatformPeriodicTiming_t *timing);

/** @brief 申请平台硬件计时资源，并使用硬件刻度建立第一拍。
 * @param[out] clock 成功才写入；必须在本次使用期间保持有效且不可直接改写。
 * @param[in] config 固定周期与预算，单位 μs。
 * @return READY 为成功，其他值说明配置、资源或调用上下文错误。
 * @note 仅在启用相应平台后端的构建中提供。每个任务最多申请一次，通道耗尽返回 RESOURCE。
 *       由申请任务负责等待和释放，在删除任务前须先释放。
 */
PlatformPeriodicResult_t PlatformTaskHighResOpen(PlatformPeriodicClock_t *clock,
    const PlatformPeriodicConfig_t *config);

/** @brief 在创建使用硬件唤醒的任务前查询空闲通道数。
 * @return 硬件时钟和中断符合要求时的可分配数量，否则为 0。
 * @note 仅在启用硬件唤醒后端的构建中提供；在创建周期任务前调用，此查询不申请通道。
 */
uint32_t PlatformTaskHighResAvailableChannels(void);

/** @brief 用硬件定时器等待下一拍，阻塞由 RTOS 提供有限超时。
 * @param[in,out] clock 已成功申请的任务私有周期状态，失败时保持原值。
 * @param[out] release 成功交付最近一个到期拍点，失败时保持原值。
 * @return READY 或具体失败原因；不补跑错过的旧拍。
 * @note 调用前须退出临界区，并解除调用者设置的中断屏蔽。
 */
PlatformPeriodicResult_t PlatformTaskHighResWait(PlatformPeriodicClock_t *clock,
    PlatformPeriodicRelease_t *release);

/** @brief 停止本任务的硬件比较并释放通道，最后一次释放才停止共享计数器。
 * @param[in] clock 本任务申请的状态地址，不可为空。
 * @return READY 为成功，错误调用不会释放其他任务的资源。
 */
PlatformPeriodicResult_t PlatformTaskHighResClose(const PlatformPeriodicClock_t *clock);
#endif
