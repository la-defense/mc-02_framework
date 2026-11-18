/** @file robot_monitor_plan.h
 * @brief 根据车型模块依赖准备当前任务监控配置。
 */
#ifndef MC02_ROBOT_MONITOR_PLAN_H
#define MC02_ROBOT_MONITOR_PLAN_H
#include "config/module_plan.h"
#include "task_monitor.h"
#if defined(MC02_PROFILE_BENCH_SAFE) || defined(MC02_PROFILE_COMMISSIONING)
#include "platform_periodic.h"

typedef enum {
    ROBOT_PERIODIC_DIAGNOSTIC = 0, /**< 既有整车诊断任务使用 RTOS 刻度。 */
#if MC02_BENCH_TIM5_DIAGNOSTICS
    ROBOT_PERIODIC_TIM5_PROBE, /**< 独立裸板自测使用 TIM5 比较通道。 */
#endif
    ROBOT_PERIODIC_COUNT
} RobotPeriodicDiagnostic_t;

/** @brief 检查本次构建启用的诊断周期和唤醒资源。
 * @param[out] failed 失败时写入具体诊断项，成功时保留原值；可以为 NULL。
 * @return READY 为通过，CONFIG 为周期或预算无效，RESOURCE 为硬件通道不足。
 * @note 板级时钟已启动且任务尚未创建时调用，只读取硬件状态。
 */
PlatformPeriodicResult_t RobotValidatePeriodicDiagnostics(RobotPeriodicDiagnostic_t *failed);

/** @brief 读取已经纳入启动检查的诊断周期。
 * @param[in] diagnostic 构建启用的诊断项。
 * @return 固件运行期间可读的配置，编号无效时返回 NULL。
 */
const PlatformPeriodicConfig_t *RobotGetDiagnosticPeriod(RobotPeriodicDiagnostic_t diagnostic);
#endif

/** @brief 将车型依赖映射到已选择任务的核心／辅助监控。
 * @param[in] plan 车型模块计划；生成配置必须非空，旧配置可传 NULL。
 * @return 任务监控初始化结果；生成配置无效时不初始化监控。
 * @note 只在调度器启动前调用；监控实现会复制临时配置。
 */
TaskMonitor_Result_e RobotConfigureTaskMonitor(const RobotModulePlan_t *plan);
#endif
