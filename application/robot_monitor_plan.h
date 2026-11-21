/** @file robot_monitor_plan.h
 * @brief 根据车型模块依赖准备当前任务监控配置。
 */
#ifndef MC02_ROBOT_MONITOR_PLAN_H
#define MC02_ROBOT_MONITOR_PLAN_H
#include "config/build_selection.h"
#include "config/control_task_config.h"
#include "config/module_plan.h"
#include "task_monitor.h"
#include "platform_periodic.h"
#if MC02_TASK_MOTOR_SPLIT
#include "../modules/motor/motor_task.h"
#endif

typedef enum {
    ROBOT_PERIODIC_DIAGNOSTIC = 0, /**< 整车诊断及状态汇总。 */
    ROBOT_PERIODIC_INS, /**< 姿态采样与解算。 */
    ROBOT_PERIODIC_COMM, /**< 通信派发，分开的电机计算和发送也用此拍点轮询。 */
    ROBOT_PERIODIC_DAEMON, /**< 设备维护及任务监控。 */
#if MC02_BENCH_TIM5_DIAGNOSTICS
    ROBOT_PERIODIC_TIM5_PROBE, /**< 独立裸板自测使用 TIM5 比较通道。 */
#endif
    ROBOT_PERIODIC_COUNT
} RobotPeriodicTask_t;

/** @brief 读取周期任务的名义间隔及预算。
 * @param[in] task 周期任务编号。
 * @return 固件运行期间可读的配置，编号无效时返回 NULL。
 */
const PlatformPeriodicConfig_t *RobotGetTaskPeriod(RobotPeriodicTask_t task);

#if MC02_TASK_MOTOR_SPLIT
/** @brief 启动时校验并保存两个电机周期。
 * @param[in] periods 只读车型配置。
 * @return READY 为已保存；失败保留旧配置。
 */
PlatformPeriodicResult_t RobotPrepareMotorTaskPeriods(const MotorTaskPeriods_t *periods);

/** @brief 取得启动时接受的计算和发送周期。
 * @return 固定只读副本；尚未准备时返回 NULL。
 */
const MotorTaskPeriods_t *RobotGetMotorTaskPeriods(void);
#endif

#if MC02_TASK_FIXED_REGULAR_PERIODS
/** @brief 检查本次启动需要的周期和唤醒资源。
 * @param[in] modules 启用模块的计划；旧配置可传 NULL。
 * @param[out] failed 失败时写入具体诊断项，成功时保留原值；可以为 NULL。
 * @return READY 为通过，CONFIG 为周期或预算无效，RESOURCE 为硬件通道不足。
 * @note 板级时钟已启动且任务尚未创建时调用，只读取硬件状态。
 */
PlatformPeriodicResult_t RobotValidateTaskPeriods(const RobotModulePlan_t *modules, RobotPeriodicTask_t *failed);
#endif

/** @brief 独立控制任务的启动检查失败原因。 */
typedef struct {
    RobotControlTaskId_t task; /**< 失败的任务；未定位到控制任务时为 ROBOT_CONTROL_TASK_COUNT。 */
    PlatformPeriodicResult_t reason; /**< 该任务的时间配置无效或缺少平台资源。 */
} RobotControlTimingFailure_t;

/** @brief 将车型依赖映射到任务监控，并为独立控制任务统一分配唤醒资源。
 * @param[in] plan 车型模块计划；生成配置必须非空，旧配置可传 NULL。
 * @param[in] control 按 RobotControlTaskId_t 排列的车型配置；没有启用独立控制任务时可为 NULL。
 * @param[out] failure 具体控制任务及失败原因，可以为 NULL。
 * @return 任务监控初始化结果；依赖或时钟配置无效时不更新当前监控和任务配置。
 * @note 仅在启动准备阶段调用。监控和任务运行配置各自持有一份副本。
 */
TaskMonitor_Result_e RobotConfigureTaskMonitor(const RobotModulePlan_t *plan,
    const RobotControlTaskConfig_t *control, RobotControlTimingFailure_t *failure);

#if MC02_TASK_CONTROL
/** @brief 取得启动时接受的独立控制任务配置。
 * @param[in] task 独立控制任务编号。
 * @return 固定只读副本；未选择该任务时返回 NULL。
 */
const RobotControlTaskConfig_t *RobotGetControlTaskConfig(RobotControlTaskId_t task);
#endif
#endif
