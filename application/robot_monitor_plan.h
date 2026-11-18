/** @file robot_monitor_plan.h
 * @brief 根据车型模块依赖准备当前任务监控配置。
 */
#ifndef MC02_ROBOT_MONITOR_PLAN_H
#define MC02_ROBOT_MONITOR_PLAN_H
#include "config/build_selection.h"
#include "config/chassis_task_config.h"
#include "config/module_plan.h"
#include "task_monitor.h"
#include "platform_periodic.h"
#if MC02_TASK_MOTOR_SPLIT
#include "../modules/motor/motor_task.h"
#endif

typedef enum {
    ROBOT_PERIODIC_DIAGNOSTIC = 0, /**< 整车诊断及状态汇总。 */
    ROBOT_PERIODIC_INS, /**< 姿态采样与解算。 */
    ROBOT_PERIODIC_COMM, /**< 裸板通信派发。 */
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

#if defined(MC02_PROFILE_BENCH_SAFE) || defined(MC02_PROFILE_COMMISSIONING)
/** @brief 检查本次启动需要的周期和唤醒资源。
 * @param[in] modules 启用模块的计划；旧配置可传 NULL。
 * @param[out] failed 失败时写入具体诊断项，成功时保留原值；可以为 NULL。
 * @return READY 为通过，CONFIG 为周期或预算无效，RESOURCE 为硬件通道不足。
 * @note 板级时钟已启动且任务尚未创建时调用，只读取硬件状态。
 */
PlatformPeriodicResult_t RobotValidateTaskPeriods(const RobotModulePlan_t *modules, RobotPeriodicTask_t *failed);
#endif

/** @brief 将车型依赖映射到已选择任务的核心／辅助监控。
 * @param[in] plan 车型模块计划；生成配置必须非空，旧配置可传 NULL。
 * @param[in] chassis 独立底盘任务配置；未选择独立底盘任务时忽略。
 * @return 任务监控初始化结果；依赖或底盘配置无效时不初始化监控。
 * @note 只在调度器启动前调用；监控实现会复制临时配置。
 */
TaskMonitor_Result_e RobotConfigureTaskMonitor(const RobotModulePlan_t *plan,
    const RobotChassisTaskConfig_t *chassis);

#if MC02_TASK_CHASSIS
/** @brief 检查底盘周期能否由当前 RTOS 表示，以及监控和栈参数是否完整。
 * @param[in] config 车型的只读底盘任务配置。
 * @return READY 为可用，CONFIG 为缺失或非法配置。
 */
PlatformPeriodicResult_t RobotValidateChassisTask(const RobotChassisTaskConfig_t *config);

/** @brief 取得启动时经监控接受的底盘任务配置。
 * @return 成功配置后指向本次运行使用的只读副本；未计划底盘时返回 NULL。
 */
const RobotChassisTaskConfig_t *RobotGetChassisTaskConfig(void);
#endif
#endif
