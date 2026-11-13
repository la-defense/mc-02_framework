/** @file robot_can_capacity.h
 * @brief 在注册设备前规划各模块所需的经典 CAN 过滤器。
 */
#ifndef MC02_ROBOT_CAN_CAPACITY_H
#define MC02_ROBOT_CAN_CAPACITY_H
#include "module_plan.h"
#include "platform_resources.h"
#include <stdint.h>

typedef struct {
    RobotModule_t module; /**< 消耗过滤器的模块。 */
    PlatformCanBus_t bus; /**< 模块使用的逻辑总线。 */
    uint8_t standard; /**< 所需标准 ID 槽数。 */
    uint8_t extended; /**< 所需扩展 ID 槽数。 */
} RobotCanFilterDemand_t;

typedef struct {
    uint8_t standard[3]; /**< 三条总线各自的标准 ID 容量。 */
    uint8_t extended[3]; /**< 三条总线各自的扩展 ID 容量。 */
    uint8_t endpoints; /**< 全部总线共用的端点登记容量。 */
} RobotCanFilterCapacity_t;

typedef struct {
    uint32_t rejected_modules; /**< 容量不足而未获资源的辅助模块位图。 */
    uint8_t failed_core_module; /**< 核心失败模块；无核心失败时为 ROBOT_MODULE_COUNT。 */
} RobotCanCapacityResult_t;

/** @brief 先为核心依赖、再为辅助模块分配过滤器容量。
 * @param[in] plan 已声明的车型模块计划，不可为 NULL。
 * @param[in] demands 模块需求数组；count 为零时可为 NULL。
 * @param[in] count 需求项数量，最多 8 项。
 * @param[in] capacity 各总线与全局容量，不可为 NULL。
 * @param[out] result 有效输入时写入结果；参数或依赖计划错误时保持原值。
 * @return OK 表示核心容量足够（辅助可能被拒绝）；CORE_FAILURE 表示核心不足；其他值表示输入无效。
 * @note 同一模块的多项需求整体接受或拒绝；不注册设备，也不更改硬件。
 */
RobotModuleStatus_t RobotCanCapacityPlan(const RobotModulePlan_t *plan,
    const RobotCanFilterDemand_t *demands, uint8_t count,
    const RobotCanFilterCapacity_t *capacity, RobotCanCapacityResult_t *result);
#endif
