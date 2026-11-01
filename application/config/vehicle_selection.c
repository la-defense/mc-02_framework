/** @file vehicle_selection.c
 * @brief 保存固定构建身份和启动结果；临界区仅用于复制少量状态位。
 */
#include "vehicle_selection.h"
#include "build_selection.h"
#include "platform_irq.h"
const RobotBuildIdentity_t robot_build_identity = {
    .layout = 0x56434631u, .version = MC02_VEHICLE_CONFIG_VERSION,
    .vehicle = MC02_VEHICLE_NAME, .platform = MC02_PLATFORM_NAME,
    .board = MC02_BOARD_NAME, .rtos = MC02_RTOS_NAME, .role = MC02_BUILD_ROLE_NAME,
    .enabled_modules = MC02_FEATURE_MASK, .required_modules = MC02_REQUIRED_MASK};
static const RobotModulePlan_t selected_plan = {
    .enabled = MC02_FEATURE_MASK, .required = MC02_REQUIRED_MASK,
    .dependencies = MC02_DEPENDENCY_INITIALIZER};
RobotModuleStartup_t robot_module_startup;
const RobotModulePlan_t *RobotBuildGetPlan(void) { return &selected_plan; }
RobotModuleStatus_t RobotBuildStart(RobotModuleStartFn_t start, void *context)
{
    return RobotModuleStart(&selected_plan, start, context, &robot_module_startup);
}
uint8_t RobotBuildModuleStarted(RobotModule_t module)
{
    if ((unsigned)module >= ROBOT_MODULE_COUNT) return 0u;
    return (robot_module_startup.started & ROBOT_MODULE_BIT(module)) != 0u;
}
uint8_t RobotBuildMarkFailure(RobotModule_t module)
{
    if ((unsigned)module >= ROBOT_MODULE_COUNT ||
        (selected_plan.enabled & ROBOT_MODULE_BIT(module)) == 0u) return 0u;
    const PlatformIrqState_t saved = PlatformIrqSaveAndDisable(); // 保留进入前的中断状态。
    const RobotModuleStatus_t result = RobotModuleStartupMarkFailure(
        &selected_plan, &robot_module_startup, module);
    PlatformIrqRestore(saved); // 保持调用前已屏蔽中断的情况。
    return (uint8_t)(result == ROBOT_MODULE_CORE_FAILURE);
}
