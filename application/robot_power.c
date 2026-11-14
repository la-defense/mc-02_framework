/** @file robot_power.c
 * @brief 在安全状态检查下执行固定容量的车型电源计划。
 */
#include "robot_power.h"
#include "platform_irq.h"
#include "robot_safety.h"
#include "init_diagnostic.h"
#include "bsp_safety.h"
#include <stddef.h>
#include <string.h>

static RobotPowerRailConfig_t selected[PLATFORM_POWER_COUNT]; // 启动时复制声明，运行期间不改配置。
static uint8_t rail_ready[PLATFORM_POWER_COUNT]; // 记录命令已写入，不代表电压已测量。
static uint32_t core_modules; // 电源失败会影响的核心模块及其依赖。
static uint8_t initialized; // 本次上电只接受一次有效计划。
static uint8_t after_completed; // 后阶段最多执行一次，失败不自动重试。
static uint32_t enabled_modules; // 只允许查询车型启用的模块。
static uint32_t before_failed; // 设备前电源失败的使用者不再被独立供电。
static RobotModulePlan_t module_plan; // 保留依赖关系，用于阶段前排除已经受阻的使用者。

// 将已失败模块沿依赖关系传播到所有下游模块。
static uint32_t RobotPowerAffectedModules(uint32_t failed)
{
    for (unsigned pass = 0u; pass < ROBOT_MODULE_COUNT; ++pass)
        for (unsigned module = 0u; module < ROBOT_MODULE_COUNT; ++module)
            if ((module_plan.enabled & ROBOT_MODULE_BIT(module)) != 0u &&
                (module_plan.dependencies[module] & failed) != 0u)
                failed |= ROBOT_MODULE_BIT(module); // 下游也不需要独立供电。
    return failed;
}

// 检查并开启一个阶段；输出为因电源失败而无法启动的模块。
static uint32_t RobotPowerApply(RobotPowerStage_t stage)
{
    uint32_t failed = 0u; // 分别汇总共享电源的使用模块。
    for (unsigned rail = 0u; rail < PLATFORM_POWER_COUNT; ++rail)
    {
        if (selected[rail].stage != stage) continue; // 只处理本阶段声明的电源。
        if ((selected[rail].modules & ~before_failed) == 0u)
        {
            // 所有使用者已因前置模块失败而受阻，不需要再开启此路电源。
            continue;
        }
        const PlatformIrqState_t saved = PlatformIrqSaveAndDisable(); // 安全检查与输出写入不可被普通中断打断。
        const uint32_t faults = RobotSafetyGetFaults(); // 读取核心故障锁存。
        const uint8_t inhibited = (uint8_t)(EstopIsLatched() ||
            (faults & (ROBOT_FAULT_INIT_FAILURE | ROBOT_FAULT_RESET_LOCKOUT | ROBOT_FAULT_TASK_TIMEOUT)) != 0u);
        const uint8_t enabled = (uint8_t)(!inhibited && PlatformPowerEnable((PlatformPowerRail_t)rail)); // 被禁止时不调用输出。
        const uint8_t core_failure = (uint8_t)(!enabled && (selected[rail].modules & core_modules) != 0u);
        if (enabled) rail_ready[rail] = 1u; // 在恢复中断前发布本次输出结果。
        if (core_failure)
        {
            BSP_SafetyStartupLatchOutputsOff(); // 核心供电失败时先直接关断全部受控输出。
            RobotFaultReport(ROBOT_FAULT_INIT_FAILURE); // 锁存初始化故障。
            RobotSafetyRequestDisable(); // 撤销运行许可。
        }
        PlatformIrqRestore(saved); // 立即恢复原中断状态。
        if (enabled)
        {
            continue;
        }
        failed |= selected[rail].modules; // 记录该路影响的全部使用者。
        before_failed |= RobotPowerAffectedModules(selected[rail].modules); // 后续电源不再供给已受阻的下游。
        if (core_failure)
        {
            InitDiagnosticReportFailure(INIT_STAGE_APPLICATION, INIT_MODULE_ROBOT,
                                        INIT_FAILURE_RESOURCE, (uint16_t)(rail + 1u)); // 保存具体电源编号。
            break; // 核心电源失败后不能继续开启其他电源。
        }
        InitDiagnosticReportDegraded(INIT_STAGE_APPLICATION, INIT_MODULE_ROBOT,
                                     INIT_FAILURE_RESOURCE, (uint16_t)(rail + 1u)); // 辅助电源失败保留降级记录。
    }
    return failed;
}

uint8_t RobotPowerBegin(const RobotPowerRailConfig_t rails[PLATFORM_POWER_COUNT],
                        const RobotModulePlan_t *modules)
{
    return RobotPowerBeginWithUnavailable(rails, modules, 0u); // 兼容没有预检拒绝项的调用。
}

uint8_t RobotPowerBeginWithUnavailable(const RobotPowerRailConfig_t rails[PLATFORM_POWER_COUNT],
                                       const RobotModulePlan_t *modules, uint32_t unavailable)
{
    if (initialized || rails == NULL || RobotModulePlanValidate(modules, NULL) != ROBOT_MODULE_OK)
        return 0u; // 禁止重复执行或使用无效模块计划。
    if ((unavailable & ~modules->enabled) != 0u ||
        (unavailable & RobotModuleCoreClosure(modules)) != 0u)
        return 0u; // 非法位或核心依赖被拒绝时，不改变电源状态。
    for (unsigned rail = 0u; rail < PLATFORM_POWER_COUNT; ++rail)
    {
        const RobotPowerRailConfig_t item = rails[rail]; // 先检查全部电源，再操作任何输出。
        if (item.stage < ROBOT_POWER_OFF || item.stage > ROBOT_POWER_AFTER_DEVICES ||
            (item.modules & ~modules->enabled) != 0u ||
            ((item.stage == ROBOT_POWER_OFF) != (item.modules == 0u)))
            return 0u; // 关闭与使用者声明不一致时拒绝整个计划。
    }
    memcpy(selected, rails, sizeof(selected)); // 所有声明通过后一次保存。
    memset(rail_ready, 0, sizeof(rail_ready)); // 新计划没有已开启的电源。
    core_modules = RobotModuleCoreClosure(modules); // 保存核心及其全部前置模块。
    enabled_modules = modules->enabled; // 保存本车型启用的模块范围。
    module_plan = *modules; // 阶段回调需要相同的依赖关系。
    initialized = 1u; // 从这里开始禁止重复计划或自动重试。
    before_failed = RobotPowerAffectedModules(unavailable); // 预检失败及其下游不能独占电源。
    const uint32_t power_failed = RobotPowerApply(ROBOT_POWER_BEFORE_DEVICES); // 执行时同时传播失败依赖。
    before_failed |= power_failed; // 合并直接受电源故障影响的使用者。
    before_failed = RobotPowerAffectedModules(before_failed); // 设备前电源失败的下游也受阻。
    return (uint8_t)((before_failed & core_modules) == 0u);
}

uint32_t RobotPowerDeferredModules(void)
{
    uint32_t result = 0u; // 没有后阶段电源时不延迟模块。
    if (!initialized) return 0u;
    for (unsigned rail = 0u; rail < PLATFORM_POWER_COUNT; ++rail)
        if (selected[rail].stage == ROBOT_POWER_AFTER_DEVICES)
            result |= selected[rail].modules; // 合并所有后阶段电源的使用者。
    return result;
}

uint8_t RobotPowerAfterDevices(void *context, const RobotModuleStartup_t *prior_state,
                               uint32_t *failed_modules)
{
    (void)context; // 阶段配置已经由 RobotPowerBegin 复制。
    if (!initialized || after_completed || prior_state == NULL || failed_modules == NULL)
        return 0u; // 未准备、状态缺失或重复调用都不能执行。
    after_completed = 1u; // 标记阶段已开始，失败也不自动重试。
    before_failed |= RobotPowerAffectedModules(prior_state->failed | prior_state->blocked); // 已失败模块的下游无需供电。
    *failed_modules = RobotPowerApply(ROBOT_POWER_AFTER_DEVICES); // 返回具体受影响模块。
    return 1u;
}

uint8_t RobotPowerModuleReady(unsigned module)
{
    if (!initialized || module >= ROBOT_MODULE_COUNT) return 0u; // 未完成计划或编号无效均不可用。
    const uint32_t bit = ROBOT_MODULE_BIT(module); // 将模块编号换成依赖位。
    if ((enabled_modules & bit) == 0u) return 0u; // 未启用模块不作为可运行功能。
    if ((before_failed & bit) != 0u) return 0u; // 共享电源开启也不能让已拒绝模块就绪。
    const PlatformIrqState_t saved = PlatformIrqSaveAndDisable(); // 检查时不与普通关断中断交错。
    uint8_t ready = 1u;
    for (unsigned rail = 0u; rail < PLATFORM_POWER_COUNT; ++rail)
        if ((selected[rail].modules & bit) != 0u &&
            (!rail_ready[rail] || !PlatformPowerIsEnabled((PlatformPowerRail_t)rail)))
            ready = 0u; // 关断后不能只沿用此前的软件成功状态。
    PlatformIrqRestore(saved);
    return ready; // GPIO 锁存为高也不证明实际电压正常。
}
