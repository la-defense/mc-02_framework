/** @file module_plan.c
 * @brief 最多按模块数扫描依赖，不分配内存；同级按模块编号启动。
 */
#include "module_plan.h"
#include <stddef.h>
#include <string.h>
static uint32_t all_modules(void) { return (UINT32_C(1) << ROBOT_MODULE_COUNT)-1u; }
uint32_t RobotModuleCoreClosure(const RobotModulePlan_t *plan)
{
    if (plan == NULL) return 0u;
    uint32_t core = plan->required;
    // 多轮扩展依赖位图，直到核心模块的完整前置链都包含在内。
    for (unsigned pass = 0u; pass < ROBOT_MODULE_COUNT; ++pass)
        for (unsigned i = 0u; i < ROBOT_MODULE_COUNT; ++i)
            if ((core & ROBOT_MODULE_BIT(i)) != 0u) core |= plan->dependencies[i];
    return core;
}
RobotModuleStatus_t RobotModulePlanValidate(const RobotModulePlan_t *plan, unsigned *module)
{
    if (module != NULL) *module = ROBOT_MODULE_COUNT;
    if (plan == NULL || (plan->enabled & ~all_modules()) != 0u ||
        (plan->required & ~plan->enabled) != 0u) return ROBOT_MODULE_ARGUMENT;
    uint32_t visited = 0u;
    for (unsigned i = 0u; i < ROBOT_MODULE_COUNT; ++i)
        if ((plan->enabled & ROBOT_MODULE_BIT(i)) != 0u &&
            (plan->dependencies[i] & ~plan->enabled) != 0u)
        {
            if (module != NULL) *module = i;
            return ROBOT_MODULE_DEPENDENCY;
        }
    // 每轮加入依赖已全部访问的模块；剩余模块含有依赖环，无法排完启动顺序。
    for (unsigned pass = 0u; pass < ROBOT_MODULE_COUNT; ++pass)
        for (unsigned i = 0u; i < ROBOT_MODULE_COUNT; ++i)
            if ((plan->enabled & ROBOT_MODULE_BIT(i)) != 0u &&
                (plan->dependencies[i] & ~visited) == 0u) visited |= ROBOT_MODULE_BIT(i);
    if (visited != plan->enabled)
    {
        if (module != NULL)
            for (unsigned i = 0u; i < ROBOT_MODULE_COUNT; ++i)
                if (((plan->enabled & ~visited) & ROBOT_MODULE_BIT(i)) != 0u)
                { *module = i; break; }
        return ROBOT_MODULE_CYCLE;
    }
    return ROBOT_MODULE_OK;
}
// 在已处理的依赖之后启动指定集合；共用状态保留第一阶段结果。
static RobotModuleStatus_t RobotModuleStartSubset(const RobotModulePlan_t *plan, uint32_t eligible,
    RobotModuleStartFn_t start, void *context, RobotModuleStartup_t *state, uint32_t *handled)
{
    const uint32_t core = RobotModuleCoreClosure(plan); // 核心及其依赖失败必须停止。
    // 模块只在依赖已处理后启动；辅助依赖失败时将其下游记为阻塞。
    for (unsigned pass = 0u; pass < ROBOT_MODULE_COUNT && ((*handled & eligible) != eligible); ++pass)
    {
        for (unsigned i = 0u; i < ROBOT_MODULE_COUNT; ++i)
        {
            const uint32_t bit = ROBOT_MODULE_BIT(i), dependencies = plan->dependencies[i];
            if ((eligible & bit) == 0u || (*handled & bit) != 0u ||
                (dependencies & ~*handled) != 0u) continue; // 非本阶段或依赖尚未完成时跳过。
            *handled |= bit; // 确保每个模块最多处理一次。
            if ((dependencies & (state->failed | state->blocked)) != 0u)
                state->blocked |= bit; // 前置模块失败时不调用该模块。
            else if ((state->failed & bit) != 0u) { /* 阶段准备已经报告此模块失败。 */ }
            else if (start(context, i) != 0u) state->started |= bit; // 初始化成功后发布已启动。
            else state->failed |= bit; // 模块自身失败交由核心闭包判断。
            if (((state->failed | state->blocked) & core) != 0u)
            {
                state->core_failed = 1u;
                state->failed_module = (uint8_t)i;
                return ROBOT_MODULE_CORE_FAILURE;
            }
        }
    }
    return (*handled & eligible) == eligible ? ROBOT_MODULE_OK : ROBOT_MODULE_DEPENDENCY;
}

RobotModuleStatus_t RobotModuleStart(const RobotModulePlan_t *plan,
    RobotModuleStartFn_t start, void *context, RobotModuleStartup_t *state)
{
    if (state == NULL) return ROBOT_MODULE_ARGUMENT;
    memset(state, 0, sizeof(*state));
    state->failed_module = ROBOT_MODULE_COUNT;
    const RobotModuleStatus_t result = RobotModulePlanValidate(plan, NULL);
    if (result != ROBOT_MODULE_OK || start == NULL)
        return result == ROBOT_MODULE_OK ? ROBOT_MODULE_ARGUMENT : result;
    uint32_t handled = 0u;
    return RobotModuleStartSubset(plan, plan->enabled, start, context, state, &handled);
}

RobotModuleStatus_t RobotModuleStartPhased(const RobotModulePlan_t *plan, uint32_t deferred,
    RobotModuleBarrierFn_t barrier, RobotModuleStartFn_t start, void *context,
    RobotModuleStartup_t *state)
{
    if (state == NULL) return ROBOT_MODULE_ARGUMENT; // 无输出时不能记录任何启动结果。
    memset(state, 0, sizeof(*state)); // 新启动不沿用上次状态。
    state->failed_module = ROBOT_MODULE_COUNT; // 默认没有确定的失败模块。
    const RobotModuleStatus_t validation = RobotModulePlanValidate(plan, NULL); // 先确认依赖完整无环。
    if (validation != ROBOT_MODULE_OK) return validation;
    if (start == NULL || barrier == NULL || (deferred & ~plan->enabled) != 0u)
        return ROBOT_MODULE_ARGUMENT; // 不调用回调处理无效的启动声明。
    uint32_t delayed = deferred; // 先保存直接依赖阶段资源的模块。
    for (unsigned pass = 0u; pass < ROBOT_MODULE_COUNT; ++pass)
        for (unsigned i = 0u; i < ROBOT_MODULE_COUNT; ++i)
            if ((plan->enabled & ROBOT_MODULE_BIT(i)) != 0u &&
                (plan->dependencies[i] & delayed) != 0u)
                delayed |= ROBOT_MODULE_BIT(i); // 依赖延后模块的下游也要延后。
    uint32_t handled = 0u; // 两个阶段共用已处理集合。
    RobotModuleStatus_t result = RobotModuleStartSubset(plan, plan->enabled & ~delayed,
                                                        start, context, state, &handled);
    if (result != ROBOT_MODULE_OK) return result; // 第一阶段核心失败时不执行资源准备。
    uint32_t failed = 0u; // 回调只报告真正受资源失败影响的模块。
    if (!barrier(context, &failed) || (failed & ~deferred) != 0u)
    {
        state->core_failed = 1u; // 阶段本身异常或结果越界时保持整体失能。
        return ROBOT_MODULE_CORE_FAILURE;
    }
    state->failed |= failed; // 后续逐项处理失败模块及其下游。
    const uint32_t failed_core = failed & RobotModuleCoreClosure(plan); // 资源直接影响核心闭包时立即停止。
    if (failed_core != 0u)
    {
        for (unsigned i = 0u; i < ROBOT_MODULE_COUNT; ++i)
            if ((failed_core & ROBOT_MODULE_BIT(i)) != 0u)
            {
                state->failed_module = (uint8_t)i; // 保留实际失败的核心模块。
                break;
            }
        state->core_failed = 1u;
        return ROBOT_MODULE_CORE_FAILURE; // 不再初始化其他模块。
    }
    result = RobotModuleStartSubset(plan, delayed, start, context, state, &handled);
    return result; // 核心失败或辅助降级均保留在 state 中。
}

RobotModuleStatus_t RobotModuleStartupMarkFailure(const RobotModulePlan_t *plan,
    RobotModuleStartup_t *state, RobotModule_t module)
{
    if (plan == NULL || state == NULL || (unsigned)module >= ROBOT_MODULE_COUNT)
        return ROBOT_MODULE_ARGUMENT;
    const uint32_t bit = ROBOT_MODULE_BIT(module);
    if ((plan->enabled & bit) == 0u)
        return ROBOT_MODULE_ARGUMENT;

    state->failed |= bit;
    state->started &= ~bit;
    // 沿依赖关系传播运行期故障，避免继续调用依赖失效模块的功能。
    for (unsigned pass = 0u; pass < ROBOT_MODULE_COUNT; ++pass)
        for (unsigned index = 0u; index < ROBOT_MODULE_COUNT; ++index)
            if ((plan->enabled & ROBOT_MODULE_BIT(index)) != 0u &&
                (plan->dependencies[index] & (state->failed | state->blocked)) != 0u)
            {
                state->blocked |= ROBOT_MODULE_BIT(index);
                state->started &= ~ROBOT_MODULE_BIT(index);
            }

    const uint32_t affected = (state->failed | state->blocked) & RobotModuleCoreClosure(plan);
    if (affected != 0u)
    {
        if (state->core_failed == 0u)
            state->failed_module = (uint8_t)module;
        state->core_failed = 1u;
        return ROBOT_MODULE_CORE_FAILURE;
    }
    return ROBOT_MODULE_OK;
}
