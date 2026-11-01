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
RobotModuleStatus_t RobotModuleStart(const RobotModulePlan_t *plan,
    RobotModuleStartFn_t start, void *context, RobotModuleStartup_t *state)
{
    if (state == NULL) return ROBOT_MODULE_ARGUMENT;
    memset(state, 0, sizeof(*state));
    state->failed_module = ROBOT_MODULE_COUNT;
    unsigned bad_module;
    RobotModuleStatus_t result = RobotModulePlanValidate(plan, &bad_module);
    if (result != ROBOT_MODULE_OK || start == NULL)
        return result == ROBOT_MODULE_OK ? ROBOT_MODULE_ARGUMENT : result;
    const uint32_t core = RobotModuleCoreClosure(plan);
    uint32_t handled = 0u;
    for (unsigned pass = 0u; pass < ROBOT_MODULE_COUNT && handled != plan->enabled; ++pass)
    {
        for (unsigned i = 0u; i < ROBOT_MODULE_COUNT; ++i)
        {
            const uint32_t bit = ROBOT_MODULE_BIT(i), dependencies = plan->dependencies[i];
            if ((plan->enabled & bit) == 0u || (handled & bit) != 0u ||
                (dependencies & ~handled) != 0u) continue;
            handled |= bit;
            if ((dependencies & (state->failed | state->blocked)) != 0u)
                state->blocked |= bit;
            else if (start(context, i) != 0u) state->started |= bit;
            else state->failed |= bit;
            if (((state->failed | state->blocked) & core) != 0u)
            {
                state->core_failed = 1u;
                state->failed_module = (uint8_t)i;
                return ROBOT_MODULE_CORE_FAILURE;
            }
        }
    }
    return ROBOT_MODULE_OK;
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
