/** @file robot_can_capacity.c
 * @brief 纯计算模块容量分配，先检查核心闭包再处理辅助需求。
 */
#include "robot_can_capacity.h"
#include <stddef.h>

_Static_assert(PLATFORM_CAN_BUS_2 == PLATFORM_CAN_BUS_1 + 1 &&
               PLATFORM_CAN_BUS_3 == PLATFORM_CAN_BUS_2 + 1,
               "CAN bus identifiers must match the capacity array"); // 三条逻辑总线连续编号。

RobotModuleStatus_t RobotCanCapacityPlan(const RobotModulePlan_t *plan,
    const RobotCanFilterDemand_t *demands, uint8_t count,
    const RobotCanFilterCapacity_t *capacity, RobotCanCapacityResult_t *result)
{
    if (plan == NULL || capacity == NULL || result == NULL || count > 8u ||
        (count != 0u && demands == NULL)) return ROBOT_MODULE_ARGUMENT; // 无效输入不写输出。
    const RobotModuleStatus_t valid = RobotModulePlanValidate(plan, NULL); // 容量计划依赖有效的模块图。
    if (valid != ROBOT_MODULE_OK) return valid; // 依赖不完整或有环时不发布容量结果。
    for (uint8_t i = 0u; i < count; ++i)
    {
        const RobotCanFilterDemand_t item = demands[i]; // 先检查全部需求，再分配资源。
        if ((unsigned)item.module >= ROBOT_MODULE_COUNT || item.bus < PLATFORM_CAN_BUS_1 ||
            item.bus > PLATFORM_CAN_BUS_3 || (plan->enabled & ROBOT_MODULE_BIT(item.module)) == 0u)
            return ROBOT_MODULE_ARGUMENT; // 未声明模块和无效总线不能进入计算。
    }
    RobotCanCapacityResult_t candidate = {.failed_core_module = ROBOT_MODULE_COUNT}; // 输出只在检查完成后发布。
    uint16_t used_standard[3] = {0}, used_extended[3] = {0}, used_total = 0u; // 使用宽整数避免需求累加回绕。
    const uint32_t core = RobotModuleCoreClosure(plan); // 核心依赖也优先获得资源。
    uint32_t processed = 0u; // 记录已按依赖顺序处理的模块。
    for (uint8_t pass = 0u; pass < 2u; ++pass)
      for (unsigned round = 0u; round < ROBOT_MODULE_COUNT; ++round)
        for (unsigned module = 0u; module < ROBOT_MODULE_COUNT; ++module)
        {
            const uint32_t bit = ROBOT_MODULE_BIT(module); // 每个模块的需求必须整体处理。
            if ((plan->enabled & bit) == 0u || (processed & bit) != 0u ||
                (((core & bit) != 0u) != (pass == 0u)) ||
                (plan->dependencies[module] & ~processed) != 0u) continue; // 只处理当前轮次中依赖已决的模块。
            processed |= bit; // 每个已启用模块只处理一次。
            if ((plan->dependencies[module] & candidate.rejected_modules) != 0u)
            {
                candidate.rejected_modules |= bit; // 上游辅助模块被拒绝时，不为其下游占用槽位。
                continue;
            }
            uint16_t standard[3] = {0}, extended[3] = {0}, total = 0u; // 汇总同一模块的各条总线。
            for (uint8_t i = 0u; i < count; ++i)
                if ((unsigned)demands[i].module == module)
                {
                    const unsigned bus = (unsigned)demands[i].bus - (unsigned)PLATFORM_CAN_BUS_1; // 转为数组下标。
                    standard[bus] += demands[i].standard; // 分别累加两类硬件槽。
                    extended[bus] += demands[i].extended; // 扩展 ID 使用独立槽位。
                    total += (uint16_t)demands[i].standard + demands[i].extended; // 每个过滤器对应一个登记端点。
                }
            uint8_t fits = (uint8_t)(used_total + total <= capacity->endpoints); // 先检查全局登记上限。
            for (unsigned bus = 0u; bus < 3u; ++bus)
                if (used_standard[bus] + standard[bus] > capacity->standard[bus] ||
                    used_extended[bus] + extended[bus] > capacity->extended[bus]) fits = 0u; // 再检查每类硬件槽。
            if (!fits)
            {
                if (pass == 0u) // 核心需求不能按辅助降级处理。
                {
                    candidate.failed_core_module = (uint8_t)module; // 保存无法分配的核心模块。
                    *result = candidate; // 交付可用于诊断的失败位置。
                    return ROBOT_MODULE_CORE_FAILURE; // 不继续分配辅助模块。
                }
                candidate.rejected_modules |= bit; // 拒绝整个辅助模块，不消耗任何槽位。
                continue;
            }
            used_total += total; // 模块完整通过后才占用容量。
            for (unsigned bus = 0u; bus < 3u; ++bus)
            {
                used_standard[bus] += standard[bus]; // 保留本总线标准槽用量。
                used_extended[bus] += extended[bus]; // 扩展槽独立计数。
            }
        }
    *result = candidate; // 所有核心需求均满足后一次交付结果。
    return ROBOT_MODULE_OK;
}
