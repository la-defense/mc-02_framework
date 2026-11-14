/** @file robot_control_policy.c
 * @brief 三种控制模式的纯C许可计算，来源期限与运行条件分别检查。
 */
#include "robot_control_policy.h"
#include <limits.h>
#include <stddef.h>

// 无数据时允许期限为0；有数据必须给出不超过半个计数范围的有效时间。
static uint8_t LeaseValid(const RobotControlLease_t *lease)
{
    return (uint8_t)(lease->valid <= 1u &&
        (!lease->valid || (lease->max_age_ms > 0u && lease->max_age_ms <= INT32_MAX)));
}

// 无符号差值同时处理自然回绕与不可信的将来时刻，读取不会改变原期限。
static uint8_t Fresh(const RobotControlLease_t *lease, uint32_t now_ms)
{
    return (uint8_t)(lease->valid && (uint32_t)(now_ms - lease->source_time_ms) <= lease->max_age_ms);
}

RobotControlResult_t RobotControlEvaluate(const RobotControlPolicy_t *policy,
                                          const RobotControlFacts_t *facts,
                                          uint32_t now_ms, RobotControlPermits_t *output)
{
    // 所有对象必须存在，错误时不接触调用者的旧输出。
    if (policy == NULL || facts == NULL || output == NULL)
        return ROBOT_CONTROL_BAD_ARGUMENT;
    // 配置必须完整且只包含已实现功能；纯观察配置允许功能位为0。
    if (policy->configured != 1u || policy->formal_autonomous > 1u || policy->commissioning > 1u ||
        (policy->enabled_functions & ~ROBOT_CONTROL_ALL) != 0u)
        return ROBOT_CONTROL_BAD_CONFIG;
    // 调试固件不能同时声明自主资格或发射功能，避免配置歧义取得额外许可。
    if (policy->commissioning && (policy->formal_autonomous ||
        (policy->enabled_functions & ROBOT_CONTROL_SHOOTER) != 0u))
        return ROBOT_CONTROL_BAD_CONFIG;
    // 在枚举选择和位操作前验证事实，四种来源期限各自检查。
    if ((unsigned)facts->mode > ROBOT_CONTROL_AUTONOMOUS ||
        ((facts->feedback_valid_functions | facts->blocked_functions) & ~ROBOT_CONTROL_ALL) != 0u ||
        !LeaseValid(&facts->manual) || !LeaseValid(&facts->aim) ||
        !LeaseValid(&facts->navigation) || !LeaseValid(&facts->match))
        return ROBOT_CONTROL_BAD_FACTS;
    // 结果先清零，任何未满足的分支都只交付禁止，不继承上周期来源。
    RobotControlPermits_t candidate = {0};
    // 核心条件与立即禁止优先于所有模式或来源授权。
    if (!facts->core_ready || facts->immediate_inhibit)
    {
        *output = candidate; // 状态有效但当前必须禁止全部功能。
        return ROBOT_CONTROL_OK;
    }
    // 限定到已启用、必需反馈有效且没有局部禁止的功能集合。
    const uint32_t available = policy->enabled_functions & facts->feedback_valid_functions & ~facts->blocked_functions;
    // 调试模式始终只接受纯人工来源；错误选择不能降级成其他有动作的模式。
    if (policy->commissioning && facts->mode != ROBOT_CONTROL_MANUAL)
    {
        *output = candidate; // 等待调用方切回合法人工模式。
        return ROBOT_CONTROL_OK;
    }
    // 正式自主的资格与比赛阶段均须有效，人工使能和遥控数据不代替这两个条件。
    if (facts->mode == ROBOT_CONTROL_AUTONOMOUS)
    {
        // 资格未启用、阶段过期或比赛未运行时保持默认禁止。
        if (policy->formal_autonomous && Fresh(&facts->match, now_ms) && facts->match_running)
        {
            candidate.functions = available; // 正式自主允许已健康的功能进入各自本地控制。
            // 有效移动命令独立接管平移；没有导航时交给本地原地自旋策略。
            if (available & ROBOT_CONTROL_CHASSIS)
                candidate.chassis_source = Fresh(&facts->navigation, now_ms) ? ROBOT_CONTROL_SOURCE_NAVIGATION : ROBOT_CONTROL_SOURCE_LOCAL;
            // 没有有效瞄准数据时交给本地三轴索敌，不保留旧视觉目标。
            if (available & ROBOT_CONTROL_GIMBAL)
                candidate.gimbal_source = Fresh(&facts->aim, now_ms) ? ROBOT_CONTROL_SOURCE_VISION : ROBOT_CONTROL_SOURCE_LOCAL;
            // 自动击发必须同时保留云台、发射功能以及有效视觉请求。
            if ((available & (ROBOT_CONTROL_GIMBAL | ROBOT_CONTROL_SHOOTER)) ==
                (ROBOT_CONTROL_GIMBAL | ROBOT_CONTROL_SHOOTER) &&
                Fresh(&facts->aim, now_ms) && facts->vision_fire_requested)
                candidate.fire_source = ROBOT_CONTROL_SOURCE_VISION;
        }
    }
    // 两种人工模式必须有当前使能请求和仍有效的已选人工输入。
    else if (facts->manual_enabled && Fresh(&facts->manual, now_ms))
    {
        candidate.functions = available; // 模式间保持使能请求，由调用方平滑接续实际目标。
        // 人工模式的底盘来源始终为人工，不受导航数据更新影响。
        if (available & ROBOT_CONTROL_CHASSIS)
            candidate.chassis_source = ROBOT_CONTROL_SOURCE_MANUAL;
        // 只有辅助模式且瞄准仍有效时让视觉取得云台来源。
        if (available & ROBOT_CONTROL_GIMBAL)
            candidate.gimbal_source = facts->mode == ROBOT_CONTROL_ASSIST && Fresh(&facts->aim, now_ms) ?
                ROBOT_CONTROL_SOURCE_VISION : ROBOT_CONTROL_SOURCE_MANUAL;
        // 两种人工模式的击发权都属于人工，视觉请求不能代替人工请求。
        if ((available & (ROBOT_CONTROL_GIMBAL | ROBOT_CONTROL_SHOOTER)) ==
            (ROBOT_CONTROL_GIMBAL | ROBOT_CONTROL_SHOOTER) && facts->manual_fire_requested)
            candidate.fire_source = ROBOT_CONTROL_SOURCE_MANUAL;
    }
    candidate.fire_allowed = candidate.fire_source != ROBOT_CONTROL_SOURCE_NONE; // 无来源时不得执行击发。
    *output = candidate; // 完整交付本轮功能与来源，消除旧输出残留。
    return ROBOT_CONTROL_OK;
}
