/** @file robot_safety.c
 * @brief 汇总故障与操作请求并选择机器人状态，供控制任务读取同一时刻的许可信息。
 * @note 状态接口可由普通任务并发调用；日志在恢复中断后提交。
 */
#include "robot_safety.h"
#include "platform_irq.h"
#include "platform_sync.h"
#include "bsp_log.h"
#include <limits.h>
#include <string.h>

static Robot_Status_e robot_state = ROBOT_INIT;
static uint32_t fault_flags = ROBOT_FAULT_NONE;
static uint8_t rc_online = 0u;
static uint8_t imu_valid = 0u;
static uint8_t calib_valid = 0u;
static uint8_t task_healthy = 1u;
static uint8_t enable_requested = 0u;
static uint8_t calib_requested = 0u;
static uint8_t estop_latched = 0u;
static uint8_t remote_chassis_permit = 0u;
static uint32_t remote_chassis_source_time_ms = 0u;
static uint32_t remote_chassis_max_age_ms = 0u;
static uint32_t chassis_inhibit_generation = 0u;
static Estop_Reason_e estop_reason = ESTOP_REASON_NONE;
static uint8_t imu_fault_logged = 0u;
static uint8_t task_fault_logged = 0u;
static uint8_t control_configured; // 旧固件未配置时继续使用原控制规则。
static uint8_t control_disabled; // 人工显式失能后，正式自主也必须等待人工重新使能。
static uint8_t control_was_healthy; // 识别核心条件恢复，要求恢复后新来源。
static uint8_t control_barrier_valid; // 区分尚未发生禁止和禁止发生于0ms。
static uint32_t control_barrier_ms; // 来源时间必须严格晚于最近恢复／禁止检查时刻。
static uint32_t control_generation; // 即使故障在两个周期之间恢复，也留下变化记录。
static uint8_t control_fire_was_allowed; // 记录上次计算是否允许击发，用于识别短暂撤销。
static uint8_t control_fire_wait; // 击发撤销后要求新的对应来源。
static uint32_t control_fire_barrier_ms; // 保存最近一次击发撤销的本机时间。
static uint32_t control_local_wait; // 发生过局部禁止、恢复后需新来源的功能位。
static uint32_t control_local_barrier[3]; // 底盘／云台／发射各自恢复时刻，避免局部故障中断其他功能。
static uint32_t control_feedback_seen; // 记录已接受原始反馈时间的功能位。
static RobotControlLease_t control_feedback_last[3]; // 保留最后接受的时间与期限，防止重复样本续期。
static uint8_t control_vehicle_feedback; // 绑定后只有整车采集者可以更新反馈。
static RobotControlPolicy_t control_policy;
static RobotSafetyControlInputs_t control_inputs;

// 调用前已进入安全状态临界区；无论由驱动发现还是到期，都记录同一类反馈失效。
static void ControlRevokeFeedbackLocked(uint32_t functions, uint32_t now_ms)
{
    const uint32_t lost = functions & control_inputs.feedback_valid_functions &
        control_policy.enabled_functions & ~control_inputs.blocked_functions; // 只记录原先反馈有效且未被单独禁止的功能。
    control_inputs.feedback_valid_functions &= ~functions; // 清除有效位后，查询不会继续使用旧反馈。
    if (lost == 0u) return; // 重复报告不推进代次，也不延长等待。
    control_local_wait |= lost; // 反馈恢复后仍需该功能的新控制输入。
    for (unsigned i = 0u; i < 3u; ++i)
        if (lost & (1u << i)) control_local_barrier[i] = now_ms; // 各功能保存实际发现失效的时刻。
    control_generation++; // 所有等待发送的旧代次目标都失效。
}

// 查询时也检查期限，覆盖驱动没有收到任何新数据的情况。
static void ControlExpireFeedbackLocked(uint32_t now_ms)
{
    uint32_t expired = 0u; // 收集所有已过期的功能，统一撤销一次。
    for (unsigned i = 0u; i < 3u; ++i)
    {
        const uint32_t bit = 1u << i; // 每组反馈对应一个功能位。
        if ((control_inputs.feedback_valid_functions & bit) == 0u) continue; // 已无有效反馈时不重复记录。
        const RobotControlLease_t *lease = &control_inputs.feedback[i]; // 检查对应机构自己的期限。
        if (lease->valid && lease->max_age_ms > 0u && lease->max_age_ms <= INT32_MAX &&
            (uint32_t)(now_ms - lease->source_time_ms) <= lease->max_age_ms) continue; // 恰好达到期限仍有效。
        expired |= bit; // 过期或非法的反馈不能保留有效位。
    }
    if (expired) ControlRevokeFeedbackLocked(expired, now_ms); // 保留实际发现过期的时刻。
}

// 同一原始时间不能通过加长期限续期；时间回绕时只接受半区间内向前的样本。
static uint8_t ControlFeedbackValidLocked(const RobotSafetyControlInputs_t *inputs, uint32_t now_ms)
{
    for (unsigned i = 0u; i < 3u; ++i)
    {
        const uint32_t bit = 1u << i; // 逐组校验声明有效的反馈。
        if ((inputs->feedback_valid_functions & bit) == 0u) continue; // 未声明有效的组不要求时间。
        const RobotControlLease_t *lease = &inputs->feedback[i]; // 期限来自该组最早失效的必需数据。
        if (!lease->valid || lease->max_age_ms == 0u || lease->max_age_ms > INT32_MAX ||
            (uint32_t)(now_ms - lease->source_time_ms) > lease->max_age_ms) return 0u; // 无效、将来或过期均拒绝。
        if (control_feedback_seen & bit)
        {
            const uint32_t advance = lease->source_time_ms - control_feedback_last[i].source_time_ms; // 比较原始时刻而非提交时刻。
            if (advance > INT32_MAX || (advance == 0u && lease->max_age_ms > control_feedback_last[i].max_age_ms))
                return 0u; // 不接受旧样本，也不允许同一份样本延长期限。
        }
    }
    return 1u; // 所有声明有效的组均通过后才允许提交。
}

// 调用前已进入安全状态临界区；资格不等于已通过比赛阶段检查。
static uint8_t ControlFormalLocked(void)
{
    return (uint8_t)(control_configured && control_policy.formal_autonomous &&
        !control_policy.commissioning && control_inputs.mode == ROBOT_CONTROL_AUTONOMOUS);
}

// 保存禁止事件；模式策略启用时还要求来源在恢复后重新接管。
static void ControlRevokeLocked(void)
{
    control_generation++; // 记录周期之间也不能漏掉的禁止变化。
    if (!control_configured) return; // 旧角色也记录代际，但不启用新来源屏障。
    control_barrier_ms = PlatformTimeNowMs(); // 保存实际禁止时刻，不替来源续期。
    control_barrier_valid = 1u; // 0ms同样需要阻止先前数据。
    control_was_healthy = 0u; // 恢复时再次建立接管起点。
}

// 将管理者保存的输入组装成策略事实；核心和使能不接受调用者自行声明。
static RobotControlFacts_t ControlFactsLocked(const RobotSafetyControlInputs_t *inputs)
{
    RobotControlFacts_t facts = {0}; // 未明确赋值的请求保持禁止。
    facts.mode = inputs->mode; // 模式由唯一命令仲裁者选择。
    facts.manual = inputs->manual; // 复制已选人工来源的原始期限。
    facts.aim = inputs->aim; // 瞄准单独保留期限。
    facts.navigation = inputs->navigation; // 导航不能替瞄准续期。
    facts.match = inputs->match; // 比赛阶段独立检查。
    facts.feedback_valid_functions = inputs->feedback_valid_functions; // 模块反馈有效位由来源模块报告。
    facts.blocked_functions = inputs->blocked_functions; // 局部禁止不能覆盖全局禁止。
    facts.match_running = inputs->match_running; // 仍需策略检查阶段是否过期。
    facts.manual_fire_requested = inputs->manual_fire_requested; // 只保存本轮人工请求。
    facts.vision_fire_requested = inputs->vision_fire_requested; // 辅助模式不会因此取得自动击发。
    uint32_t faults = fault_flags; // 从真实安全状态读取故障，调用方不能自行清除。
    // 正式自主允许无遥控，但仍保留RC离线诊断，其他模式继续要求遥控在线。
    const uint8_t autonomous = (uint8_t)(control_policy.formal_autonomous &&
        !control_policy.commissioning && inputs->mode == ROBOT_CONTROL_AUTONOMOUS);
    if (autonomous) faults &= ~(uint32_t)ROBOT_FAULT_RC_OFFLINE; // 只豁免该正式模式不依赖的遥控故障。
    facts.core_ready = (uint8_t)(faults == 0u && imu_valid && calib_valid && task_healthy &&
        !calib_requested && (autonomous || rc_online)); // 保留全部核心反馈和人工模式的遥控条件。
    facts.immediate_inhibit = (uint8_t)(estop_latched || control_disabled || calib_requested); // 急停和显式失能不可由模式绕过。
    facts.manual_enabled = enable_requested; // 人工使能只由原安全请求接口修改。
    return facts;
}

// 来源必须严格晚于对应恢复时刻；计数自然回绕时仍只接受向前半区间。
static uint8_t ControlSourceAfter(const RobotControlLease_t *lease, uint32_t barrier)
{
    const uint32_t advance = lease->source_time_ms - barrier; // 使用原始时间，不用发布时刻。
    return (uint8_t)(lease->valid && advance != 0u && advance <= INT32_MAX);
}

// 对局部故障分别处理接管，底盘和云台可以继续使用各自健康的来源。
static void ControlFilterLocalLocked(const RobotControlFacts_t *facts, RobotControlPermits_t *permits)
{
    if ((control_local_wait & ROBOT_CONTROL_CHASSIS) && (permits->functions & ROBOT_CONTROL_CHASSIS))
    {
        const RobotControlLease_t *lease = permits->chassis_source == ROBOT_CONTROL_SOURCE_NAVIGATION ? &facts->navigation : &facts->manual; // 按实际来源选原始时间。
        if (permits->chassis_source != ROBOT_CONTROL_SOURCE_LOCAL && !ControlSourceAfter(lease, control_local_barrier[0]))
        {
            if (facts->mode == ROBOT_CONTROL_AUTONOMOUS) permits->chassis_source = ROBOT_CONTROL_SOURCE_LOCAL; // 正式自主只恢复本地自旋，不重放旧移动。
            else permits->functions &= ~(uint32_t)ROBOT_CONTROL_CHASSIS; // 人工模式等待恢复后新输入。
        }
    }
    if ((control_local_wait & ROBOT_CONTROL_GIMBAL) && (permits->functions & ROBOT_CONTROL_GIMBAL))
    {
        const RobotControlLease_t *lease = permits->gimbal_source == ROBOT_CONTROL_SOURCE_VISION ? &facts->aim : &facts->manual; // 云台可由人工或视觉接管。
        if (permits->gimbal_source != ROBOT_CONTROL_SOURCE_LOCAL && !ControlSourceAfter(lease, control_local_barrier[1]))
        {
            if (facts->mode == ROBOT_CONTROL_AUTONOMOUS) permits->gimbal_source = ROBOT_CONTROL_SOURCE_LOCAL; // 先以本地索敌等待新瞄准目标。
            else if (ControlSourceAfter(&facts->manual, control_local_barrier[1])) permits->gimbal_source = ROBOT_CONTROL_SOURCE_MANUAL; // 辅助模式可使用恢复后的新人工输入。
            else permits->functions &= ~(uint32_t)ROBOT_CONTROL_GIMBAL; // 两个来源都旧时禁止云台。
        }
    }
    if ((control_local_wait & ROBOT_CONTROL_SHOOTER) && (permits->functions & ROBOT_CONTROL_SHOOTER))
    {
        const RobotControlLease_t *lease = facts->mode == ROBOT_CONTROL_AUTONOMOUS ? &facts->aim : &facts->manual; // 自动击发跟随瞄准，人工击发跟随人工来源。
        if (!ControlSourceAfter(lease, control_local_barrier[2]))
        {
            permits->fire_source = ROBOT_CONTROL_SOURCE_NONE; // 无论何种模式都不能重放恢复前的击发请求。
            if (facts->mode != ROBOT_CONTROL_AUTONOMOUS) permits->functions &= ~(uint32_t)ROBOT_CONTROL_SHOOTER; // 人工机构也等待新来源才恢复。
        }
    }
    if (!(permits->functions & ROBOT_CONTROL_CHASSIS)) permits->chassis_source = ROBOT_CONTROL_SOURCE_NONE; // 被撤销功能不残留来源名称。
    if (!(permits->functions & ROBOT_CONTROL_GIMBAL)) permits->gimbal_source = ROBOT_CONTROL_SOURCE_NONE; // 云台停止时同步撤销来源。
    if ((permits->functions & (ROBOT_CONTROL_GIMBAL | ROBOT_CONTROL_SHOOTER)) !=
        (ROBOT_CONTROL_GIMBAL | ROBOT_CONTROL_SHOOTER)) permits->fire_source = ROBOT_CONTROL_SOURCE_NONE; // 任一必要功能失效均禁止击发。
    if (permits->gimbal_source == ROBOT_CONTROL_SOURCE_LOCAL) permits->fire_source = ROBOT_CONTROL_SOURCE_NONE; // 本地重捕获期间不执行旧视觉击发。
    permits->fire_allowed = permits->fire_source != ROBOT_CONTROL_SOURCE_NONE; // 最后统一更新击发结果。
}

// 故障恢复后的同一轮旧来源不能重新授权；裁判阶段仅表达比赛事实，不作为运动目标。
static uint8_t ControlEvaluateLocked(uint32_t now_ms, RobotControlPermits_t *output)
{
    if (!control_configured) return 0u; // 未配置时不使用新的许可接口。
    const uint32_t actual_ms = PlatformTimeNowMs(); // 调用者取时后可能已有新反馈被发布。
    if ((uint32_t)(actual_ms - now_ms) <= INT32_MAX) now_ms = actual_ms; // 使用较晚时刻，避免把新样本误判为将来数据。
    ControlExpireFeedbackLocked(now_ms); // 即使发布任务停止，查询也要撤销过期反馈。
    RobotControlFacts_t facts = ControlFactsLocked(&control_inputs); // 同一临界区读取核心与来源。
    const uint8_t healthy = (uint8_t)(facts.core_ready && !facts.immediate_inhibit); // 核心恢复与业务来源恢复分别处理。
    if (healthy && !control_was_healthy)
    {
        control_barrier_ms = now_ms; // 只在恢复边沿建立新起点，不能每周期续期。
        control_barrier_valid = 1u; // 后续只接收严格晚于这个起点的目标。
    }
    control_was_healthy = healthy; // 保存本轮核心状态，识别下一次变化。
    if (control_barrier_valid)
    {
        RobotControlLease_t *leases[] = {&facts.manual, &facts.aim, &facts.navigation}; // 三类控制来源都不得复用恢复前目标。
        for (unsigned i = 0u; i < 3u; ++i)
        {
            const uint32_t advance = leases[i]->source_time_ms - control_barrier_ms; // 无符号差值允许自然回绕。
            if (advance == 0u || advance > INT32_MAX) leases[i]->valid = 0u; // 起点之前或同刻的数据不能重接管。
        }
    }
    if (RobotControlEvaluate(&control_policy, &facts, now_ms, output) != ROBOT_CONTROL_OK) return 0u; // 错误时不交付许可。
    ControlFilterLocalLocked(&facts, output); // 最后复核各功能自己的恢复起点。
    if (control_fire_was_allowed && !output->fire_allowed)
    {
        control_fire_barrier_ms = now_ms; // 记录击发许可从有到无的时刻。
        control_fire_wait = 1u; // 恢复时不能重放此前的击发来源。
        control_generation++; // 即使在发射任务两次运行之间恢复，旧命令也会失效。
    }
    if (control_fire_wait && output->fire_allowed)
    {
        const RobotControlLease_t *fire_lease = output->fire_source == ROBOT_CONTROL_SOURCE_VISION ? &facts.aim : &facts.manual; // 按实际击发来源检查原始时间。
        if (!ControlSourceAfter(fire_lease, control_fire_barrier_ms))
        {
            output->fire_source = ROBOT_CONTROL_SOURCE_NONE; // 恢复前的来源不能重新获得击发权。
            output->fire_allowed = 0u; // 其他功能及摩擦轮许可不随之撤销。
        }
    }
    control_fire_was_allowed = output->fire_allowed; // 保存过滤后的许可，避免重复记录同一次撤销。
    return 1u;
}

static uint32_t RobotSafetyEnterCritical(void)
{
    // 返回进入前的屏蔽状态，退出时须原样恢复。
    return PlatformIrqSaveAndDisable();
}

static void RobotSafetyExitCritical(uint32_t previous)
{
    PlatformIrqRestore(previous); // 保留调用前已经屏蔽中断的状态。
}

uint8_t RobotSafetyConfigureControl(const RobotControlPolicy_t *policy)
{
    if (policy == NULL) return 0u; // 配置缺失不能建立新许可。
#if defined(CHASSIS_BOARD)
    return 0u; // 底盘板继续使用已有的本地条件与远端租约，不重复选择三模式。
#else
    const RobotControlFacts_t empty = {0}; // 配置检查不提供任何运行条件。
    RobotControlPermits_t unused;
    if (RobotControlEvaluate(policy, &empty, 0u, &unused) != ROBOT_CONTROL_OK) return 0u; // 复用同一策略配置检查。
    const uint32_t previous = RobotSafetyEnterCritical(); // 与所有核心状态变更使用同一保护。
    const uint8_t allowed = (uint8_t)(!control_configured && robot_state == ROBOT_SAFE &&
        !enable_requested && !calib_requested && !estop_latched); // 启动前只允许配置一次。
    if (allowed)
    {
        control_policy = *policy; // 复制只读车型策略，调用者后续改内存不会影响运行。
        memset(&control_inputs, 0, sizeof(control_inputs)); // 初始来源全部无效。
        control_inputs.mode = policy->formal_autonomous ? ROBOT_CONTROL_AUTONOMOUS : ROBOT_CONTROL_MANUAL; // 正式自主仍等待有效比赛阶段。
        control_disabled = 0u; // 人工模式仍必须另有enable_requested，正式自主使用车型资格。
        control_configured = 1u; // 所有存储准备好后才发布已配置标记。
        ControlRevokeLocked(); // 首次启用也要求新的来源数据。
    }
    RobotSafetyExitCritical(previous); // 恢复进入前的中断状态。
    return allowed;
#endif
}

// 持有安全临界区时提交已合并的来源和反馈，统一记录每个功能的失效与恢复。
static uint8_t ControlPublishLocked(const RobotSafetyControlInputs_t *inputs, uint32_t now_ms)
{
    const RobotControlFacts_t facts = ControlFactsLocked(inputs); // 核心和使能始终由管理者提供。
    RobotControlPermits_t unused; // 提前用同一策略检查本次提交的格式。
    if (!ControlFeedbackValidLocked(inputs, now_ms) ||
        RobotControlEvaluate(&control_policy, &facts, now_ms, &unused) != ROBOT_CONTROL_OK)
        return 0u; // 非法期限、位号或模式不修改现有状态。
    const uint32_t old_available = control_policy.enabled_functions & control_inputs.feedback_valid_functions & ~control_inputs.blocked_functions; // 记录原来可用的功能。
    const uint32_t new_available = control_policy.enabled_functions & inputs->feedback_valid_functions & ~inputs->blocked_functions; // 记录本次可用的功能。
    const uint32_t lost = old_available & ~new_available; // 找出刚失去反馈或被禁止的功能。
    const uint32_t changed = lost | (new_available & ~old_available & control_local_wait); // 首次收到反馈不等于从故障恢复。
    control_local_wait |= lost; // 发生过局部禁止的功能须等待恢复后的新来源。
    for (unsigned i = 0u; i < 3u; ++i)
    {
        const uint32_t bit = 1u << i; // 按固定顺序检查每个功能。
        if (changed & bit) control_local_barrier[i] = now_ms; // 记录禁止和恢复各自的时刻。
        if (inputs->feedback_valid_functions & bit)
        {
            control_feedback_seen |= bit; // 已取得可信原始时间。
            control_feedback_last[i] = inputs->feedback[i]; // 记录原始时间与期限供后续比较。
        }
    }
    if (lost) control_generation++; // 排队的旧代次目标不能跨过这次失效。
    control_inputs = *inputs; // 完整交付同一轮数据。
    (void)ControlEvaluateLocked(now_ms, &unused); // 立即记录功能和来源许可的变化。
    return 1u;
}

uint8_t RobotSafetyBindVehicleFeedback(void)
{
#if defined(CHASSIS_BOARD)
    return 0u; // 双板底盘继续由远端许可与本地检查配合。
#else
    const uint32_t previous = RobotSafetyEnterCritical(); // 绑定和准备使能不能交错。
    const uint8_t allowed = (uint8_t)(!control_vehicle_feedback && !control_configured &&
        robot_state == ROBOT_SAFE && !enable_requested && !calib_requested && !estop_latched);
    if (allowed) control_vehicle_feedback = 1u; // 启动后固定反馈的提交入口。
    RobotSafetyExitCritical(previous); // 恢复进入前的中断状态。
    return allowed;
#endif
}

uint8_t RobotSafetyPublishVehicleFeedback(uint32_t valid_functions, const RobotControlLease_t feedback[3])
{
    const uint32_t previous = RobotSafetyEnterCritical(); // 不允许命令提交覆盖本次反馈更新。
    uint8_t accepted = 0u; // 等待来源策略配置通过，保持禁止。
    if (control_configured && control_vehicle_feedback)
    {
        const uint32_t now_ms = PlatformTimeNowMs(); // 先记录两次提交间已经发生的过期。
        RobotControlPermits_t unused; // 查询只用于处理过期和禁止边沿。
        (void)ControlEvaluateLocked(now_ms, &unused);
        RobotSafetyControlInputs_t candidate = control_inputs; // 保留选择好的模式和来源。
        if (feedback != NULL && (valid_functions & ~ROBOT_CONTROL_ALL) == 0u)
        {
            candidate.feedback_valid_functions = valid_functions; // 只提交采集者已验证的反馈位。
            memcpy(candidate.feedback, feedback, sizeof(candidate.feedback)); // 带上各来源最早到期时间。
            accepted = ControlPublishLocked(&candidate, now_ms); // 同一规则处理失效和恢复。
        }
        if (!accepted)
        {
            candidate.feedback_valid_functions = 0u; // 汇总结果错误或缺少数据时撤销全部反馈。
            memset(candidate.feedback, 0, sizeof(candidate.feedback)); // 清除相应期限。
            (void)ControlPublishLocked(&candidate, now_ms); // 仍保留已取得的来源事实和禁止记录。
        }
    }
    RobotSafetyExitCritical(previous); // 发布结束后恢复进入前的屏蔽状态。
    return accepted;
}

void RobotSafetyRevokeVehicleFeedback(uint32_t functions)
{
    const uint32_t previous = RobotSafetyEnterCritical(); // 禁止插入新的汇总或发送许可查询。
    if (control_configured && control_vehicle_feedback && (functions & ~ROBOT_CONTROL_ALL) == 0u)
        ControlRevokeFeedbackLocked(functions, PlatformTimeNowMs()); // 驱动只能撤销，恢复仍由整车采集入口提交。
    RobotSafetyExitCritical(previous); // 在已有保护内调用时保留原中断状态。
}

uint8_t RobotSafetyPublishControlInputs(const RobotSafetyControlInputs_t *inputs)
{
    const uint32_t previous = RobotSafetyEnterCritical(); // 来源和反馈按同一状态发布。
    uint8_t accepted = 0u; // 尚未配置时不接收来源提交。
    if (control_configured)
    {
        const uint32_t now_ms = PlatformTimeNowMs(); // 此时刻同时用于处理过期和本次提交。
        RobotControlPermits_t unused; // 新提交之前检查旧许可是否已经撤销。
        (void)ControlEvaluateLocked(now_ms, &unused);
        RobotSafetyControlInputs_t candidate = {0}; // 无效输入最终清除全部来源。
        if (inputs != NULL)
        {
            candidate = *inputs; // 读取本轮命令仲裁结果。
            if (control_vehicle_feedback)
            {
                candidate.feedback_valid_functions = control_inputs.feedback_valid_functions; // 命令不能授予自己反馈许可。
                memcpy(candidate.feedback, control_inputs.feedback, sizeof(candidate.feedback)); // 反馈仍沿用采集者提供的期限。
            }
            accepted = ControlPublishLocked(&candidate, now_ms); // 同时检查期限、模式与枚举。
        }
        if (!accepted)
        {
            memset(&candidate, 0, sizeof(candidate)); // 撤销旧来源，保持禁止。
            candidate.mode = control_inputs.mode; // 无效提交不改变当前模式。
            if (control_vehicle_feedback)
            {
                candidate.feedback_valid_functions = control_inputs.feedback_valid_functions; // 独立有效性由采集者维护。
                memcpy(candidate.feedback, control_inputs.feedback, sizeof(candidate.feedback)); // 保留其原始期限。
            }
            control_inputs = candidate; // 不再保留旧命令及请求。
            ControlRevokeLocked(); // 下一次接管仍要用新的来源。
        }
    }
    RobotSafetyExitCritical(previous); // 恢复调用前的屏蔽状态。
    return accepted;
}

uint8_t RobotSafetyGetFunctionPermitsAt(uint32_t now_ms, RobotControlPermits_t *output)
{
    if (output == NULL) return 0u; // 参数错误时不触碰输出。
    const uint32_t previous = RobotSafetyEnterCritical(); // 保证来源、核心事实和状态来自同一时刻。
    RobotControlPermits_t candidate = {0}; // 默认禁止全部功能。
    const uint8_t valid = ControlEvaluateLocked(now_ms, &candidate); // 即使状态仍READY也复查当前过期和立即禁止。
    if (robot_state != ROBOT_READY) memset(&candidate, 0, sizeof(candidate)); // 只在状态所有者放行后允许动作。
    if (valid) *output = candidate; // 计算成功才完整交付。
    RobotSafetyExitCritical(previous); // 不延长任何来源的有效时间。
    return valid;
}

uint8_t RobotSafetyIsFormalAutonomous(void)
{
    const uint32_t previous = RobotSafetyEnterCritical(); // 和模式来源提交同步读取。
    const uint8_t selected = ControlFormalLocked(); // 只查询资格选择，不授予动作许可。
    RobotSafetyExitCritical(previous);
    return selected;
}

#if defined(CHASSIS_BOARD)
static uint8_t RobotSafetyChassisLocalReadyLocked(void)
{
    return (uint8_t)(((fault_flags & (uint32_t)~ROBOT_FAULT_RC_OFFLINE) == ROBOT_FAULT_NONE &&
                      imu_valid != 0u && calib_valid != 0u && task_healthy != 0u &&
                      calib_requested == 0u && estop_latched == 0u) ? 1u : 0u);
}

static uint8_t RobotSafetyRemoteLeaseFreshLocked(uint32_t now_ms)
{
    return (uint8_t)(remote_chassis_permit != 0u && remote_chassis_max_age_ms != 0u &&
                     (uint32_t)(now_ms - remote_chassis_source_time_ms) <=
                         remote_chassis_max_age_ms);
}

static void RobotSafetyClearRemoteChassisPermitLocked(uint8_t advance_generation)
{
    if (remote_chassis_permit != 0u && advance_generation != 0u)
        chassis_inhibit_generation++;
    remote_chassis_permit = 0u;
    remote_chassis_source_time_ms = 0u;
    remote_chassis_max_age_ms = 0u;
}
#endif

static void RobotSafetyLogStateChange(Robot_Status_e old_state,
                                      Robot_Status_e new_state)
{
    if (old_state != new_state)
        BSP_LOG_EVENT_2(EV_MODULES_SAFETY_ROBOT_SAFETY_C_01,
            BSP_LOG_FIELD_CSTR_LIMIT((RobotSafetyStateName(old_state)), 46u),
            BSP_LOG_FIELD_CSTR_LIMIT((RobotSafetyStateName(new_state)), 46u));
}

static void RobotSafetyReportFaultLocked(Robot_Fault_e fault)
{
    // 正式自主不依赖遥控在线；其他新禁止位即使短暂出现也留下接管变化。
    const uint32_t relevant = ControlFormalLocked() ? (uint32_t)fault & ~(uint32_t)ROBOT_FAULT_RC_OFFLINE : (uint32_t)fault;
    if ((fault_flags & relevant) != relevant) ControlRevokeLocked(); // 已存在的故障不反复延长恢复起点。
#if defined(CHASSIS_BOARD)
    const uint32_t inhibiting_faults = (uint32_t)fault & (uint32_t)~ROBOT_FAULT_RC_OFFLINE;
    if (inhibiting_faults != 0u)
    {
        if ((fault_flags & inhibiting_faults) != inhibiting_faults)
            chassis_inhibit_generation++;
        RobotSafetyClearRemoteChassisPermitLocked(0u);
    }
#endif
    fault_flags |= (uint32_t)fault;
}

static void RobotSafetyClearFaultLocked(Robot_Fault_e fault)
{
    const uint32_t permanent_faults = ROBOT_FAULT_INIT_FAILURE | ROBOT_FAULT_RESET_LOCKOUT;
    fault = (Robot_Fault_e)((uint32_t)fault & ~permanent_faults);
    fault_flags &= ~(uint32_t)fault;
}

static uint8_t RobotSafetyRequestEstopLocked(Estop_Reason_e reason)
{
    if (estop_latched && estop_reason == reason)
        return 0u;

    estop_latched = 1u;
    ControlRevokeLocked(); // 已排队的旧来源在急停明确解除后也不能重新接管。
    estop_reason = reason;
    enable_requested = 0u;
#if defined(CHASSIS_BOARD)
    RobotSafetyClearRemoteChassisPermitLocked(0u);
    chassis_inhibit_generation++;
#endif
    return 1u;
}

void RobotSafetyInit(void)
{
    Robot_Status_e old_state;
    const uint32_t previous = RobotSafetyEnterCritical();
    old_state = robot_state;
    robot_state = ROBOT_INIT;
    fault_flags = ROBOT_FAULT_NONE;
    rc_online = 0u;
    imu_valid = 0u;
    calib_valid = 0u;
    task_healthy = 1u;
    enable_requested = 0u;
    calib_requested = 0u;
    estop_latched = 0u;
    remote_chassis_permit = 0u;
    remote_chassis_source_time_ms = 0u;
    remote_chassis_max_age_ms = 0u;
    chassis_inhibit_generation = 0u;
    estop_reason = ESTOP_REASON_NONE;
    imu_fault_logged = 0u;
    task_fault_logged = 0u;
    control_vehicle_feedback = 0u; // 重新启动后重新绑定反馈的提交入口。
    control_configured = 0u; // 系统重新初始化后须由车型重新配置模式策略。
    control_disabled = 0u; // 尚未配置的旧路径不受新策略影响。
    control_was_healthy = 0u; // 新一次启动重新建立核心恢复边沿。
    control_barrier_valid = 0u; // 尚未发生新启动周期中的接管检查。
    control_barrier_ms = 0u; // 初始数值不作为有效来源时间。
    control_generation = 0u; // 该计数只描述当前启动周期。
    control_fire_was_allowed = 0u; // 不沿用上次启动的击发许可。
    control_fire_wait = 0u; // 新启动由全局接管规则处理旧来源。
    control_fire_barrier_ms = 0u; // 没有本周期击发撤销时不使用此时间。
    control_local_wait = 0u; // 新启动没有沿用上个周期的局部恢复记录。
    control_feedback_seen = 0u; // 新启动不沿用前次反馈时间。
    memset(control_feedback_last, 0, sizeof(control_feedback_last)); // 清除前次反馈及其期限。
    memset(control_local_barrier, 0, sizeof(control_local_barrier)); // 三种功能各自重新建立恢复时间。
    memset(&control_policy, 0, sizeof(control_policy)); // 清掉上个启动的车型策略副本。
    memset(&control_inputs, 0, sizeof(control_inputs)); // 不保存重启前的目标和击发请求。
    robot_state = ROBOT_SAFE;
    RobotSafetyExitCritical(previous);

    RobotSafetyLogStateChange(old_state, ROBOT_SAFE);
}

void RobotSafetySetRcOnline(uint8_t online)
{
    const uint32_t previous = RobotSafetyEnterCritical();
    rc_online = online != 0u ? 1u : 0u;
    if (rc_online != 0u)
        RobotSafetyClearFaultLocked(ROBOT_FAULT_RC_OFFLINE);
    else
        RobotSafetyReportFaultLocked(ROBOT_FAULT_RC_OFFLINE);
    RobotSafetyExitCritical(previous);
}

void RobotSafetySetImuValid(uint8_t valid)
{
    uint8_t log_fault = 0u;
    const uint32_t previous = RobotSafetyEnterCritical();
    imu_valid = valid != 0u ? 1u : 0u;
    if (imu_valid != 0u)
    {
        RobotSafetyClearFaultLocked(ROBOT_FAULT_IMU_INVALID);
        imu_fault_logged = 0u;
    }
    else
    {
        RobotSafetyReportFaultLocked(ROBOT_FAULT_IMU_INVALID);
        if (imu_fault_logged == 0u)
        {
            imu_fault_logged = 1u;
            log_fault = 1u;
        }
    }
    RobotSafetyExitCritical(previous);

    if (log_fault != 0u)
        BSP_LOG_EVENT_0(EV_MODULES_SAFETY_ROBOT_SAFETY_C_02);
}

void RobotSafetySetCalibValid(uint8_t valid)
{
    const uint32_t previous = RobotSafetyEnterCritical();
    calib_valid = valid != 0u ? 1u : 0u;
    if (calib_valid != 0u)
        RobotSafetyClearFaultLocked(ROBOT_FAULT_CALIB_INVALID);
    else
        RobotSafetyReportFaultLocked(ROBOT_FAULT_CALIB_INVALID);
    RobotSafetyExitCritical(previous);
}

void RobotSafetySetTaskHealthy(uint8_t healthy)
{
    uint8_t log_fault = 0u;
    const uint32_t previous = RobotSafetyEnterCritical();
    task_healthy = healthy != 0u ? 1u : 0u;
    if (task_healthy != 0u)
    {
        RobotSafetyClearFaultLocked(ROBOT_FAULT_TASK_TIMEOUT);
        task_fault_logged = 0u;
    }
    else
    {
        RobotSafetyReportFaultLocked(ROBOT_FAULT_TASK_TIMEOUT);
        if (task_fault_logged == 0u)
        {
            task_fault_logged = 1u;
            log_fault = 1u;
        }
    }
    RobotSafetyExitCritical(previous);

    if (log_fault != 0u)
        BSP_LOG_EVENT_0(EV_MODULES_SAFETY_ROBOT_SAFETY_C_03);
}

void RobotSafetyRequestEnable(void)
{
    const uint32_t previous = RobotSafetyEnterCritical();
    if ((fault_flags & (ROBOT_FAULT_INIT_FAILURE | ROBOT_FAULT_RESET_LOCKOUT)) == 0u)
    {
        if (!enable_requested || control_disabled) ControlRevokeLocked(); // 重新使能要求新输入，保持使能时不反复撤销。
        control_disabled = 0u; // 显式人工使能可以解除此前显式失能。
        enable_requested = 1u;
        calib_requested = 0u;
    }
    RobotSafetyExitCritical(previous);
}

void RobotSafetyRequestDisable(void)
{
    const uint32_t previous = RobotSafetyEnterCritical();
    control_disabled = 1u; // 正式自主也不能绕过人工明确失能。
    ControlRevokeLocked(); // 撤销旧来源，之后必须重新取得明确使能。
    enable_requested = 0u;
    calib_requested = 0u;
#if defined(CHASSIS_BOARD)
    RobotSafetyClearRemoteChassisPermitLocked(0u);
    chassis_inhibit_generation++;
#endif
    RobotSafetyExitCritical(previous);
}

void RobotSafetyRequestCalib(uint8_t enable)
{
    const uint32_t previous = RobotSafetyEnterCritical();
    if (!enable || (fault_flags & (ROBOT_FAULT_INIT_FAILURE | ROBOT_FAULT_RESET_LOCKOUT)) == 0u)
    {
        calib_requested = enable != 0u ? 1u : 0u;
        if (calib_requested != 0u)
        {
            control_disabled = 1u; // 维护标定结束后等待人工重新使能。
            ControlRevokeLocked(); // 标定前旧目标不能在维护后重放。
            enable_requested = 0u;
#if defined(CHASSIS_BOARD)
            RobotSafetyClearRemoteChassisPermitLocked(0u);
            chassis_inhibit_generation++;
#endif
        }
    }
    RobotSafetyExitCritical(previous);
}

uint8_t RobotSafetyTryGrantRemoteChassisPermit(uint32_t expected_generation,
                                                uint32_t source_time_ms,
                                                uint32_t max_age_ms,
                                                uint32_t now_ms)
{
    const uint32_t previous = RobotSafetyEnterCritical();
#if defined(CHASSIS_BOARD)
    const uint8_t lease_valid = (uint8_t)(max_age_ms != 0u && max_age_ms <= INT32_MAX &&
        (uint32_t)(now_ms - source_time_ms) <= max_age_ms);
    const uint8_t granted = (uint8_t)(expected_generation == chassis_inhibit_generation &&
        RobotSafetyChassisLocalReadyLocked() != 0u && lease_valid != 0u);
    if (granted != 0u)
    {
        remote_chassis_permit = 1u;
        remote_chassis_source_time_ms = source_time_ms;
        remote_chassis_max_age_ms = max_age_ms;
    }
    else
        RobotSafetyClearRemoteChassisPermitLocked(1u);
#else
    (void)expected_generation;
    (void)source_time_ms;
    (void)max_age_ms;
    (void)now_ms;
    const uint8_t granted = 0u;
#endif
    RobotSafetyExitCritical(previous);
    return granted;
}

void RobotSafetyRevokeRemoteChassisPermit(void)
{
    const uint32_t previous = RobotSafetyEnterCritical();
#if defined(CHASSIS_BOARD)
    RobotSafetyClearRemoteChassisPermitLocked(1u);
#endif
    RobotSafetyExitCritical(previous);
}

uint8_t RobotSafetyIsChassisLocalReady(void)
{
#if defined(CHASSIS_BOARD)
    const uint32_t previous = RobotSafetyEnterCritical();
    const uint8_t ready = RobotSafetyChassisLocalReadyLocked();
    RobotSafetyExitCritical(previous);
    return ready;
#else
    return 0u;
#endif
}

void RobotFaultReport(Robot_Fault_e fault)
{
    const uint32_t previous = RobotSafetyEnterCritical();
    RobotSafetyReportFaultLocked(fault);
    RobotSafetyExitCritical(previous);
}

void RobotFaultClear(Robot_Fault_e fault)
{
    const uint32_t previous = RobotSafetyEnterCritical();
    RobotSafetyClearFaultLocked(fault);
    RobotSafetyExitCritical(previous);
}

// 调用方已进入安全临界区；这里只判断资格，不清除故障或推进状态。
static uint8_t RobotSafetyCanRecoverLocked(uint32_t expected_generation, uint32_t now_ms)
{
    const uint8_t formal = ControlFormalLocked(); // 正式自主不依赖遥控在线或人工使能。
    uint32_t other_faults = fault_flags & ~(uint32_t)ROBOT_FAULT_CAN_BUSOFF; // 此处仅考虑当前要恢复的 CAN 故障。
    if (formal) other_faults &= ~(uint32_t)ROBOT_FAULT_RC_OFFLINE; // 保留自主模式已有的遥控豁免。
    uint8_t allowed = (uint8_t)(control_generation == expected_generation &&
        (fault_flags & ROBOT_FAULT_CAN_BUSOFF) != 0u && other_faults == 0u &&
        imu_valid && calib_valid && task_healthy && !estop_latched && !calib_requested); // 核心或代际变化时不得清除故障。
    if (formal)
    {
        const RobotControlLease_t *match = &control_inputs.match; // 自主运行资格必须来自当前裁判阶段。
        allowed = (uint8_t)(allowed && !control_disabled && control_inputs.match_running &&
            match->valid == 1u && match->max_age_ms > 0u && match->max_age_ms <= INT32_MAX &&
            (uint32_t)(now_ms - match->source_time_ms) <= match->max_age_ms); // 过期或将来的阶段数据不能授权。
    }
    else
    {
        allowed = (uint8_t)(allowed && rc_online && enable_requested && !control_disabled); // 人工模式保留在线和显式使能要求。
        if (control_configured)
        {
            const RobotControlLease_t *manual = &control_inputs.manual; // 新策略还需已选择的人工来源有效。
            allowed = (uint8_t)(allowed && control_inputs.mode != ROBOT_CONTROL_AUTONOMOUS &&
                manual->valid == 1u && manual->max_age_ms > 0u && manual->max_age_ms <= INT32_MAX &&
                (uint32_t)(now_ms - manual->source_time_ms) <= manual->max_age_ms); // 无效或过期输入不能恢复。
        }
    }
    return allowed; // 调用方按同一临界区的判断决定是否清除。
}

uint8_t RobotSafetyCanRecover(uint32_t expected_generation)
{
    const uint32_t previous = RobotSafetyEnterCritical(); // 来源和核心状态必须在同一保护内读取。
    const uint8_t allowed = RobotSafetyCanRecoverLocked(expected_generation, PlatformTimeNowMs()); // 以实际查询时刻检查期限。
    RobotSafetyExitCritical(previous); // 恢复调用者的中断状态。
    return allowed; // 不清除故障，不代表允许运动。
}

uint8_t RobotSafetyTryClearCanFault(uint32_t expected_generation, uint32_t now_ms)
{
    (void)now_ms; // 恢复许可必须以安全管理实际检查时刻为准。
    const uint32_t previous = RobotSafetyEnterCritical(); // 同时检查并修改安全管理保存的状态。
    const uint8_t allowed = RobotSafetyCanRecoverLocked(expected_generation, PlatformTimeNowMs()); // 防止时间倒退或调用方传入旧时刻。
    if (allowed) RobotSafetyClearFaultLocked(ROBOT_FAULT_CAN_BUSOFF); // 只清除已验证的这一故障位。
    RobotSafetyExitCritical(previous); // 恢复进入前的中断状态，不推进整车状态。
    return allowed; // 剩余控制许可由正常状态机和新命令继续决定。
}

void EstopRequest(Estop_Reason_e reason)
{
    const uint32_t previous = RobotSafetyEnterCritical();
    const uint8_t log_request = RobotSafetyRequestEstopLocked(reason);
    RobotSafetyExitCritical(previous);

    if (log_request != 0u)
        BSP_LOG_EVENT_1(EV_MODULES_SAFETY_ROBOT_SAFETY_C_04,
                        BSP_LOG_FIELD_I32((int)reason));
}

uint8_t EstopClear(void)
{
    Robot_Status_e old_state;
    const uint32_t previous = RobotSafetyEnterCritical();
    uint32_t blocking = ROBOT_FAULT_TASK_TIMEOUT | ROBOT_FAULT_INIT_FAILURE | ROBOT_FAULT_RESET_LOCKOUT; // 核心禁止不能由急停解除。
    if (!ControlFormalLocked()) blocking |= ROBOT_FAULT_RC_OFFLINE; // 正式自主不依赖遥控，其他模式仍要求其恢复。
    if ((fault_flags & blocking) != 0u)
    {
        RobotSafetyExitCritical(previous);
        return 0u;
    }

    if (estop_latched == 0u)
    {
        RobotSafetyExitCritical(previous);
        return 1u;
    }

    estop_latched = 0u;
    estop_reason = ESTOP_REASON_NONE;
    enable_requested = 0u;
    old_state = robot_state;
    robot_state = ROBOT_SAFE;
    RobotSafetyExitCritical(previous);

    BSP_LOG_EVENT_0(EV_MODULES_SAFETY_ROBOT_SAFETY_C_05);
    RobotSafetyLogStateChange(old_state, ROBOT_SAFE);
    return 1u;
}

uint8_t EstopIsLatched(void)
{
    const uint32_t previous = RobotSafetyEnterCritical();
    const uint8_t latched = estop_latched;
    RobotSafetyExitCritical(previous);
    return latched;
}

Estop_Reason_e EstopGetReason(void)
{
    const uint32_t previous = RobotSafetyEnterCritical();
    const Estop_Reason_e reason = estop_reason;
    RobotSafetyExitCritical(previous);
    return reason;
}

Robot_Status_e RobotSafetyGetState(void)
{
    const uint32_t previous = RobotSafetyEnterCritical();
    const Robot_Status_e state = robot_state;
    RobotSafetyExitCritical(previous);
    return state;
}

uint8_t RobotSafetyIsReady(void)
{
    if (control_configured) return RobotSafetyIsReadyAt(PlatformTimeNowMs()); // 新策略必须按真实时间复查各来源。
#if defined(CHASSIS_BOARD)
    return RobotSafetyIsReadyAt(PlatformTimeNowMs());
#else
    return RobotSafetyIsReadyAt(0u);
#endif
}

uint8_t RobotSafetyIsReadyAt(uint32_t now_ms)
{
    const uint32_t previous = RobotSafetyEnterCritical();
    if (control_configured)
    {
        RobotControlPermits_t permits = {0}; // 默认没有功能许可。
        const uint8_t valid = ControlEvaluateLocked(now_ms, &permits); // 查询时即时撤销过期或故障目标。
        const uint8_t ready = (uint8_t)(valid && robot_state == ROBOT_READY && permits.functions != 0u); // 表示至少一项功能可用。
        RobotSafetyExitCritical(previous);
        return ready;
    }
#if defined(CHASSIS_BOARD)
    const uint8_t ready = (robot_state == ROBOT_READY &&
                           RobotSafetyChassisLocalReadyLocked() != 0u &&
                           RobotSafetyRemoteLeaseFreshLocked(now_ms) != 0u) ? 1u : 0u;
#else
    (void)now_ms;
    const uint8_t ready = (!control_vehicle_feedback && robot_state == ROBOT_READY && fault_flags == ROBOT_FAULT_NONE &&
                           rc_online != 0u && imu_valid != 0u && calib_valid != 0u &&
                           task_healthy != 0u && enable_requested != 0u &&
                           calib_requested == 0u && estop_latched == 0u) ? 1u : 0u;
#endif
    RobotSafetyExitCritical(previous);
    return ready;
}

uint8_t RobotSafetyIsReadyForGeneration(uint32_t now_ms, uint32_t expected_generation)
{
    const uint32_t previous = RobotSafetyEnterCritical(); // 故障事件不能插入代际和许可之间。
    const uint8_t currently_ready = RobotSafetyIsReadyAt(now_ms); // 来源过期检查可能本身推进安全代际。
    const uint8_t ready = (uint8_t)(currently_ready != 0u &&
                                     control_generation == expected_generation); // 完成许可计算后再核对代际。
    RobotSafetyExitCritical(previous); // 恢复进入前的中断状态。
    return ready;
}

uint8_t RobotSafetyGetOutputPermitsForGeneration(uint32_t now_ms, uint32_t expected_generation,
                                                  RobotControlPermits_t *permits, uint8_t *policy_active)
{
    if (permits == NULL || policy_active == NULL) return 0u; // 无法完整交付时禁止调用者使用旧结果。
    const uint32_t previous = RobotSafetyEnterCritical(); // 配置、代际和许可必须来自同一受保护状态。
    RobotControlPermits_t candidate = {0}; // 默认所有机构和击发均禁止。
    const uint8_t active = control_configured; // 标记是否启用了分功能策略。
    uint8_t valid = 1u; // 记录计算是否成功，禁止动作本身不是错误。
    if (active != 0u)
    {
        valid = ControlEvaluateLocked(now_ms, &candidate); // 根据当前来源和反馈重新计算。
        if (robot_state != ROBOT_READY) memset(&candidate, 0, sizeof(candidate)); // 管理者未放行时禁止全部功能。
    }
    else if (RobotSafetyIsReadyAt(now_ms) != 0u)
    {
        candidate.functions = ROBOT_CONTROL_ALL; // 旧整车路径不具备分功能信息，保留原有行为。
        candidate.fire_allowed = 1u; // 旧发射条件仍由发射模块判断。
    }
    if (control_generation != expected_generation) valid = 0u; // 计算过程中发生禁止变化时拒绝旧目标。
    if (valid != 0u)
    {
        *permits = candidate; // 同时交付许可和策略类型。
        *policy_active = active;
    }
    RobotSafetyExitCritical(previous); // 恢复调用者进入前的中断状态。
    return valid;
}

uint32_t RobotSafetyGetFaults(void)
{
    const uint32_t previous = RobotSafetyEnterCritical();
    const uint32_t faults = fault_flags;
    RobotSafetyExitCritical(previous);
    return faults;
}

uint8_t RobotSafetyGetSnapshot(RobotSafetySnapshot_t *snapshot)
{
    if (snapshot == NULL)
        return 0u;

    RobotSafetySnapshot_t candidate;
    const uint32_t previous = RobotSafetyEnterCritical();
    candidate.state = robot_state;
    candidate.fault_flags = fault_flags;
    candidate.rc_online = rc_online;
    candidate.imu_valid = imu_valid;
    candidate.calib_valid = calib_valid;
    candidate.task_healthy = task_healthy;
    candidate.enable_requested = enable_requested;
    candidate.calib_requested = calib_requested;
    candidate.estop_latched = estop_latched;
    candidate.remote_chassis_permit = remote_chassis_permit;
    candidate.chassis_inhibit_generation = chassis_inhibit_generation;
    candidate.estop_reason = estop_reason;
    candidate.control_generation = control_generation; // 与故障、状态和请求在同一保护中交付。
    RobotSafetyExitCritical(previous);

    *snapshot = candidate;
    return 1u;
}

const char *RobotSafetyStateName(Robot_Status_e state)
{
    switch (state)
    {
    case ROBOT_INIT: return "INIT";
    case ROBOT_SAFE: return "SAFE";
    case ROBOT_CALIB: return "CALIB";
    case ROBOT_READY: return "READY";
    case ROBOT_FAULT: return "FAULT";
    case ROBOT_ESTOP: return "ESTOP";
    case ROBOT_STOP:
    default: return "STOP";
    }
}

void RobotSafetyUpdate(void)
{
    Robot_Status_e old_state;
    Robot_Status_e new_state;
    Estop_Reason_e requested_estop_reason = ESTOP_REASON_NONE;
    const uint32_t previous = RobotSafetyEnterCritical();
    const uint32_t active_faults = fault_flags;

#if defined(CHASSIS_BOARD)
    // 任务周期之外也以租约期限判定；到期立刻撤销旧远端命令并要求新序号重接管。
    if (remote_chassis_permit != 0u && RobotSafetyRemoteLeaseFreshLocked(PlatformTimeNowMs()) == 0u)
        RobotSafetyClearRemoteChassisPermitLocked(1u);
    // 底盘板不接收遥控器；运动授权来自云台板的新鲜命令并继续受本地状态约束。
    if ((active_faults & (ROBOT_FAULT_INIT_FAILURE | ROBOT_FAULT_RESET_LOCKOUT)) != 0u)
    {
        calib_requested = 0u;
        new_state = ROBOT_FAULT;
    }
    else if (estop_latched != 0u)
    {
        new_state = ROBOT_ESTOP;
    }
    else if (task_healthy == 0u ||
             (active_faults & (uint32_t)~ROBOT_FAULT_RC_OFFLINE) != 0u)
    {
        new_state = ROBOT_FAULT;
    }
    else if (calib_requested != 0u)
    {
        new_state = ROBOT_CALIB;
    }
    else if (imu_valid == 0u || calib_valid == 0u || remote_chassis_permit == 0u)
    {
        new_state = ROBOT_SAFE;
    }
    else
    {
        new_state = ROBOT_READY;
    }
#else
    if ((active_faults & (ROBOT_FAULT_INIT_FAILURE | ROBOT_FAULT_RESET_LOCKOUT)) != 0u)
    {
        enable_requested = 0u;
        calib_requested = 0u;
        new_state = ROBOT_FAULT;
    }
    else
    {
        if (!ControlFormalLocked() && (rc_online == 0u || (active_faults & ROBOT_FAULT_RC_OFFLINE) != 0u))
        {
            if (RobotSafetyRequestEstopLocked(ESTOP_REASON_RC_OFFLINE) != 0u)
                requested_estop_reason = ESTOP_REASON_RC_OFFLINE;
        }

        if (estop_latched != 0u)
        {
            enable_requested = 0u;
            new_state = ROBOT_ESTOP;
        }
        else if (task_healthy == 0u ||
                 (active_faults & ROBOT_FAULT_TASK_TIMEOUT) != 0u)
        {
            enable_requested = 0u;
            new_state = ROBOT_FAULT;
        }
        else if (calib_requested != 0u)
        {
            new_state = ROBOT_CALIB;
        }
        else if ((active_faults & (ROBOT_FAULT_MOTOR_OFFLINE |
                                   ROBOT_FAULT_CAN_BUSOFF |
                                   ROBOT_FAULT_TILT)) != 0u)
        {
            new_state = ROBOT_FAULT;
        }
        else if (control_configured)
        {
            RobotControlPermits_t permits = {0}; // 独立功能许可由同一个管理者计算。
            const uint8_t valid = ControlEvaluateLocked(PlatformTimeNowMs(), &permits); // 使用当前来源和恢复起点，不借旧READY授权。
            new_state = valid && permits.functions != 0u ? ROBOT_READY : ROBOT_SAFE; // 无可用功能时保持SAFE等待新条件。
        }
        else if (control_vehicle_feedback)
        {
            new_state = ROBOT_SAFE; // 反馈已绑定但来源策略尚未准备好，不能使用旧整车许可。
        }
        else if (enable_requested != 0u)
        {
            if (active_faults != ROBOT_FAULT_NONE || rc_online == 0u ||
                imu_valid == 0u || calib_valid == 0u)
                new_state = ROBOT_SAFE;
            else
                new_state = ROBOT_READY;
        }
        else
        {
            new_state = ROBOT_SAFE;
        }
    }
#endif

    old_state = robot_state;
    robot_state = new_state;
    RobotSafetyExitCritical(previous);

    if (requested_estop_reason != ESTOP_REASON_NONE)
        BSP_LOG_EVENT_1(EV_MODULES_SAFETY_ROBOT_SAFETY_C_04,
                        BSP_LOG_FIELD_I32((int)requested_estop_reason));
    RobotSafetyLogStateChange(old_state, new_state);
}
