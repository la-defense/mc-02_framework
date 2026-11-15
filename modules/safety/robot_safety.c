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
static uint32_t control_local_wait; // 发生过局部禁止、恢复后需新来源的功能位。
static uint32_t control_local_barrier[3]; // 底盘／云台／发射各自恢复时刻，避免局部故障中断其他功能。
static RobotControlPolicy_t control_policy;
static RobotSafetyControlInputs_t control_inputs;

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

uint8_t RobotSafetyPublishControlInputs(const RobotSafetyControlInputs_t *inputs)
{
    const uint32_t previous = RobotSafetyEnterCritical(); // 数据与核心状态在同一保护内检查。
    uint8_t accepted = 0u; // 默认不接受尚未配置的提交。
    if (control_configured)
    {
        if (inputs != NULL)
        {
            const RobotControlFacts_t facts = ControlFactsLocked(inputs); // 核心和使能始终由管理者提供。
            RobotControlPermits_t unused;
            accepted = (uint8_t)(RobotControlEvaluate(&control_policy, &facts, PlatformTimeNowMs(), &unused) == ROBOT_CONTROL_OK); // 检查来源枚举和期限。
        }
        if (accepted)
        {
            const uint32_t old_available = control_policy.enabled_functions & control_inputs.feedback_valid_functions & ~control_inputs.blocked_functions; // 上一批可用功能。
            const uint32_t new_available = control_policy.enabled_functions & inputs->feedback_valid_functions & ~inputs->blocked_functions; // 本批可用功能。
            const uint32_t lost = old_available & ~new_available; // 只记录本次新出现的局部禁止。
            const uint32_t recovered = new_available & ~old_available & control_local_wait; // 首次得到反馈不当作故障恢复。
            const uint32_t changed = lost | recovered; // 故障和恢复边沿都留下时间，重复发布不会续期。
            control_local_wait |= lost; // 保留已发生的局部禁止。
            for (unsigned i = 0u; i < 3u; ++i)
                if (changed & (1u << i)) control_local_barrier[i] = PlatformTimeNowMs(); // 每个功能独立等待恢复后的新来源。
            if (lost) control_generation++; // 对查询者保留周期之间的局部故障变化。
            control_inputs = *inputs; // 全部检查后一次发布来源与反馈事实。
        }
        else
        {
            const RobotControlMode_t mode = control_inputs.mode; // 错误提交不能切换到其他有动作模式。
            memset(&control_inputs, 0, sizeof(control_inputs)); // 清除旧来源和反馈位，立即阻止旧结果继续使用。
            control_inputs.mode = mode; // 正式自主可保留模式等待新阶段事实，但无功能许可。
            ControlRevokeLocked(); // 错误提交后也要求重新接管新输入。
        }
    }
    RobotSafetyExitCritical(previous); // 发布结束后恢复进入前的屏蔽状态。
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
    control_configured = 0u; // 系统重新初始化后须由车型重新配置模式策略。
    control_disabled = 0u; // 尚未配置的旧路径不受新策略影响。
    control_was_healthy = 0u; // 新一次启动重新建立核心恢复边沿。
    control_barrier_valid = 0u; // 尚未发生新启动周期中的接管检查。
    control_barrier_ms = 0u; // 初始数值不作为有效来源时间。
    control_generation = 0u; // 该计数只描述当前启动周期。
    control_local_wait = 0u; // 新启动没有沿用上个周期的局部恢复记录。
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
    const uint8_t ready = (robot_state == ROBOT_READY && fault_flags == ROBOT_FAULT_NONE &&
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
