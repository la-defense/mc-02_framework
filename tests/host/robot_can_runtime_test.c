/** @file robot_can_runtime_test.c
 * @brief 用生产安全管理验证核心与辅助 CAN 故障的不同影响。
 */
#include "robot_can_runtime.h"
#include "platform_can.h"
#include "robot_safety.h"
#include "can_recovery.h" // 检查实际故障代次能否打断恢复。
#include "test_check.h"

static PlatformCanFaultObserver_t installed_observer; // 保存平台安装的观察者供故障注入。
static void *installed_context; // 回调时传回相同上下文。
static uint8_t reject_observer_install; // 模拟平台无法安装观察者。
static uint32_t now_ms; // 本机测试时钟。
static PlatformCanStatus_t bus_status[3] = {PLATFORM_CAN_STATUS_OK, PLATFORM_CAN_STATUS_OK,
                                           PLATFORM_CAN_STATUS_OK}; // 每条逻辑总线独立注入状态。
static uint8_t inject_fault_in_status; // 查询第二条核心总线时可插入短故障。

uint32_t HAL_GetTick(void) { return now_ms; } // 平台时基使用主机测试时间。
PlatformCanStatus_t PlatformCanGetStatus(PlatformCanBus_t bus)
{
    if (bus < PLATFORM_CAN_BUS_1 || bus > PLATFORM_CAN_BUS_3) return PLATFORM_CAN_STATUS_UNKNOWN;
    if (inject_fault_in_status && bus == PLATFORM_CAN_BUS_3)
    {
        inject_fault_in_status = 0u; // 每轮只注入一次。
        installed_observer(PLATFORM_CAN_BUS_1, installed_context); // 模拟查询前一条总线后发生短故障。
    }
    return bus_status[(unsigned)bus - (unsigned)PLATFORM_CAN_BUS_1]; // 返回该控制器的当前状态。
}
uint8_t PlatformCanBusIsBound(PlatformCanBus_t bus)
{
    return (uint8_t)(bus >= PLATFORM_CAN_BUS_1 && bus <= PLATFORM_CAN_BUS_3); // 测试板声明三条逻辑总线。
}
uint8_t PlatformCanInstallFaultObserver(PlatformCanFaultObserver_t observer, void *context)
{
    if (observer == NULL || installed_observer != NULL || reject_observer_install) return 0u; // 不接受空、重复或被注入拒绝的安装。
    installed_context = context; // 先保存上下文。
    installed_observer = observer; // 使故障注入能调用生产观察者。
    return 1u;
}

int main(void)
{
    RobotSafetyInit(); // 建立真实安全管理状态。
    RobotSafetySetRcOnline(1u); // 人工控制所需遥控在线。
    RobotSafetySetImuValid(1u); // 核心姿态有效。
    RobotSafetySetCalibValid(1u); // 核心标定有效。
    RobotSafetySetTaskHealthy(1u); // 核心任务健康。
    RobotSafetyRequestEnable(); RobotSafetyUpdate(); // 操作者请求使能。
    TEST_CHECK(RobotSafetyIsReady()); // 故障前整车具备运行条件。
    RobotModulePlan_t plan = {0}; // 构造隔离的测试车型依赖。
    plan.enabled = ROBOT_MODULE_BIT(ROBOT_MODULE_CHASSIS) | ROBOT_MODULE_BIT(ROBOT_MODULE_ALARM) |
                   ROBOT_MODULE_BIT(ROBOT_MODULE_COMMUNICATION); // 底盘、必要通信和辅助模块均启用。
    plan.required = ROBOT_MODULE_BIT(ROBOT_MODULE_CHASSIS); // 只有底盘属于核心。
    plan.dependencies[ROBOT_MODULE_CHASSIS] = ROBOT_MODULE_BIT(ROBOT_MODULE_COMMUNICATION); // 通信是底盘必要依赖。
    const RobotCanFilterDemand_t demands[] = {
        {ROBOT_MODULE_CHASSIS, PLATFORM_CAN_BUS_1, 4u, 0u}, // 第一条总线承载核心轮电机。
        {ROBOT_MODULE_ALARM, PLATFORM_CAN_BUS_2, 1u, 0u}, // 第二条总线只承载辅助设备。
        {ROBOT_MODULE_COMMUNICATION, PLATFORM_CAN_BUS_3, 1u, 0u}, // 第三条总线承载核心依赖。
    };
    RobotCanFaultSnapshot_t events = {0}; // 结果只在观察者成功安装后有效。
    RobotCanReadiness_t readiness = {0}; // 核心总线状态只能在观察者已安装后读取。
    TEST_CHECK(!RobotCanRuntimeGetReadiness(&readiness)); // 未安装不能绕过故障代次保护。
    TEST_CHECK(!RobotCanRuntimeGetFaultSnapshot(&events)); // 启动前不能伪造总线状态。
    TEST_CHECK(!RobotCanRuntimeInstall(NULL, demands, 2u)); // 空计划不能建立故障绑定。
    TEST_CHECK(!RobotCanRuntimeInstall(&plan, NULL, 1u)); // 需求数组缺失必须拒绝。
    RobotCanFilterDemand_t invalid = {ROBOT_MODULE_CHASSIS, PLATFORM_CAN_INVALID, 1u, 0u}; // 非法总线不能被忽略。
    TEST_CHECK(!RobotCanRuntimeInstall(&plan, &invalid, 1u)); // 失败后仍可安装有效配置。
    invalid = (RobotCanFilterDemand_t){ROBOT_MODULE_GIMBAL, PLATFORM_CAN_BUS_1, 1u, 0u}; // 未启用模块不能声明有效需求。
    TEST_CHECK(!RobotCanRuntimeInstall(&plan, &invalid, 1u)); // 不接受与车型计划不一致的需求。
    reject_observer_install = 1u; // 模拟平台拒绝观察者安装。
    TEST_CHECK(!RobotCanRuntimeInstall(&plan, demands, 3u) && installed_observer == NULL); // 启动者必须收到失败，不能假定保护已接入。
    reject_observer_install = 0u; // 后续验证成功安装。
    TEST_CHECK(RobotCanRuntimeInstall(&plan, demands, 3u)); // 安装真实车型依赖转换。
    TEST_CHECK(installed_observer != NULL); // 核心总线必须建立观察者。
    TEST_CHECK(RobotCanRuntimeGetFaultSnapshot(&events) && events.core_bus_mask == 5u &&
               events.fault_generation == 0u); // 第一和第三条总线属于核心闭包。
    bus_status[1] = PLATFORM_CAN_STATUS_BUSOFF; // 辅助总线故障不改变核心总线状态。
    TEST_CHECK(RobotCanRuntimeGetReadiness(&readiness) && readiness.ready_bus_mask == 5u);
    bus_status[2] = PLATFORM_CAN_STATUS_UNKNOWN; // 必需总线无法确认时不能恢复。
    TEST_CHECK(RobotCanRuntimeGetReadiness(&readiness) && readiness.ready_bus_mask == 1u);
    bus_status[2] = PLATFORM_CAN_STATUS_OK; // 恢复测试状态。
    installed_observer(PLATFORM_CAN_BUS_2, installed_context); // 辅助总线故障不应全局失能。
    TEST_CHECK(RobotSafetyIsReady()); // 核心许可保持有效。
    TEST_CHECK(RobotCanRuntimeGetFaultSnapshot(&events) && events.fault_generation == 0u); // 辅助故障不改变核心代次。
    installed_observer(PLATFORM_CAN_BUS_1, installed_context); // 核心总线故障立即通知。
    TEST_CHECK(!RobotSafetyIsReady()); // 无须等下一次状态更新即撤销许可。
    TEST_CHECK((RobotSafetyGetFaults() & ROBOT_FAULT_CAN_BUSOFF) != 0u); // 故障原因可查询。
    TEST_CHECK(RobotCanRuntimeGetFaultSnapshot(&events) && events.fault_generation == 1u &&
               events.bus_generation[0] == 1u && events.fault_bus_mask == 1u); // 记录首个核心故障。
    installed_observer(PLATFORM_CAN_BUS_1, installed_context); // 故障位仍置位时再次发生故障。
    TEST_CHECK(RobotCanRuntimeGetFaultSnapshot(&events) && events.fault_generation == 2u &&
               events.bus_generation[0] == 2u); // 不能将恢复期间的第二次故障吞掉。
    installed_observer(PLATFORM_CAN_BUS_3, installed_context); // 另一条核心依赖总线随后故障。
    TEST_CHECK(RobotCanRuntimeGetFaultSnapshot(&events) && events.fault_generation == 3u &&
               events.bus_generation[0] == 2u && events.bus_generation[2] == 1u &&
               events.fault_bus_mask == 5u); // 各总线代次独立，整体代次记录全部事件。
    inject_fault_in_status = 1u; // 两条总线状态逐一查询的间隙发生核心故障。
    TEST_CHECK(RobotCanRuntimeGetReadiness(&readiness) && readiness.ready_bus_mask == 0u &&
               readiness.fault_generation == 4u); // 即使硬件状态已恢复，本轮也不能交付旧结果。
    TEST_CHECK(RobotCanRuntimeGetFaultSnapshot(&events) && events.fault_generation == 4u); // 后续恢复计算从新代次开始。
    CanRecovery_t recovery = {0}; // 创建独立的主机恢复检查实例。
    const CanRecoveryConfig_t recovery_config = {1u, 100u}; // 100 ms 是待实测的哨兵候选值。
    TEST_CHECK(CanRecoveryInit(&recovery, &recovery_config)); // 配置必须有效。
    CanRecoveryInput_t input = {.now_ms = 1u, .inhibit_generation = events.fault_generation,
        .permit_ok = 1u, .feedback_sequence = {5u}, .command_sequence = 10u}; // 故障中记录旧样本和命令。
    TEST_CHECK(!CanRecoveryStep(&recovery, &input)); // 故障总线不能授予恢复。
    input.now_ms = 2u; input.bus_ok = 1u; input.valid_feedback_mask = 1u; input.feedback_sequence[0] = 6u; // 模拟总线恢复后的新反馈。
    TEST_CHECK(!CanRecoveryStep(&recovery, &input)); // 先进入稳定等待。
    input.now_ms = 102u; TEST_CHECK(!CanRecoveryStep(&recovery, &input)); // 满足候选时间后等待新命令。
    installed_observer(PLATFORM_CAN_BUS_1, installed_context); // 命令到达之前再次发生核心故障。
    TEST_CHECK(RobotCanRuntimeGetFaultSnapshot(&events)); // 读取真实故障入口更新的代次。
    input.now_ms = 103u; input.inhibit_generation = events.fault_generation; input.command_sequence = 11u; // 即使同时来了新命令也不能跳过故障。
    TEST_CHECK(!CanRecoveryStep(&recovery, &input) && recovery.phase == CAN_RECOVERY_WAIT_FEEDBACK); // 新代次要求重新验证反馈。
    RobotSafetyUpdate(); // 控制器恢复不能自行清除故障。
    TEST_CHECK(!RobotSafetyIsReady()); // 在恢复流程接入前保持禁止。
    TEST_CHECK(!RobotCanRuntimeInstall(&plan, demands, 2u)); // 不允许运行中重置观察者。
    RobotSafetySnapshot_t safety = {0}; // 模拟协调者最后一次读取的安全代际。
    TEST_CHECK(RobotSafetyGetSnapshot(&safety));
    TEST_CHECK(RobotSafetyCanRecover(safety.control_generation)); // 检查资格本身不清除故障。
    TEST_CHECK((RobotSafetyGetFaults() & ROBOT_FAULT_CAN_BUSOFF) != 0u);
    TEST_CHECK(!RobotCanRuntimeTryClearFault(events.fault_generation - 1u, safety.control_generation, now_ms)); // 过期的总线代次不能清除新故障。
    RobotFaultReport(ROBOT_FAULT_TILT); // 检查期间出现另一项核心故障。
    TEST_CHECK(!RobotCanRuntimeTryClearFault(events.fault_generation, safety.control_generation, now_ms)); // 旧安全代次不能提交。
    TEST_CHECK(RobotSafetyGetSnapshot(&safety));
    TEST_CHECK(!RobotCanRuntimeTryClearFault(events.fault_generation, safety.control_generation, now_ms)); // 即使取得新代次，其他故障也阻止清除。
    RobotFaultClear(ROBOT_FAULT_TILT); // 模拟其他故障已由各自流程恢复。
    RobotSafetyRequestDisable(); // 人工主动失能。
    TEST_CHECK(RobotSafetyGetSnapshot(&safety));
    TEST_CHECK(!RobotSafetyCanRecover(safety.control_generation));
    TEST_CHECK(!RobotCanRuntimeTryClearFault(events.fault_generation, safety.control_generation, now_ms)); // 失能期间不能清除 CAN 故障。
    RobotSafetyRequestEnable(); // 模拟满足前置条件后的明确使能请求。
    TEST_CHECK(RobotSafetyGetSnapshot(&safety));
    TEST_CHECK(RobotCanRuntimeTryClearFault(events.fault_generation, safety.control_generation, now_ms)); // 当前代次和核心资格均有效时才清除。
    TEST_CHECK((RobotSafetyGetFaults() & ROBOT_FAULT_CAN_BUSOFF) == 0u);
    TEST_CHECK(!RobotSafetyIsReady()); // 清除故障本身不推进整车状态。
    RobotSafetyUpdate(); // 由原整车状态机重新检查是否允许运行。
    TEST_CHECK(RobotSafetyIsReady());

    RobotSafetyInit(); // 重新构造正式自主测试，不沿用人工使能。
    RobotSafetySetImuValid(1u); RobotSafetySetCalibValid(1u); RobotSafetySetTaskHealthy(1u);
    const RobotControlPolicy_t autonomous_policy = {ROBOT_CONTROL_ALL, 1u, 1u, 0u}; // 测试车型具备正式自主资格。
    TEST_CHECK(RobotSafetyConfigureControl(&autonomous_policy));
    RobotSafetyControlInputs_t autonomous = {0};
    autonomous.mode = ROBOT_CONTROL_AUTONOMOUS; // 自主运行不依赖遥控。
    autonomous.match = (RobotControlLease_t){10u, 100u, 1u}; // 裁判阶段有独立时限。
    autonomous.match_running = 1u; // 裁判阶段允许比赛运行。
    now_ms = 10u;
    TEST_CHECK(RobotSafetyPublishControlInputs(&autonomous));
    RobotSafetySetRcOnline(0u); // 遥控离线只保留诊断，不阻止正式自主。
    installed_observer(PLATFORM_CAN_BUS_1, installed_context);
    TEST_CHECK(RobotCanRuntimeGetFaultSnapshot(&events) && RobotSafetyGetSnapshot(&safety));
    now_ms = 111u; // 比赛阶段数据已经过期。
    TEST_CHECK(!RobotSafetyCanRecover(safety.control_generation));
    TEST_CHECK(!RobotCanRuntimeTryClearFault(events.fault_generation, safety.control_generation, 10u)); // 传入旧时间不能延长比赛阶段期限。
    autonomous.match.source_time_ms = 111u; // 模拟新的有效裁判阶段。
    TEST_CHECK(RobotSafetyPublishControlInputs(&autonomous));
    TEST_CHECK(RobotSafetyGetSnapshot(&safety));
    TEST_CHECK(RobotSafetyCanRecover(safety.control_generation)); // 正式自主资格无需遥控或人工使能。
    TEST_CHECK(RobotCanRuntimeTryClearFault(events.fault_generation, safety.control_generation, now_ms)); // 比赛有效且其他核心条件正常时不要求遥控。
    installed_observer(PLATFORM_CAN_BUS_1, installed_context); // 再次模拟 CAN 故障。
    EstopRequest(ESTOP_REASON_MANUAL); // 操作者尚未解除急停。
    TEST_CHECK(RobotCanRuntimeGetFaultSnapshot(&events) && RobotSafetyGetSnapshot(&safety));
    TEST_CHECK(!RobotSafetyCanRecover(safety.control_generation));
    TEST_CHECK(!RobotCanRuntimeTryClearFault(events.fault_generation, safety.control_generation, now_ms)); // 正式自主也不能绕过人工急停。
    return 0;
}
