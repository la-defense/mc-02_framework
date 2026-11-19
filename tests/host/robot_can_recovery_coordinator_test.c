/** @file robot_can_recovery_coordinator_test.c
 * @brief 使用模拟输入验证生产协调器的等待、间断和最终复核。
 */
#include "robot_can_recovery_coordinator.h"
#include "robot_can_runtime.h"
#include "robot_safety.h"
#include "platform_irq.h"
#include "test_check.h"
#include <string.h>

static uint32_t tick, can_generation, safety_generation, gap, sequence, clears; // 模拟各独立来源的代次。
static uint8_t bus_ready = 1u, permit = 1u, final_feedback = 1u, final_remote = 1u; // 按场景改变检查结果。
static uint8_t final_bus_ready = 1u, formal_mode; // 分别模拟最终总线状态和控制模式。
static uint32_t final_check_delay; // 模拟最终检查前反馈期限恰好到期。
static uint32_t irq_state; // 保留调用前的中断状态。
uint32_t PlatformTimeNowMs(void) { return tick; }
PlatformIrqState_t PlatformIrqSaveAndDisable(void) { const uint32_t old = irq_state; irq_state = 1u; return old; }
void PlatformIrqRestore(PlatformIrqState_t old) { irq_state = old; }
uint8_t RobotSafetyIsFormalAutonomous(void) { return formal_mode; }
uint8_t RobotSafetyGetSnapshot(RobotSafetySnapshot_t *out)
{
    *out = (RobotSafetySnapshot_t){.fault_flags = ROBOT_FAULT_CAN_BUSOFF, .control_generation = safety_generation};
    return 1u;
}
uint8_t RobotSafetyCanRecover(uint32_t generation) { return (uint8_t)(permit && generation == safety_generation); }
uint8_t RobotCanRuntimeGetReadiness(RobotCanReadiness_t *out)
{
    *out = (RobotCanReadiness_t){1u, bus_ready ? 1u : 0u, can_generation};
    return 1u;
}
uint8_t PlatformCanRecoveryStillReady(PlatformCanBus_t bus)
{
    TEST_CHECK(irq_state == 1u); // 最后一次硬件检查必须在同一保护内。
    tick += final_check_delay; // 模拟读取硬件期间本机时钟推进。
    final_check_delay = 0u; // 每个注入只执行一次。
    return (uint8_t)(bus_ready && final_bus_ready && bus == PLATFORM_CAN_BUS_1);
}
uint8_t RobotCanRuntimeTryClearFault(uint32_t can, uint32_t safety, uint32_t now)
{
    TEST_CHECK(irq_state == 1u && now == tick); // 清除和最后的检查处于同一保护中。
    if (!permit || can != can_generation || safety != safety_generation) return 0u;
    ++clears;
    return 1u;
}
uint8_t RobotCanMotorPlanBuild(const Robot_Config_s *config, const RobotModulePlan_t *modules,
                               RobotCanMotorPlan_t *out)
{
    (void)config; (void)modules;
    *out = (RobotCanMotorPlan_t){.count = 1u}; // 此处仅模拟一台必需电机。
    return 1u;
}
uint8_t DJIMotorGetRecoveryFeedback(uint32_t now, DJI_RecoveryFeedback_s *out)
{
    *out = (DJI_RecoveryFeedback_s){.count = 1u};
    out->motors[0].sequence = sequence; // 模拟新收到的电机样本。
    out->motors[0].source_time_ms = now;
    out->motors[0].gap_generation = gap; // 模拟两轮检查间的反馈间断。
    return 1u;
}
uint8_t RobotCanMotorRecoveryMapWithMask(const RobotCanMotorPlan_t *plan, const DJI_RecoveryFeedback_s *feedback,
    uint32_t now, CanRecoveryInput_t *input, uint32_t *mask)
{
    (void)plan; (void)now;
    *mask = 1u; // 此场景的核心电机是登记表第一项。
    input->valid_feedback_mask = 1u;
    input->feedback_sequence[0] = feedback->motors[0].sequence;
    return 1u;
}
uint8_t DJIMotorRecoveryStillValid(const DJI_RecoveryFeedback_s *observed, uint32_t mask, uint32_t now)
{
    TEST_CHECK(irq_state == 1u && mask == 1u && now == tick);
    return (uint8_t)(final_feedback && observed->motors[0].gap_generation == gap);
}
uint8_t RemoteControlRecoveryStillValid(const RemoteControlSnapshot_s *remote, uint32_t now)
{
    TEST_CHECK(irq_state == 1u && now == tick);
    return (uint8_t)(final_remote && remote->radio_sequence != 0u);
}

// 每轮均模拟一份当前输入及反馈，避免把重复读取当成新接收。
static uint8_t Step(RobotCanRecoveryCoordinator_t *state, RemoteControlSnapshot_s *remote, uint32_t time)
{
    tick = time; // 控制恢复判断使用的本机时间。
    ++sequence; // 模拟新电机反馈。
    ++remote->radio_sequence; // 模拟新遥控帧。
    remote->radio_source_time_ms = time; // 来源时间随实际新帧推进。
    const MessageFreshness_t freshness = {.valid = 1u, .timestamp_ms = time}; // 模拟当前云台反馈。
    return RobotCanRecoveryCoordinatorStep(state, remote, 0u, &freshness);
}

int main(void)
{
    Robot_Config_s config = {0}; // 未确认候选时间时默认禁止自动恢复。
    RobotModulePlan_t modules = {.enabled = ROBOT_MODULE_BIT(ROBOT_MODULE_GIMBAL)}; // 本场景检查云台失效是否撤销恢复。
    RobotCanRecoveryCoordinator_t state = {0};
    RemoteControlSnapshot_s remote = {.remote_online = 1u, .source_selection_generation = 1u};
    remote.input.rc.switch_left = RC_SW_DOWN;
    config.can_recovery.feedback_stable_ms = 100u; // 这里仅为隔离测试值。
    TEST_CHECK(RobotCanRecoveryCoordinatorInit(&state, &config, &modules));
    TEST_CHECK(!Step(&state, &remote, 1u) && !state.enabled && clears == 0u);
    config.can_recovery.confirmed = 1u; // 测试专用配置，生产车型确认状态不变。
    TEST_CHECK(RobotCanRecoveryCoordinatorInit(&state, &config, &modules));
    TEST_CHECK(!Step(&state, &remote, 2u)); // 首次检查只记录既有反馈。
    TEST_CHECK(!Step(&state, &remote, 3u)); // 收到故障后的反馈后开始计时。
    TEST_CHECK(!Step(&state, &remote, 102u)); // 尚不足 100 ms。
    ++gap; // 两轮之间曾断流，即使当前样本有效也要重新开始。
    TEST_CHECK(!Step(&state, &remote, 103u));
    TEST_CHECK(!Step(&state, &remote, 104u)); // 间断之后新反馈开始计时。
    TEST_CHECK(!Step(&state, &remote, 204u)); // 稳定完成后仍需新命令。
    final_remote = 0u; // 最终来源变化不能清除故障。
    TEST_CHECK(!Step(&state, &remote, 205u) && clears == 0u);
    final_remote = 1u;
    TEST_CHECK(!Step(&state, &remote, 206u)); // 最终检查失败后重新等反馈。
    TEST_CHECK(!Step(&state, &remote, 306u)); // 重新满足稳定时间。
    final_feedback = 0u; // 最终反馈变化也不能清除故障。
    TEST_CHECK(!Step(&state, &remote, 307u) && clears == 0u);
    final_feedback = 1u;
    TEST_CHECK(!Step(&state, &remote, 308u));
    TEST_CHECK(!Step(&state, &remote, 408u));
    TEST_CHECK(Step(&state, &remote, 409u) && clears == 1u); // 新来源和反馈通过最终检查。
    TEST_CHECK(irq_state == 0u); // 每次失败和成功都恢复调用前屏蔽状态。
    TEST_CHECK(RobotCanRecoveryCoordinatorInit(&state, &config, &modules)); // 重新模拟另一次故障。
    TEST_CHECK(!Step(&state, &remote, 300u) && !Step(&state, &remote, 301u)); // 初始反馈与开始计时。
    tick = 390u; ++sequence; ++remote.radio_sequence; remote.radio_source_time_ms = tick;
    TEST_CHECK(!RobotCanRecoveryCoordinatorStep(&state, &remote, 0u, NULL)); // 云台应用反馈失效中断等待。
    TEST_CHECK(!Step(&state, &remote, 391u)); // 新反馈重新开始计时。
    TEST_CHECK(!Step(&state, &remote, 491u)); // 稳定完成后依旧等待新命令。
    final_bus_ready = 0u; // 最终寄存器检查发现控制器刚刚发生故障。
    TEST_CHECK(!Step(&state, &remote, 492u) && clears == 1u);
    final_bus_ready = 1u;
    TEST_CHECK(!Step(&state, &remote, 493u)); // 控制器恢复后重新等待反馈和稳定时间。
    formal_mode = 1u;
    TEST_CHECK(!Step(&state, &remote, 500u)); // 自主来源尚未接入时不能沿用人工等待。
    formal_mode = 0u;
    TEST_CHECK(!Step(&state, &remote, 501u)); // 切回人工后从新反馈重新计时。
    TEST_CHECK(!Step(&state, &remote, 601u)); // 时间满足后先等待新的命令。
    TEST_CHECK(Step(&state, &remote, 602u) && clears == 2u); // 人工新命令才能恢复。
    TEST_CHECK(RobotCanRecoveryCoordinatorInit(&state, &config, &modules)); // 准备期限边界场景。
    TEST_CHECK(!Step(&state, &remote, 700u) && !Step(&state, &remote, 701u));
    TEST_CHECK(!Step(&state, &remote, 801u)); // 稳定期结束后等待新输入。
    final_check_delay = MESSAGE_DEFAULT_MAX_AGE_MS + 1u; // 模拟云台反馈在最终检查前到期。
    TEST_CHECK(!Step(&state, &remote, 802u) && clears == 2u); // 过期反馈不能清除故障。
    TEST_CHECK(!Step(&state, &remote, tick + 1u)); // 反馈到期后必须重新开始计时。
    TEST_CHECK(!Step(&state, &remote, tick + 100u)); // 新的稳定周期结束后仍等待命令。
    TEST_CHECK(Step(&state, &remote, tick + 1u) && clears == 3u); // 稳定后的新输入才可恢复。
    return 0;
}
