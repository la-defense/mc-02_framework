/** @file robot_can_recovery_coordinator_test.c
 * @brief 使用模拟输入验证生产协调器的等待、间断和最终复核。
 */
#include "robot_can_recovery_coordinator.h"
#include "robot_can_runtime.h"
#include "robot_safety.h"
#include "message_center.h"
#include "platform_irq.h"
#include "topics/robot_topics.h"
#include "test_check.h"
#include <string.h>

static uint32_t tick, can_generation, safety_generation, gap, sequence, clears; // 模拟各独立来源的代次。
static uint8_t bus_ready = 1u, permit = 1u, final_feedback = 1u, final_remote = 1u; // 按场景改变检查结果。
static uint8_t final_bus_ready = 1u, formal_mode; // 分别模拟最终总线状态和控制模式。
static uint8_t final_clear_allowed = 1u; // 模拟前置条件通过后，最终安全许可仍可能拒绝清除。
static uint32_t final_check_delay; // 模拟最终检查前反馈期限恰好到期。
static uint32_t irq_state; // 保留调用前的中断状态。
static uint8_t invalidate_before_final_lock; // 模拟取到应用反馈后、最终保护前的撤销。
static MessagePublisher_t *application_publisher;
static MessageSubscriber_t *application_subscriber;
static uint32_t last_application_publish_ms;
uint32_t PlatformTimeNowMs(void) { return tick; }
void PlatformTaskCriticalEnter(void) {} // 用例在同一主机线程控制事件顺序。
void PlatformTaskCriticalExit(void) {}
PlatformIrqState_t PlatformIrqSaveAndDisable(void)
{
    if (invalidate_before_final_lock)
    {
        invalidate_before_final_lock = 0u; // 一次性模拟更高优先级任务撤销反馈。
        TEST_CHECK(MessagePublisherInvalidate(application_publisher) == MESSAGE_OK);
    }
    const uint32_t old = irq_state;
    irq_state = 1u;
    return old;
}
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
    if (!final_clear_allowed || !permit || can != can_generation || safety != safety_generation) return 0u;
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

static void PublishApplicationAt(uint32_t time)
{
    const uint8_t feedback = 1u; // 内容由命令任务判定，此处只测试来源期限。
    TEST_CHECK(MessagePublishAt(application_publisher, &feedback, time, time, time, MESSAGE_CLOCK_LOCAL) == MESSAGE_OK);
    last_application_publish_ms = time;
}

// 发布者通常比恢复检查快，在两次检查之间补足连续有效的应用反馈。
static uint8_t Step(RobotCanRecoveryCoordinator_t *state, RemoteControlSnapshot_s *remote, uint32_t time)
{
    while (time - last_application_publish_ms > 25u && time - last_application_publish_ms <= INT32_MAX)
        PublishApplicationAt(last_application_publish_ms + 25u); // 让上一份反馈存续到下一次发布。
    tick = time; // 控制恢复判断使用的本机时间。
    PublishApplicationAt(time);
    ++sequence; // 模拟新电机反馈。
    ++remote->radio_sequence; // 模拟新遥控帧。
    remote->radio_source_time_ms = time; // 来源时间随实际新帧推进。
    uint8_t feedback;
    MessageFreshness_t freshness;
    TEST_CHECK(MessageReadSnapshot(application_subscriber, &feedback, sizeof(feedback), time, &freshness) == MESSAGE_OK);
    return RobotCanRecoveryCoordinatorStep(state, remote, 0u, &freshness);
}

int main(void)
{
    const MessageTopicDescriptor_t application_topic = {
        .topic_id = ROBOT_TOPIC_GIMBAL_FEEDBACK, // 与固件使用同一话题编号。
        .payload_size = sizeof(uint8_t), // 测试只消费消息时间和发布序号。
        .max_age_ms = MESSAGE_DEFAULT_MAX_AGE_MS, // 使用固件的云台反馈期限。
        .capacity = 1u, // 只保留最新反馈。
        .delivery = MESSAGE_DELIVERY_SNAPSHOT, // 自然过期或撤销后重新划分连续区间。
        .overflow = MESSAGE_OVERFLOW_REPLACE_OLDEST, // 连续更新可以覆盖旧值。
        .source_clock = MESSAGE_CLOCK_LOCAL, // 接收、发布和检查都使用本机毫秒。
        .publisher_mask = UINT32_C(1) << (ROBOT_TOPIC_ENDPOINT_GIMBAL - 1u), // 仅云台发布。
        .subscriber_mask = UINT32_C(1) << (ROBOT_TOPIC_ENDPOINT_COMMAND - 1u), // 仅命令端读反馈。
    };
    TEST_CHECK(MessageCenterInit(&application_topic, 1u) == MESSAGE_OK);
    application_publisher = MessagePublisherRegister(ROBOT_TOPIC_GIMBAL_FEEDBACK, ROBOT_TOPIC_ENDPOINT_GIMBAL);
    application_subscriber = MessageSubscriberRegister(ROBOT_TOPIC_GIMBAL_FEEDBACK, ROBOT_TOPIC_ENDPOINT_COMMAND);
    TEST_CHECK(application_publisher != NULL && application_subscriber != NULL);
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

    TEST_CHECK(RobotCanRecoveryCoordinatorInit(&state, &config, &modules));
    TEST_CHECK(!Step(&state, &remote, 1000u) && !Step(&state, &remote, 1001u));
    TEST_CHECK(!Step(&state, &remote, 1101u)); // 已达到稳定时间，正在等待新人工输入。
    TEST_CHECK(MessagePublisherInvalidate(application_publisher) == MESSAGE_OK); // 短暂撤销，两轮检查之间恢复。
    TEST_CHECK(!Step(&state, &remote, 1102u) && clears == 3u); // 新应用反馈不能沿用上一区间的稳定时间。
    TEST_CHECK(!Step(&state, &remote, 1103u) && !Step(&state, &remote, 1203u));
    invalidate_before_final_lock = 1u; // 本轮已读过合法反馈，最后一次复核前撤销。
    TEST_CHECK(!Step(&state, &remote, 1204u) && clears == 3u && !invalidate_before_final_lock);
    TEST_CHECK(!Step(&state, &remote, 1205u) && !Step(&state, &remote, 1305u));
    TEST_CHECK(Step(&state, &remote, 1306u) && clears == 4u);

    TEST_CHECK(RobotCanRecoveryCoordinatorInit(&state, &config, &modules));
    TEST_CHECK(!Step(&state, &remote, 1400u) && !Step(&state, &remote, 1401u));
    TEST_CHECK(!Step(&state, &remote, 1501u));
    PublishApplicationAt(1552u); // 上一份已自然过期，跳过正常的中间发布。
    TEST_CHECK(!Step(&state, &remote, 1552u) && clears == 4u); // 新样本的年龄为零，仍需重新等待。
    TEST_CHECK(!Step(&state, &remote, 1553u) && !Step(&state, &remote, 1653u));
    TEST_CHECK(Step(&state, &remote, 1654u) && clears == 5u);

    TEST_CHECK(RobotCanRecoveryCoordinatorInit(&state, &config, &modules));
    TEST_CHECK(!Step(&state, &remote, 1700u) && !Step(&state, &remote, 1701u));
    TEST_CHECK(!Step(&state, &remote, 1801u)); // 完成反馈稳定等待。
    final_clear_allowed = 0u;
    TEST_CHECK(!Step(&state, &remote, 1802u) && clears == 5u && irq_state == 0u); // 前置检查全通过，最终清除仍被拒绝。
    final_clear_allowed = 1u;
    ++sequence;
    tick = 1803u;
    uint8_t payload;
    MessageFreshness_t application;
    TEST_CHECK(MessageReadSnapshot(application_subscriber, &payload, sizeof(payload), tick, &application) == MESSAGE_OK);
    TEST_CHECK(!RobotCanRecoveryCoordinatorStep(&state, &remote, 0u, &application) && clears == 5u); // 保持同一人工样本，不能重试原许可。
    TEST_CHECK(!Step(&state, &remote, 1903u) && clears == 5u); // 稳定时长满足后仍等待新输入。
    TEST_CHECK(Step(&state, &remote, 1904u) && clears == 6u); // 本轮稳定后的新输入方可清除。
    return 0;
}
