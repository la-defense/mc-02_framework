/** @file commissioning_chassis_cmd_test.c
 * @brief 联测受限底盘命令、人工手势、安全许可及消息有效期。
 */
#include "commissioning_chassis_cmd.h"
#include "daemon.h"
#include "message_center.h"
#include "platform_irq.h"
#include "platform_sync.h"
#include "robot_can_recovery_coordinator.h"
#include "robot_control_feedback_runtime.h"
#include "robot_def.h"
#include "robot_safety.h"
#include "std_cmd_remote.h"
#include "test_check.h"
#include "topics/robot_topics.h"
#include <math.h>

static uint32_t tick;
static RemoteControlSnapshot_s radio;
static uint8_t wheel_feedback_available = 1u;
static uint8_t reject_remote_final_check;

uint32_t HAL_GetTick(void) { return tick; }

static uint8_t IrqRestored(void)
{
    const PlatformIrqState_t previous = PlatformIrqSaveAndDisable(); // 通过平台替身读取之前的屏蔽状态。
    PlatformIrqRestore(previous);
    return previous == 0u;
}

void BSP_ResetRecoverySetManualEstop(uint8_t latched) { (void)latched; }

void RemoteControlGetSnapshot(RemoteControlSnapshot_s *snapshot) { *snapshot = radio; }

uint8_t RemoteControlRecoveryStillValid(const RemoteControlSnapshot_s *snapshot, uint32_t now_ms)
{
    return !reject_remote_final_check && snapshot->radio_sequence == radio.radio_sequence &&
        (uint32_t)(now_ms - snapshot->radio_source_time_ms) <= REMOTE_RADIO_LEASE_MS; // 最终读取失败由独立场景注入。
}

DaemonStatus_t DaemonProcessTaskEvents(uint8_t consumer, uint8_t budget, uint8_t *processed)
{
    (void)consumer;
    (void)budget;
    (void)processed;
    return DAEMON_OK; // 该联测直接提供遥控和反馈变化。
}

uint8_t RobotControlFeedbackPoll(const MessageFreshness_t *pose)
{
    TEST_CHECK(pose == NULL); // 纯底盘无云台话题。
    const RobotControlLease_t feedback[3] = {
        {tick, 20u, 1u}, // 用本机测试时间模拟有效四轮反馈。
        {0}, // 本构建没有云台反馈。
        {0}, // 本构建没有发射反馈。
    };
    return RobotSafetyPublishVehicleFeedback(wheel_feedback_available ? ROBOT_CONTROL_CHASSIS : 0u, feedback);
}

uint8_t RobotCanRecoveryPoll(const RemoteControlSnapshot_s *remote, uint8_t keyboard_allowed,
                             const MessageFreshness_t *pose)
{
    (void)remote;
    (void)keyboard_allowed;
    TEST_CHECK(pose == NULL); // CAN 恢复本身由协调器的专项回归验证。
    return 0u;
}

static void NewRadio(uint32_t time_ms)
{
    tick = time_ms;
    ++radio.radio_sequence;
    radio.radio_source_time_ms = tick;
}

int main(void)
{
    tick = 1u;
    RobotSafetyInit();
    TEST_CHECK(RobotSafetyBindVehicleFeedback());
    const RobotControlPolicy_t policy = {
        .enabled_functions = ROBOT_CONTROL_CHASSIS, // 测试调用与启动配置使用同一功能。
        .configured = 1u, // 测试提供完整的人工输入。
        .formal_autonomous = 0u, // 无自主来源。
        .commissioning = 1u, // 只接受人工。
    };
    TEST_CHECK(RobotSafetyConfigureControl(&policy));
    RobotSafetySetImuValid(1u);
    RobotSafetySetCalibValid(1u);
    RobotSafetySetTaskHealthy(1u);

    TEST_CHECK(RobotTopicsInit());
    uint8_t topic_count = 0u;
    TEST_CHECK(MessageGetDiagnosticTopicCount(&topic_count) == MESSAGE_OK && topic_count == 2u);
    TEST_CHECK(MessageTopicGetDescriptor(ROBOT_TOPIC_GIMBAL_COMMAND, &(MessageTopicDescriptor_t){0}) != MESSAGE_OK);
    MessageSubscriber_t *subscriber = MessageSubscriberRegister(ROBOT_TOPIC_CHASSIS_COMMAND, ROBOT_TOPIC_ENDPOINT_CHASSIS);
    TEST_CHECK(subscriber != NULL);

    StdCmdRemoteMap_t map = {0};
    map.remote_map.max_age_ms = 15u; // 主机测试的无线输入短期限。
    map.keyboard_map.max_age_ms = 15u; // 键盘来源也有自己的短期限。
    map.remote_map.allowed_frames = 1u << STD_CMD_FRAME_CHASSIS; // 测试只接受本体坐标。
    map.keyboard_map.allowed_frames = 1u << STD_CMD_FRAME_CHASSIS; // 键盘与遥控采用同一参照。
    map.remote_frame = STD_CMD_FRAME_BIG_YAW; // 先验证缺少云台参照时拒绝启动。
    map.keyboard_frame = STD_CMD_FRAME_CHASSIS;
    for (unsigned axis = 0u; axis < STD_CMD_AXIS_COUNT; ++axis)
    {
        map.remote_map.axis[axis] = (StdCmdAxisMap_t){660.0f, 0.001f, 1.0f}; // 摇杆方向和范围仅供测试。
        map.keyboard_map.axis[axis] = (StdCmdAxisMap_t){1.0f, 0.1f, 1.0f}; // 测试键盘比例与范围。
    }
    map.remote_axis[STD_CMD_VX] = STD_CMD_RC_RIGHT_Y; // 测试右摇杆控制前向。
    map.keyboard_positive[STD_CMD_VX] = (uint16_t)(1u << Key_W); // 测试 W 键控制前向。
    const RobotManualInputConfig_s input = {.map = &map, .max_dt_ms = 20u};
    TEST_CHECK(!CommissioningChassisCmdInit(&input));
    map.remote_frame = STD_CMD_FRAME_CHASSIS;
    TEST_CHECK(CommissioningChassisCmdInit(&input));

    radio.remote_online = 1u;
    radio.input.rc.switch_left = RC_SW_DOWN;
    radio.input.rc.switch_right = RC_SW_MID;
    radio.input.rc.rocker_r1 = 100;
    NewRadio(2u);
    CommissioningChassisCmdTask(); // 新鲜输入本身不得使能。
    Chassis_Ctrl_Cmd_s command;
    MessageFreshness_t freshness;
    TEST_CHECK(MessageReadSnapshot(subscriber, &command, sizeof(command), tick, NULL) != MESSAGE_OK);
    radio.input.rc.switch_right = RC_SW_DOWN; // 用生产手势进入人工使能。
    NewRadio(10u);
    CommissioningChassisCmdTask();
    NewRadio(1010u);
    CommissioningChassisCmdTask();
    TEST_CHECK(MessageReadSnapshot(subscriber, &command, sizeof(command), tick, NULL) != MESSAGE_OK); // 使能同刻的输入仍旧。
    NewRadio(1011u);
    CommissioningChassisCmdTask();
    TEST_CHECK(MessageReadSnapshot(subscriber, &command, sizeof(command), tick, &freshness) == MESSAGE_OK);
    TEST_CHECK(command.chassis_mode == CHASSIS_NO_FOLLOW && fabsf(command.vx - 100.0f) < 0.01f);
    TEST_CHECK(freshness.source_time_ms == 1011u && freshness.source_max_age_ms == 15u);

    tick = 1020u;
    CommissioningChassisCmdTask(); // 重发也保留原始来源期限。
    TEST_CHECK(MessageReadSnapshot(subscriber, &command, sizeof(command), tick, &freshness) == MESSAGE_OK &&
        freshness.source_time_ms == 1011u && freshness.source_max_age_ms == 15u);
    tick = 1027u;
    CommissioningChassisCmdTask();
    TEST_CHECK(MessageReadSnapshot(subscriber, &command, sizeof(command), tick, NULL) != MESSAGE_OK);

    NewRadio(1028u);
    CommissioningChassisCmdTask(); // 新无线数据允许恢复。
    TEST_CHECK(MessageReadSnapshot(subscriber, &command, sizeof(command), tick, NULL) == MESSAGE_OK);
    ++radio.source_selection_generation;
    radio.input.rc.switch_left = RC_SW_UP;
    radio.pc_valid = 1u;
    radio.input.key[KEY_PRESS].keys = (uint16_t)(1u << Key_W);
    radio.pc_sample_sequence = 1u;
    radio.pc_source_time_ms = 1029u;
    NewRadio(1029u);
    CommissioningChassisCmdTask(); // 切档这一轮先撤销，不沿用旧目标。
    TEST_CHECK(MessageReadSnapshot(subscriber, &command, sizeof(command), tick, NULL) != MESSAGE_OK);
    ++radio.pc_sample_sequence;
    radio.pc_source_time_ms = 1030u;
    NewRadio(1030u);
    CommissioningChassisCmdTask();
    TEST_CHECK(MessageReadSnapshot(subscriber, &command, sizeof(command), tick, NULL) == MESSAGE_OK);
    const uint32_t old_generation = command.safety_generation; // 发送方不能重用检查失败前的代际。
    reject_remote_final_check = 1u;
    NewRadio(1031u);
    CommissioningChassisCmdTask();
    TEST_CHECK(MessageReadSnapshot(subscriber, &command, sizeof(command), tick, NULL) != MESSAGE_OK);
    TEST_CHECK(!RobotSafetyIsReadyForGeneration(tick, old_generation) && IrqRestored());
    reject_remote_final_check = 0u;

    ++radio.pc_sample_sequence;
    radio.pc_source_time_ms = 1032u;
    NewRadio(1032u);
    CommissioningChassisCmdTask();
    TEST_CHECK(MessageReadSnapshot(subscriber, &command, sizeof(command), tick, NULL) == MESSAGE_OK);
    wheel_feedback_available = 0u;
    NewRadio(1033u);
    CommissioningChassisCmdTask(); // 仍有人工输入，缺少轮组反馈也必须禁止。
    TEST_CHECK(MessageReadSnapshot(subscriber, &command, sizeof(command), tick, NULL) != MESSAGE_OK);
    radio.remote_online = 0u;
    NewRadio(1034u);
    CommissioningChassisCmdTask(); // 无线离线锁存急停。
    TEST_CHECK(EstopIsLatched() && EstopGetReason() == ESTOP_REASON_RC_OFFLINE && IrqRestored());
    return 0;
}
