/** @file commissioning_chassis_cmd.c
 * @brief 发布受限调试中的人工底盘目标；目标只使用底盘本体坐标。
 */
#include "commissioning_chassis_cmd.h"
#include "daemon.h"
#include "message_center.h"
#include "platform_irq.h"
#include "platform_sync.h"
#include "remote_safety.h"
#include "robot_can_recovery_coordinator.h"
#include "robot_control_feedback_runtime.h"
#include "robot_def.h"
#include "robot_manual_motion.h"
#include "robot_safety.h"
#include "std_cmd_remote.h"
#include "topics/robot_topics.h"
#include <limits.h>

static const RobotManualInputConfig_s *manual_input; // 启动后只读，保留车型映射。
static MessagePublisher_t *chassis_publisher; // 专用入口仅登记底盘命令。
static RemoteSafetyGestures_s gestures; // 人工手势计时只由整车任务修改。
static uint32_t source_generation; // 保存上次读到的切档代次，零也可以是有效代次。
static uint8_t source_seen; // 区分尚未读到任何来源。
static uint8_t manual_lease_published; // 已提交的人工来源在失效时需要撤销。

uint8_t CommissioningChassisCmdInit(const RobotManualInputConfig_s *input)
{
    if (chassis_publisher != NULL || input == NULL || input->max_dt_ms == 0u || input->max_dt_ms > INT32_MAX ||
        !StdCmdRemoteMapIsValid(input->map) ||
        input->map->remote_frame != STD_CMD_FRAME_CHASSIS || input->map->keyboard_frame != STD_CMD_FRAME_CHASSIS)
        return 0u; // 无法验证两个输入方向都基于底盘时不登记发布者。
    MessagePublisher_t *publisher = MessagePublisherRegister(ROBOT_TOPIC_CHASSIS_COMMAND, ROBOT_TOPIC_ENDPOINT_COMMAND);
    if (publisher == NULL) return 0u; // 话题未建立或重复登记时不能接受此模块。
    RemoteSafetyGesturesInit(&gestures);
    manual_input = input;
    chassis_publisher = publisher;
    return 1u;
}

// 撤销人工来源时更新安全代际，电机队列也会丢弃已计算的旧目标。
static void ClearManualCommand(void)
{
    if (manual_lease_published)
        (void)RobotSafetyPublishControlInputs(NULL); // 同一次失效只撤销一次已交付的来源。
    manual_lease_published = 0u;
    (void)MessagePublisherInvalidate(chassis_publisher); // 不把旧速度留给底盘任务继续使用。
}

void CommissioningChassisCmdTask(void)
{
    if (chassis_publisher == NULL) return; // 未完成登记不能进入周期处理。
    (void)DaemonProcessTaskEvents(DAEMON_CONSUMER_ROBOT_TASK, DAEMON_EVENT_MAX_DEPTH, NULL); // 先处理离线等状态事件。
    (void)RobotControlFeedbackPoll(NULL); // 底盘使用四轮自身反馈，姿态核心状态由安全管理独立检查。

    RemoteControlSnapshot_s remote = {0}; // 遥控与键盘状态取自同一次读取。
    RemoteControlGetSnapshot(&remote);
    const uint32_t now_ms = PlatformTimeNowMs(); // 使用与原始遥控帧同一时基。
    const uint8_t radio_fresh = (uint8_t)(remote.remote_online && remote.radio_sequence != 0u &&
        (uint32_t)(now_ms - remote.radio_source_time_ms) <= REMOTE_RADIO_LEASE_MS);
    RobotSafetySetRcOnline(radio_fresh); // 遥控离线在受限调试仍按人工急停处理。
    RemoteSafetyGesturesUpdate(&gestures, &remote.input, radio_fresh, now_ms, 1u); // 复用现有使能、失能和急停手势。

    const uint8_t switched = (uint8_t)(source_seen && remote.source_selection_generation != source_generation);
    source_generation = remote.source_selection_generation; // 即使切出去又切回，代次仍会变化。
    source_seen = 1u;
    StdCmdManualIntent_t intent; // 仅在适配成功后读取人工意图。
    RobotManualMotion_t motion; // 复用当前底盘消息的单位与左右约定。
    if (switched || StdCmdAdaptRemote(manual_input->map, &remote, now_ms, &intent) != STD_CMD_OK ||
        intent.frame != STD_CMD_FRAME_CHASSIS ||
        !RobotManualMotionProject(&intent, now_ms, 0u, manual_input->max_dt_ms, 0.0f, 0.0f, 0.0f, &motion))
    {
        ClearManualCommand(); // 切档或输入无效时，先撤销而不交付本轮目标。
        RobotSafetyUpdate(); // 即使没有运动输入也要推进急停和无效状态。
        remote.remote_online = 0u; // 恢复检查也不得把这一轮输入视为有效命令。
        (void)RobotCanRecoveryPoll(&remote, 1u, NULL); // 无有效人工来源时中断已开始的恢复等待。
        return; // 只在后续新帧有效时尝试接管。
    }

    const RobotSafetyControlInputs_t inputs = {
        .mode = ROBOT_CONTROL_MANUAL, // 此命令入口只会选择纯人工控制。
        .manual = {intent.source_time_ms, intent.max_age_ms, 1u}, // 保留无线或键盘的原始期限。
    };
    manual_lease_published = RobotSafetyPublishControlInputs(&inputs); // 命令不提交轮组反馈和运行许可。
    RobotSafetyUpdate(); // 由原有安全管理更新状态及恢复起点。
    if (RobotCanRecoveryPoll(&remote, 1u, NULL))
    {
        ClearManualCommand(); // 清除故障的这一轮仍禁止发布旧目标。
        RobotSafetyUpdate(); // 保留恢复后的新命令起点。
        return;
    }

    const PlatformIrqState_t saved = PlatformIrqSaveAndDisable(); // 最终来源复核与发布之间不插入接收中断。
    const uint32_t publish_ms = PlatformTimeNowMs(); // 按发布时刻再次检查原期限。
    RobotSafetySnapshot_t safety; // 取得即将写入命令的安全代际。
    RobotControlPermits_t permits;
    uint8_t policy_active = 0u; // 未启用受限策略时禁止使用旧整车许可。
    const uint8_t remote_current = RemoteControlRecoveryStillValid(&remote, publish_ms); // 复核实际来源与接收序号。
    if (!manual_lease_published || !remote_current ||
        !RobotSafetyGetSnapshot(&safety) ||
        !RobotSafetyGetOutputPermitsForGeneration(publish_ms, safety.control_generation, &permits, &policy_active) ||
        !policy_active ||
        permits.functions != ROBOT_CONTROL_CHASSIS || permits.chassis_source != ROBOT_CONTROL_SOURCE_MANUAL ||
        permits.fire_allowed)
    {
        if (!remote_current)
            ClearManualCommand(); // 切档或新帧发生在任务运行中，排队目标也需撤销。
        else
            (void)MessagePublisherInvalidate(chassis_publisher); // 当前未获许可时撤销消息。
        PlatformIrqRestore(saved);
        return;
    }

    const Chassis_Ctrl_Cmd_s command = {
        .vx = motion.vx_mm_s, // 前向速度，mm/s。
        .vy = motion.vy_right_mm_s, // 旧底盘消息以向右为正，mm/s。
        .wz = motion.wz_deg_s, // 手动绕自身上轴转动，deg/s。
        .chassis_mode = CHASSIS_NO_FOLLOW, // 只执行本体目标，不依赖方向跟随。
        .safety_generation = safety.control_generation, // 底盘及电机发送前还需复核该代际。
    };
    if (MessagePublishAtBounded(chassis_publisher, &command, intent.source_time_ms, intent.source_time_ms,
                               publish_ms, MESSAGE_CLOCK_LOCAL, intent.max_age_ms) != MESSAGE_OK)
        ClearManualCommand(); // 发布失败不复用上一轮已计算或排队的目标。
    PlatformIrqRestore(saved); // 恢复进入前的中断状态。
}
