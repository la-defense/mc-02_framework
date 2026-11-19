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
#include <math.h>

static const RobotManualInputConfig_s *manual_input; // 启动后只读，保留车型映射。
static MessagePublisher_t *chassis_publisher; // 专用入口仅登记底盘命令。
static RemoteSafetyGestures_s gestures; // 人工手势计时只由整车任务修改。
static uint32_t source_generation; // 保存上次读到的切档代次，零也可以是有效代次。
static uint8_t source_seen; // 区分尚未读到任何来源。
static uint8_t manual_lease_published; // 已提交的人工来源在失效时需要撤销。

typedef struct {
    ChassisTwist_t twist; // 上次成功发布的底盘本体速度，使用 m/s、rad/s。
    uint32_t published_ms; // 上次发布时刻，用于计算允许的速度变化量。
    uint32_t source_time_ms; // 来源原始时刻，不能用本轮新输入延长。
    uint32_t source_max_age_ms; // 上次目标自身的来源期限。
    uint32_t safety_generation; // 跨失能或恢复边沿时丢弃这份速度。
    uint8_t valid; // 只有成功发布过当前控制过程的目标才置位。
} PublishedChassisMotion_t;

static PublishedChassisMotion_t last_motion; // 只由整车任务维护。
static uint32_t chassis_topic_max_age_ms; // 话题本身的有效期独立于原始输入有效期。

uint8_t CommissioningChassisCmdInit(const RobotManualInputConfig_s *input)
{
    if (chassis_publisher != NULL || input == NULL || input->max_dt_ms == 0u || input->max_dt_ms > INT32_MAX ||
        !StdCmdRemoteMapIsValid(input->map) ||
        input->map->remote_frame != STD_CMD_FRAME_CHASSIS || input->map->keyboard_frame != STD_CMD_FRAME_CHASSIS)
        return 0u; // 无法验证两个输入方向都基于底盘时不登记发布者。
    if (!isfinite(input->commissioning_linear_accel_m_s2) || input->commissioning_linear_accel_m_s2 <= 0.0f ||
        !isfinite(input->commissioning_angular_accel_rad_s2) || input->commissioning_angular_accel_rad_s2 <= 0.0f)
        return 0u; // 平移和转动都必须有明确的正上限。
    MessageTopicDescriptor_t topic; // 取得底盘话题自身的有效期。
    if (MessageTopicGetDescriptor(ROBOT_TOPIC_CHASSIS_COMMAND, &topic) != MESSAGE_OK ||
        topic.source_clock != MESSAGE_CLOCK_LOCAL || topic.max_age_ms == 0u || topic.max_age_ms > INT32_MAX)
        return 0u; // 无法校验旧目标期限时不登记发布者。
    MessagePublisher_t *publisher = MessagePublisherRegister(ROBOT_TOPIC_CHASSIS_COMMAND, ROBOT_TOPIC_ENDPOINT_COMMAND);
    if (publisher == NULL) return 0u; // 话题未建立或重复登记时不能接受此模块。
    RemoteSafetyGesturesInit(&gestures);
    manual_input = input;
    chassis_topic_max_age_ms = topic.max_age_ms;
    chassis_publisher = publisher;
    return 1u;
}

// 撤销人工来源时更新安全代际，电机队列也会丢弃已计算的旧目标。
static void ClearManualCommand(void)
{
    if (manual_lease_published)
        (void)RobotSafetyPublishControlInputs(NULL); // 同一次失效只撤销一次已交付的来源。
    manual_lease_published = 0u;
    last_motion.valid = 0u; // 下一条获准的人工命令从零开始。
    (void)MessagePublisherInvalidate(chassis_publisher); // 不把旧速度留给底盘任务继续使用。
}

// 同时核对之前目标的原始输入期限和话题期限。
static uint8_t LastMotionIsFresh(uint32_t now_ms)
{
    return (uint32_t)(now_ms - last_motion.source_time_ms) <= last_motion.source_max_age_ms &&
           (uint32_t)(now_ms - last_motion.published_ms) <= chassis_topic_max_age_ms;
}

// 只修改三个底盘速度分量，保留本次输入的原始时刻和期限。
static uint8_t LimitMotion(StdCmdManualIntent_t *intent, uint32_t now_ms,
                           uint32_t generation, uint8_t *uses_previous)
{
    *uses_previous = 0u; // 默认从零开始，旧速度需要再次证明可用。
    if (!last_motion.valid || last_motion.safety_generation != generation)
    {
        intent->axis[STD_CMD_VX] = 0.0f; // 首轮建立零速度基准。
        intent->axis[STD_CMD_VY] = 0.0f; // 横移和前进同时清零。
        intent->axis[STD_CMD_WZ] = 0.0f; // 转动目标也从零开始。
        return 1u;
    }

    const uint32_t dt_ms = now_ms - last_motion.published_ms; // 无符号减法跨越正常的毫秒回绕。
    if (dt_ms > manual_input->max_dt_ms)
        return 0u; // 时钟异常或任务间隔过长时撤销许可，不能积累一次大步长。
    if (!LastMotionIsFresh(now_ms))
        return 0u; // 旧目标已经失效，必须以新的来源重新建立零速度基准。
    *uses_previous = 1u; // 发布前还要复核这一份旧目标是否仍可使用。

    const float dt_s = (float)dt_ms * 0.001f; // 配置的变化率以秒为单位。
    const float dx = intent->axis[STD_CMD_VX] - last_motion.twist.vx_m_s; // 本体前向速度需要改变的量。
    const float dy = intent->axis[STD_CMD_VY] - last_motion.twist.vy_m_s; // 本体左向速度需要改变的量。
    const float dw = intent->axis[STD_CMD_WZ] - last_motion.twist.wz_rad_s; // 需要改变的绕上轴角速度。
    const float distance = hypotf(dx, dy); // 两个平移分量合计只获得一个步长。
    const float linear_step = manual_input->commissioning_linear_accel_m_s2 * dt_s; // 本次最多改变多少 m/s。
    const float angular_step = manual_input->commissioning_angular_accel_rad_s2 * dt_s; // 本次最多改变多少 rad/s。
    if (!isfinite(distance) || !isfinite(dw) || !isfinite(linear_step) || !isfinite(angular_step))
        return 0u; // 即使配置和输入分别有限，相减或换算仍可能溢出。
    if (distance > linear_step)
    {
        const float scale = linear_step / distance; // 按同一比例沿目标方向改变前向和左向速度。
        intent->axis[STD_CMD_VX] = last_motion.twist.vx_m_s + dx * scale;
        intent->axis[STD_CMD_VY] = last_motion.twist.vy_m_s + dy * scale;
    }
    const float bounded_dw = fminf(fmaxf(dw, -angular_step), angular_step); // 转动使用独立的双向步长。
    if (bounded_dw != dw)
        intent->axis[STD_CMD_WZ] = last_motion.twist.wz_rad_s + bounded_dw; // 只移动到本轮允许的范围内。
    return 1u;
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
    if (switched || StdCmdAdaptRemote(manual_input->map, &remote, now_ms, &intent) != STD_CMD_OK ||
        intent.frame != STD_CMD_FRAME_CHASSIS)
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

    RobotSafetySnapshot_t safety; // 将本次计算对应的安全代际固定下来。
    const uint32_t compute_ms = PlatformTimeNowMs(); // 数学计算在中断保护之外执行。
    uint8_t uses_previous = 0u; // 发布前检查限速使用的历史目标。
    RobotManualMotion_t motion; // 将限速后的 SI 目标换算成现有底盘消息。
    if (!RobotSafetyGetSnapshot(&safety) ||
        !LimitMotion(&intent, compute_ms, safety.control_generation, &uses_previous) ||
        !RobotManualMotionProject(&intent, compute_ms, 0u, manual_input->max_dt_ms, 0.0f, 0.0f, 0.0f, &motion))
    {
        ClearManualCommand(); // 间隔错误或单位换算失败时直接撤销，不沿变化率缓慢减速。
        RobotSafetyUpdate(); // 当场记录接管起点，下一轮只接受此后到达的新输入。
        return;
    }

    const PlatformIrqState_t saved = PlatformIrqSaveAndDisable(); // 最终来源复核与发布之间不插入接收中断。
    const uint32_t publish_ms = PlatformTimeNowMs(); // 按发布时刻再次检查原期限。
    const uint8_t timing_current = (uint8_t)((uint32_t)(publish_ms - compute_ms) <= manual_input->max_dt_ms &&
        (!uses_previous || ((uint32_t)(publish_ms - last_motion.published_ms) <= manual_input->max_dt_ms &&
                            LastMotionIsFresh(publish_ms)))); // 计算期间旧目标也可能到期。
    RobotControlPermits_t permits;
    uint8_t policy_active = 0u; // 未启用受限策略时禁止使用旧整车许可。
    const uint8_t remote_current = RemoteControlRecoveryStillValid(&remote, publish_ms); // 复核实际来源与接收序号。
    if (!manual_lease_published || !remote_current || !timing_current ||
        !RobotSafetyGetOutputPermitsForGeneration(publish_ms, safety.control_generation, &permits, &policy_active) ||
        !policy_active ||
        permits.functions != ROBOT_CONTROL_CHASSIS || permits.chassis_source != ROBOT_CONTROL_SOURCE_MANUAL ||
        permits.fire_allowed)
    {
        const uint8_t revoke = (uint8_t)(!remote_current || !timing_current); // 只有来源或时间错误需主动撤销来源。
        if (revoke)
            ClearManualCommand(); // 来源变化或等待超限时也撤销已排队目标。
        else
        {
            last_motion.valid = 0u; // 没有许可就清除下次计算的速度基准。
            (void)MessagePublisherInvalidate(chassis_publisher); // 当前未获许可时撤销消息。
        }
        PlatformIrqRestore(saved);
        if (revoke) RobotSafetyUpdate(); // 恢复中断后再推进状态和记录。
        return;
    }

    const Chassis_Ctrl_Cmd_s command = {
        .vx = motion.vx_mm_s, // 前向速度，mm/s。
        .vy = motion.vy_right_mm_s, // 旧底盘消息以向右为正，mm/s。
        .wz = motion.wz_deg_s, // 手动绕自身上轴转动，deg/s。
        .chassis_mode = CHASSIS_NO_FOLLOW, // 只执行本体目标，不依赖方向跟随。
        .safety_generation = safety.control_generation, // 底盘及电机发送前还需复核该代际。
    };
    const uint8_t published = (uint8_t)(MessagePublishAtBounded(chassis_publisher, &command, intent.source_time_ms,
        intent.source_time_ms, publish_ms, MESSAGE_CLOCK_LOCAL, intent.max_age_ms) == MESSAGE_OK);
    if (!published)
        ClearManualCommand(); // 发布失败不复用上一轮已计算或排队的目标。
    else
    {
        last_motion.twist = (ChassisTwist_t){intent.axis[STD_CMD_VX], intent.axis[STD_CMD_VY],
                                           intent.axis[STD_CMD_WZ]}; // 保存真正交付的 SI 速度。
        last_motion.published_ms = publish_ms; // 下次只计算自这条消息发布以来的时间。
        last_motion.source_time_ms = intent.source_time_ms; // 重复发布不能延长来源时间。
        last_motion.source_max_age_ms = intent.max_age_ms; // 也保留来源原有期限。
        last_motion.safety_generation = safety.control_generation; // 恢复后不能沿用这个速度。
        last_motion.valid = 1u; // 已成功发布，可以作为下一轮基准。
    }
    PlatformIrqRestore(saved); // 恢复进入前的中断状态。
    if (!published) RobotSafetyUpdate(); // 记录发布失败后的安全接管起点。
}
