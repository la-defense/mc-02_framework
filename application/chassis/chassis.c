/**
 * @file chassis.c
 * @author NeoZeng neozng1@hnu.edu.cn
 * @brief 接收 robot_cmd 的控制命令，计算轮速目标并读取本地底盘反馈。
 *        旧命令的横向为右，轮组计算和反馈使用前左上坐标。
 *
 * @version 0.1
 * @date 2022-12-04
 *
 * @copyright Copyright (c) 2022
 *
 */

#include "chassis.h"
#include "chassis_motion.h"
#include "platform_sync.h"
#include "robot_def.h"
#include "config.h"
#include "config/build_selection.h"
#include "dji_motor.h"
#if MC02_FEATURE_SUPERCAP
#include "chassis_supercap_legacy.h"
#endif
#include "message_center.h"
#include "topics/robot_topics.h"
#include "referee_task.h"
#include "bsp_log.h"
#include "general_def.h"
#ifdef CHASSIS_BOARD
#include "robot_safety.h"
#endif
#include <string.h>

/* 底盘应用包含的模块和信息存储,底盘是单例模式,因此不需要为底盘建立单独的结构体 */
#ifdef CHASSIS_BOARD // 如果是底盘板,使用板载IMU获取底盘转动角速度
#include "ins_task.h"
attitude_t *Chassis_IMU_data;
#endif // CHASSIS_BOARD
#if defined(ONE_BOARD) || defined(CHASSIS_BOARD)
static MessagePublisher_t *chassis_pub;              // 用于发布底盘的数据
static MessageSubscriber_t *chassis_sub;             // 用于订阅底盘的控制命令
#endif
static Chassis_Ctrl_Cmd_s chassis_cmd_recv;         // 底盘接收到的控制命令
#if defined(ONE_BOARD) || defined(CHASSIS_BOARD)
static Chassis_Upload_Data_s chassis_feedback_data; // 底盘回传的反馈数据
#endif

static const Chassis_Config_s *chassis_config;
static DJIMotorInstance *motor_lf, *motor_rf, *motor_lb, *motor_rb; // left right forward back
static uint8_t chassis_rollback_failed; // 记录启动时无法撤销的设备资源。
static ChassisVelocityFeedback_t chassis_velocity; // 底盘任务更新，其他任务只读取完整副本。
static uint8_t chassis_velocity_valid; // 时间戳零也可有效，单独记录。
static ChassisOdometryFeedback_t chassis_odometry; // 与速度在同一临界区交付。
static uint8_t chassis_odometry_valid;
#if !defined(MC02_CONFIG_GENERATED) || MC02_CHASSIS_IS_OMNI
typedef struct {
    uint32_t time_ms; // 轮速的原始接收时刻，允许毫秒回绕。
    uint32_t sequence; // 该轮本次反馈序号，零表示没有样本。
} ChassisWheelObservation_t;

static ChassisOdometryFeedback_t odometry_anchor; // 只由底盘任务更新，启动阶段负责清理。
static ChassisTwist_t odometry_anchor_twist; // 上一个不同来源时刻对应的底盘自身速度。
static ChassisWheelObservation_t odometry_anchor_wheels[CHASSIS_WHEEL_COUNT]; // 上次积分使用的四轮原始反馈身份。
static uint8_t odometry_anchor_known; // 时间为零也能成为有效样本。
static uint8_t odometry_integrating; // 收到无效数据后等待下一个新样本开启新段。
#endif

// 在同一个短临界区交付速度、里程计和各自有效位。
static void PublishMotionFeedback(const ChassisVelocityFeedback_t *velocity, const ChassisOdometryFeedback_t *odometry)
{
    PlatformTaskCriticalEnter(); // 计算已完成，此处只复制数据。
    chassis_velocity = velocity != NULL ? *velocity : (ChassisVelocityFeedback_t){0}; // 无效时一起清除三个分量。
    chassis_velocity_valid = (uint8_t)(velocity != NULL); // 读者不会误用已清除的值。
    chassis_odometry = odometry != NULL ? *odometry : (ChassisOdometryFeedback_t){0}; // 断流时撤销已发布位姿。
    chassis_odometry_valid = (uint8_t)(odometry != NULL); // 单独标记，关闭里程计不影响速度。
    PlatformTaskCriticalExit(); // 读者现在可取得完整副本。
}

// 底盘尚未运行时调用；保留段号，使重启底盘模块后的新段可以识别。
static void ResetMotionFeedback(void)
{
#if !defined(MC02_CONFIG_GENERATED) || MC02_CHASSIS_IS_OMNI
    odometry_anchor_known = 0u; // 初始化前的时间不再用于比较。
    odometry_integrating = 0u; // 下一个可信样本从零开始。
#endif
    PublishMotionFeedback(NULL, NULL);
}

uint8_t ChassisInitRollbackFailed(void)
{
    return chassis_rollback_failed; // 启动协调任务读取最近一次回退结果。
}
#ifdef CHASSIS_BOARD
static uint8_t chassis_command_rearm_required = 1u;
static uint8_t chassis_rearm_sequence_valid = 0u;
static uint8_t chassis_rearm_tracking = 0u;
static uint8_t chassis_inhibit_generation_valid = 0u;
static uint32_t chassis_rearm_sequence = 0u;
static uint32_t chassis_rearm_after_ms = 0u;
static uint32_t chassis_inhibit_generation = 0u;
#endif
#if defined(ONE_BOARD) || defined(CHASSIS_BOARD)
static uint32_t chassis_command_max_age_ms = 0u;
#endif

uint8_t ChassisInit(void)
{
    ResetMotionFeedback(); // 重新初始化时清除旧轮组估计及位姿。
    chassis_rollback_failed = 0u; // 本次初始化从无回退故障开始。
    chassis_config = ChassisConfigFeed();
    if (chassis_config == NULL || !ChassisOdometryGapIsValid(chassis_config->odometry_max_gap_ms,
                                                            chassis_config->wheel_type == OMNI_WHEEL))
        return 0u;
    if (!RobotTopicsInit())
        return 0u;
    DJIMotorInstance *pending[4] = {NULL}; // 申请完成之前不发布轮电机实例。
#if MC02_FEATURE_SUPERCAP
    uint8_t pending_cap = 0u; // 标记本次底盘启动是否取得旧电容实例。
#endif
#ifdef CHASSIS_BOARD
    attitude_t *pending_imu = INS_GetStorage(); // 双板底盘需要预先取得姿态存储。
    if (pending_imu == NULL) return 0u; // 没有存储时不注册设备。
#endif
#if defined(ONE_BOARD) || defined(CHASSIS_BOARD)
    MessageTopicDescriptor_t descriptor = {0}; // 在登记端点之前核对命令话题期限。
    if (MessageTopicGetDescriptor(ROBOT_TOPIC_CHASSIS_COMMAND, &descriptor) != MESSAGE_OK ||
        descriptor.max_age_ms == 0u || descriptor.max_age_ms > INT32_MAX)
        return 0u; // 话题期限必须允许用 32 位本机计数比较。
#if defined(ONE_BOARD)
    if (descriptor.source_clock != MESSAGE_CLOCK_LOCAL) return 0u; // 单板截止点只采用同一个本机时钟。
#endif
#endif

#if defined(MC02_CONFIG_GENERATED)
    RefereeStatus_t referee_status = REFEREE_OK;
#else
    RefereeStatus_t referee_status = RefereeInit(PLATFORM_UART_1, NULL);
#if MC02_ENABLE_REFEREE_UI
    // UI只读取车型布局和裁判快照；绑定失败只停用显示，不撤销裁判接收。
    UIStatus_t ui_status = UITaskInit(RefereeUIConfigFeed());
    if (ui_status != UI_OK) BSP_LOG_EVENT_1(EV_APPLICATION_CHASSIS_CHASSIS_C_01, BSP_LOG_FIELD_I32((int)ui_status));
#endif
#endif

    if (referee_status != REFEREE_OK) return 0u; // 旧主机组合的裁判初始化失败时不申请电机。
    Motor_Init_Config_s chassis_motor_config = chassis_config->chassis_motor_config; // 复制公共电机参数。
    chassis_motor_config.output_role = MOTOR_OUTPUT_CHASSIS; // 四个轮电机使用底盘功能许可。
    for (uint8_t i = 0u; i < CHASSIS_WHEEL_COUNT; ++i) // 按左前、右前、左后、右后的固定轮序登记。
    {
        const ChassisWheelMotorConfig_t *wheel = &chassis_config->wheel_motors[i]; // 同一轮的编号和方向一起读取。
        chassis_motor_config.can_init_config.tx_id = wheel->device_id; // 配置的是 DJI 设备编号，由驱动换算帧 ID。
        chassis_motor_config.output_axis.direction = wheel->direction; // 由驱动在目标和反馈两处使用该安装方向。
        pending[i] = DJIMotorInit(&chassis_motor_config); // 暂存本次取得的实例。
        if (pending[i] == NULL) goto rollback; // 任意轮电机失败时撤销此前实例。
    }
#if MC02_FEATURE_SUPERCAP
    pending_cap = ChassisLegacySuperCapInit(); // 普通供电回退未实现前仍需检查旧电容。
    if (!pending_cap) goto rollback; // 电容失败时撤销四台轮电机。
#endif

#if defined(ONE_BOARD) || defined(CHASSIS_BOARD)
    MessageSubscriber_t *pending_sub = NULL; // 暂存底盘命令订阅句柄。
    MessagePublisher_t *pending_pub = NULL; // 暂存底盘反馈发布句柄。
    if (MessageEndpointPairRegister(ROBOT_TOPIC_CHASSIS_FEEDBACK, ROBOT_TOPIC_CHASSIS_COMMAND,
                                    ROBOT_TOPIC_ENDPOINT_CHASSIS, &pending_pub, &pending_sub) != MESSAGE_OK)
        goto rollback; // 两个话题任一不足时不新增端点。
    chassis_sub = pending_sub; // 成功后发布命令订阅句柄。
    chassis_pub = pending_pub; // 同时发布反馈句柄。
    chassis_command_max_age_ms = descriptor.max_age_ms; // 使用事先检查的命令期限。
#endif
#ifdef CHASSIS_BOARD
    Chassis_IMU_data = pending_imu; // 发布已经确认存在的姿态存储。
#endif
    motor_lf = pending[0]; // 全部成功后发布左前轮。
    motor_rf = pending[1]; // 发布右前轮。
    motor_lb = pending[2]; // 发布左后轮。
    motor_rb = pending[3]; // 发布右后轮。
    return 1u; // 启动计划可接受底盘模块。

rollback:
#if MC02_FEATURE_SUPERCAP
    if (pending_cap && !ChassisLegacySuperCapRollbackStartup())
        chassis_rollback_failed = 1u; // 电容无法撤销时必须升级为整车故障。
#endif
    for (uint8_t i = 4u; i > 0u; --i)
        if (pending[i - 1u] != NULL && !DJIMotorRollbackStartup(pending[i - 1u]))
            chassis_rollback_failed = 1u; // 按申请逆序尽力撤销所有电机。
    return 0u; // 失败时不交付底盘模块。
}

uint8_t ChassisReleaseStartupDevices(void)
{
    ResetMotionFeedback(); // 资源撤销后不能再交付旧轮速或位姿。
    uint8_t released = 1u; // 汇总所有设备的撤销结果。
#if MC02_FEATURE_SUPERCAP
    if (!ChassisLegacySuperCapRollbackStartup()) released = 0u; // 旧电容由底盘持有，先撤销它。
#endif
    DJIMotorInstance **motors[] = {&motor_rb, &motor_lb, &motor_rf, &motor_lf}; // 按登记逆序处理四个轮电机。
    for (unsigned index = 0u; index < 4u; ++index) // 只访问本底盘持有的实例。
    {
        if (*motors[index] != NULL && !DJIMotorRollbackStartup(*motors[index])) released = 0u; // 单个失败后仍继续清理。
        *motors[index] = NULL; // 撤销失败也可能已释放软件实例，不再保留旧地址。
    }
    return released; // 底层无法安全清理时禁止启动后续任务。
}

void ChassisStopAndClear(void)
{
    DJIMotorStop(motor_lf);
    DJIMotorStop(motor_rf);
    DJIMotorStop(motor_lb);
    DJIMotorStop(motor_rb);
    DJIMotorSetRef(motor_lf, 0.0f);
    DJIMotorSetRef(motor_rf, 0.0f);
    DJIMotorSetRef(motor_lb, 0.0f);
    DJIMotorSetRef(motor_rb, 0.0f);
}

#ifdef CHASSIS_BOARD
static uint8_t ChassisMotorFeedbackIsHealthy(void)
{
    DJIMotorInstance *motors[4] = {motor_lf, motor_rf, motor_lb, motor_rb};
    uint8_t healthy = 1u;
    for (size_t i = 0u; i < sizeof(motors) / sizeof(motors[0]); ++i)
    {
        // 每个电机读取前重新取时，避免遍历期间跨毫秒后沿用较早时刻。
        const uint32_t now_ms = PlatformTimeNowMs();
        Motor_Feedback_Snapshot_s feedback = {0};
        if (DJIMotorGetFeedbackSnapshot(motors[i], now_ms, &feedback) != MOTOR_FEEDBACK_OK ||
            feedback.speed.valid == 0u || feedback.current.valid == 0u)
            healthy = 0u;
    }
    if (healthy != 0u)
        RobotFaultClear(ROBOT_FAULT_MOTOR_OFFLINE);
    else
        RobotFaultReport(ROBOT_FAULT_MOTOR_OFFLINE);
    return healthy;
}
#endif

uint8_t Chassis_GetVelocityFeedback(ChassisVelocityFeedback_t *output)
{
    if (output == NULL) return 0u;
    PlatformTaskCriticalEnter(); // 速度、来源时间和有效位来自同一次发布。
    const ChassisVelocityFeedback_t velocity = chassis_velocity;
    const uint8_t available = chassis_velocity_valid;
    PlatformTaskCriticalExit();
    if (!available || (uint32_t)(PlatformTimeNowMs() - velocity.source_time_ms) > DJI_FEEDBACK_MAX_AGE_MS)
    {
        *output = (ChassisVelocityFeedback_t){0}; // 发布者没有再运行时，读者也不能延用旧值。
        return 0u;
    }
    *output = velocity;
    return 1u;
}

uint8_t Chassis_GetOdometry(ChassisOdometryFeedback_t *output)
{
    if (output == NULL) return 0u;
    PlatformTaskCriticalEnter(); // 位姿、来源时间和段号必须取自同一份结果。
    const ChassisOdometryFeedback_t odometry = chassis_odometry;
    const uint8_t available = chassis_odometry_valid;
    PlatformTaskCriticalExit();
    if (!available || (uint32_t)(PlatformTimeNowMs() - odometry.source_time_ms) > DJI_FEEDBACK_MAX_AGE_MS)
    {
        *output = (ChassisOdometryFeedback_t){0}; // 底盘任务停更时也按轮速的原期限失效。
        return 0u;
    }
    *output = odometry;
    return 1u;
}

#if !defined(MC02_CONFIG_GENERATED) || MC02_CHASSIS_IS_OMNI
// 取四个已经换算到输出轴的轮速；不足四份时整个底盘速度无效。
static uint8_t EstimateOmniVelocity(ChassisVelocityFeedback_t *velocity,
                                    ChassisWheelObservation_t wheels[CHASSIS_WHEEL_COUNT])
{
    DJIMotorInstance *const motors[CHASSIS_WHEEL_COUNT] = {motor_lf, motor_rf, motor_lb, motor_rb}; // 统一轮序。
    ChassisWheelSpeeds_t speeds;
    for (unsigned i = 0u; i < CHASSIS_WHEEL_COUNT; ++i)
    {
        Motor_Feedback_Snapshot_s feedback = {0};
        if (DJIMotorGetFeedbackSnapshot(motors[i], PlatformTimeNowMs(), &feedback) != MOTOR_FEEDBACK_OK ||
            feedback.speed.valid != 1u || feedback.speed.unit != MOTOR_UNIT_RAD_S ||
            feedback.speed.reference != MOTOR_REFERENCE_OUTPUT_AXIS)
            return 0u; // 未接收、过期或错误单位的轮速均不能用来估算。
        speeds.rad_s[i] = feedback.speed.value; // 驱动已按安装方向和减速比换算，不重复处理符号。
        wheels[i].time_ms = feedback.speed.source_time_ms; // 记录原始接收时间，不以本轮执行时间替代。
        wheels[i].sequence = feedback.speed.sequence; // 每轮单独保留驱动报告的原始反馈身份。
    }
    if (OmniEstimate(&speeds, &chassis_config->omni_geometry, &velocity->twist) != CHASSIS_KINEMATICS_OK)
        return 0u; // 几何退化或数值异常时不发布部分结果。
    const uint32_t now = PlatformTimeNowMs(); // 计算后检查最早轮速是否依然有效。
    uint32_t oldest_age = 0u;
    for (unsigned i = 0u; i < CHASSIS_WHEEL_COUNT; ++i)
    {
        const uint32_t age = now - wheels[i].time_ms; // 无符号差值处理本机毫秒回绕。
        if (age > DJI_FEEDBACK_MAX_AGE_MS) return 0u; // 错误的未来时间也无法通过。
        if (age > oldest_age) oldest_age = age; // 选择剩余有效时间最短的样本。
    }
    velocity->source_time_ms = now - oldest_age; // 保留这个样本的原始时刻。
    return 1u;
}

// 只用不同来源时刻的有效速度积分；计算期间不占用发布临界区。
static const ChassisOdometryFeedback_t *UpdateOdometry(const ChassisVelocityFeedback_t *velocity,
    const ChassisWheelObservation_t wheels[CHASSIS_WHEEL_COUNT])
{
    if (velocity == NULL || chassis_config->odometry_max_gap_ms == 0u)
    {
        odometry_integrating = 0u; // 丢失任一轮后不能用旧位姿续算。
        return NULL; // 关闭里程计也不能交付位姿。
    }
    const uint32_t now = PlatformTimeNowMs(); // 计算期间跨越期限时，以当前时间为准。
    if (now - velocity->source_time_ms > DJI_FEEDBACK_MAX_AGE_MS)
    {
        odometry_integrating = 0u; // 本轮速度在积分前已经到期。
        return NULL;
    }
    const uint8_t anchor_recent = odometry_anchor_known &&
        now - odometry_anchor.source_time_ms <= DJI_FEEDBACK_MAX_AGE_MS; // 来源失效后只允许重新起算。
    uint8_t all_new = 1u; // 只有四轮都离开上次积分使用的样本，才推进位姿。
    for (unsigned i = 0u; i < CHASSIS_WHEEL_COUNT; ++i)
    {
        const uint32_t sequence_step = wheels[i].sequence - odometry_anchor_wheels[i].sequence; // 允许非零序号跨回绕。
        const uint32_t time_step = wheels[i].time_ms - odometry_anchor_wheels[i].time_ms; // 同一毫秒可以收到多帧。
        if (wheels[i].sequence == 0u || (anchor_recent && (sequence_step > INT32_MAX || time_step > INT32_MAX)))
        {
            odometry_integrating = 0u; // 任一轮缺少身份或近期样本倒退时中断当前段。
            return NULL;
        }
        if (odometry_anchor_known && sequence_step == 0u)
            all_new = 0u; // 另外几轮更新也不能让该轮的旧反馈再次参与积分。
    }
    const uint32_t elapsed_ms = velocity->source_time_ms - odometry_anchor.source_time_ms; // 无符号差同时适用于时间回绕。
    if (!all_new || (anchor_recent && elapsed_ms == 0u))
    {
        if (!anchor_recent) odometry_integrating = 0u; // 未等齐新反馈前也不能延长旧位姿期限。
        return odometry_integrating ? &odometry_anchor : NULL; // 样本未齐或毫秒时间未前进，保留原积分基准。
    }

    if (!odometry_integrating || !anchor_recent || elapsed_ms > chassis_config->odometry_max_gap_ms)
    {
        odometry_anchor.pose = (ChassisLocalPose_t){0}; // 当前样本成为新的起点和初始朝向。
        ++odometry_anchor.segment; // 读者据此识别两份位姿是否共享原点。
    }
    else if (ChassisKinematicsIntegrate(&odometry_anchor.pose, &odometry_anchor_twist,
                                       &velocity->twist, (float)elapsed_ms * 0.001f) != CHASSIS_KINEMATICS_OK)
    {
        odometry_integrating = 0u; // 位姿计算失败，下一份新样本重新起算。
        return NULL;
    }

    odometry_anchor.source_time_ms = velocity->source_time_ms; // 不用任务执行时间延长有效期。
    odometry_anchor_twist = velocity->twist; // 作为下一段区间中较早一端的速度。
    memcpy(odometry_anchor_wheels, wheels, sizeof(odometry_anchor_wheels)); // 四轮身份与这次积分一起推进。
    odometry_anchor_known = 1u; // 只有可信的新样本可以更新比较基准。
    odometry_integrating = 1u; // 当前段已有可发布位姿。
    return &odometry_anchor;
}
#endif

#if defined(ONE_BOARD) || defined(CHASSIS_BOARD)
// 轮速计算完成后再核对消息；若已有不同命令，下一轮才用它重新计算。
static uint8_t ChassisCommandIsSameAndFresh(const MessageFreshness_t *used, uint32_t now_ms)
{
    MessageFreshness_t latest = {0}; // 复制当前命令时一并取得序号和来源时间。
    return MessageReadSnapshot(chassis_sub, &chassis_cmd_recv, sizeof(chassis_cmd_recv),
                               now_ms, &latest) == MESSAGE_OK &&
        latest.sequence == used->sequence && latest.source_time_ms == used->source_time_ms;
}
#endif

#if defined(ONE_BOARD)
// 仅对刚复核过的本地消息取截止点，减法得出的剩余时间不会跨过半个计数范围。
static uint32_t ChassisCommandDeadline(const MessageFreshness_t *command, uint32_t now_ms)
{
    if (command->source_max_age_ms == 0u)
        return command->source_time_ms + chassis_command_max_age_ms; // 旧本地消息从原始来源时刻起算话题期限。
    const uint32_t source_remaining = command->source_max_age_ms - (now_ms - command->source_time_ms); // 来源允许的剩余时间。
    const uint32_t topic_remaining = chassis_command_max_age_ms - (now_ms - command->published_time_ms); // 本次写入话题的剩余时间。
    return now_ms + (source_remaining < topic_remaining ? source_remaining : topic_remaining); // 交付更早的截止点。
}
#endif

/* 机器人底盘控制核心任务 */
void ChassisTask()
{
    // 读取时一起检查话题和来源期限，失败后清零旧命令。
    uint8_t command_fresh = 0u;
    MessageFreshness_t command_freshness = {0};
#if defined(ONE_BOARD) || defined(CHASSIS_BOARD)
    command_fresh = (uint8_t)(MessageReadSnapshot(chassis_sub, &chassis_cmd_recv,
        sizeof(chassis_cmd_recv), PlatformTimeNowMs(), &command_freshness) == MESSAGE_OK);
#endif
    if (!command_fresh)
        memset(&chassis_cmd_recv, 0, sizeof(chassis_cmd_recv));

#ifdef CHASSIS_BOARD
    const uint8_t command_requests_motion = (uint8_t)(command_fresh != 0u &&
        chassis_cmd_recv.chassis_mode != CHASSIS_ZERO_FORCE);
    const uint8_t motor_feedback_healthy = ChassisMotorFeedbackIsHealthy();
    RobotSafetyUpdate();
    RobotSafetySnapshot_t safety_snapshot = {0};
    const uint8_t safety_snapshot_valid = RobotSafetyGetSnapshot(&safety_snapshot);
    if (safety_snapshot_valid == 0u)
    {
        chassis_command_rearm_required = 1u;
        chassis_rearm_tracking = 0u;
        chassis_rearm_sequence_valid = 0u;
    }
    else if (chassis_inhibit_generation_valid == 0u ||
             safety_snapshot.chassis_inhibit_generation != chassis_inhibit_generation)
    {
        chassis_inhibit_generation = safety_snapshot.chassis_inhibit_generation;
        chassis_inhibit_generation_valid = 1u;
        chassis_command_rearm_required = 1u;
        chassis_rearm_tracking = 0u;
        chassis_rearm_sequence_valid = 0u;
    }
    const uint8_t local_ready = (uint8_t)(safety_snapshot_valid != 0u &&
        RobotSafetyIsChassisLocalReady() != 0u && motor_feedback_healthy != 0u);
    if (local_ready == 0u || command_requests_motion == 0u)
    {
        chassis_command_rearm_required = 1u;
        chassis_rearm_tracking = 0u;
        chassis_rearm_sequence_valid = 0u;
    }
    else if (chassis_command_rearm_required != 0u)
    {
        if (chassis_rearm_tracking == 0u)
        {
            // 记录本地恢复时刻和当时已接收的序号，旧分片晚到也不能取得新许可。
            chassis_rearm_after_ms = PlatformTimeNowMs();
            chassis_rearm_sequence_valid = (uint8_t)(command_fresh != 0u &&
                                                       command_freshness.valid != 0u);
            if (chassis_rearm_sequence_valid != 0u)
                chassis_rearm_sequence = command_freshness.sequence;
            chassis_rearm_tracking = 1u;
        }
        else if (command_fresh != 0u && command_freshness.valid != 0u &&
                 (chassis_rearm_sequence_valid == 0u ||
                  command_freshness.sequence != chassis_rearm_sequence) &&
                 // 映射后的来源时刻晚于本地恢复点，拒绝恢复前发出、之后才拼完的命令。
                 (int32_t)(command_freshness.source_time_ms - chassis_rearm_after_ms) > 0)
        {
            chassis_command_rearm_required = 0u;
            chassis_rearm_tracking = 0u;
        }
    }
    const uint8_t remote_permit_requested = (uint8_t)(command_requests_motion != 0u &&
        local_ready != 0u && chassis_command_rearm_required == 0u &&
        command_freshness.valid != 0u);
    uint8_t remote_permit = 0u;
    if (remote_permit_requested != 0u)
    {
        remote_permit = RobotSafetyTryGrantRemoteChassisPermit(
            safety_snapshot.chassis_inhibit_generation,
            command_freshness.source_time_ms, chassis_command_max_age_ms, PlatformTimeNowMs());
        if (remote_permit == 0u)
        {
            chassis_command_rearm_required = 1u;
            chassis_rearm_tracking = 0u;
            chassis_rearm_sequence_valid = 0u;
        }
    }
    else
        RobotSafetyRevokeRemoteChassisPermit();
    RobotSafetyUpdate();
    uint8_t local_permit = (uint8_t)(RobotSafetyIsReadyAt(PlatformTimeNowMs()) != 0u &&
                                     local_ready != 0u);
#endif

    ChassisMotionMode_e motion_mode = CHASSIS_MOTION_MODE_INVALID;
    // 只将支持的模式交给运动计算；停机或未知模式都保持无效，随后停止四轮。
    switch (chassis_cmd_recv.chassis_mode)
    {
    case CHASSIS_NO_FOLLOW:
        motion_mode = CHASSIS_MOTION_MODE_NO_FOLLOW;
        break;
    case CHASSIS_FOLLOW_GIMBAL_YAW:
        motion_mode = CHASSIS_MOTION_MODE_FOLLOW;
        break;
    case CHASSIS_ROTATE:
        motion_mode = CHASSIS_MOTION_MODE_ROTATE;
        break;
    default:
        break;
    }

    const ChassisMotionInput_s motion_input = {
        .vx_mm_s = chassis_cmd_recv.vx,
        .vy_mm_s = chassis_cmd_recv.vy,
        .requested_wz_deg_s = chassis_cmd_recv.wz,
        .offset_angle_deg = chassis_cmd_recv.offset_angle,
        .mode = motion_mode,
    };
    const ChassisMotionGeometry_s geometry = {
        .reduction_ratio = chassis_config->chassis_motor_config.output_axis.gear_ratio, // 与电机反馈使用同一减速比。
        .model = chassis_config->wheel_type == OMNI_WHEEL ?
                     CHASSIS_MOTION_MODEL_OMNI : CHASSIS_MOTION_MODEL_MECANUM, // 使用本车型已选轮组。
        .mecanum = chassis_config->mecanum_geometry, // 麦轮半轴距、半轮距、计算中心偏移和轮径，m。
        .omni = chassis_config->omni_geometry, // 各全向轮相对底盘中心的轮位、方向和轮径，m。
    };
    const ChassisMotionLimits_s limits = chassis_config->motion_limits;
    ChassisMotionOutput_s motion_output;
    uint8_t command_allowed = command_fresh != 0u &&
#ifdef CHASSIS_BOARD
                                    local_permit != 0u &&
#endif
                                    chassis_cmd_recv.chassis_mode != CHASSIS_ZERO_FORCE &&
                                    ChassisMotionCalculate(&motion_input, &geometry, &limits,
                                                           &motion_output);
#if defined(ONE_BOARD)
    uint32_t command_deadline_ms = 0u; // 只有复核通过才用于提交，截止点为零也合法。
    if (command_allowed != 0u)
    {
        const uint32_t checked_at_ms = PlatformTimeNowMs(); // 到期检查和剩余时间使用同一时刻。
        command_allowed = ChassisCommandIsSameAndFresh(&command_freshness, checked_at_ms); // 拒绝计算期间失效或替换的旧命令。
        if (command_allowed != 0u)
            command_deadline_ms = ChassisCommandDeadline(&command_freshness, checked_at_ms); // 保留已复核命令的原始截止点。
    }
#elif defined(CHASSIS_BOARD)
    if (command_allowed != 0u)
    {
        if (!ChassisCommandIsSameAndFresh(&command_freshness, PlatformTimeNowMs()) || RobotSafetyIsReadyAt(PlatformTimeNowMs()) == 0u ||
            ChassisMotorFeedbackIsHealthy() == 0u)
        {
            command_allowed = 0u;
            local_permit = 0u;
            RobotSafetyRevokeRemoteChassisPermit();
            RobotSafetyUpdate();
        }
    }
    if (command_allowed == 0u && local_permit != 0u)
    {
        chassis_command_rearm_required = 1u;
        chassis_rearm_tracking = 0u;
        chassis_rearm_sequence_valid = 0u;
        RobotSafetyRevokeRemoteChassisPermit();
        RobotSafetyUpdate();
    }
    if (command_allowed == 0u)
    {
        chassis_command_rearm_required = 1u;
    }
#endif
    if (!command_allowed)
    {
        // 配置、模式或数值不允许计算时，同时停机和清掉参考值，避免重新使能后使用旧速度。
        ChassisStopAndClear();
    }
    else
    {
        const float wheel_direction = geometry.model == CHASSIS_MOTION_MODEL_OMNI ? 1.0f : -1.0f; // 全向轮目标与估速共用轴方向，麦轮沿用旧电机映射。
        const float rotor_to_output = wheel_direction * DEGREE_2_RAD / geometry.reduction_ratio; // 把受限的电机轴 deg/s 还原为输出轴 rad/s。
#if defined(ONE_BOARD)
        // 每台轮电机都用命令原有的代际；任何一台未获许可时清除四轮目标。
        const uint32_t generation = chassis_cmd_recv.safety_generation;
        const uint8_t lf_ok = DJIMotorSetChassisRefUntil(motor_lf, motion_output.motor_speed_deg_s[0] * rotor_to_output, generation, command_deadline_ms); // 左前沿用来源期限。
        const uint8_t rf_ok = DJIMotorSetChassisRefUntil(motor_rf, motion_output.motor_speed_deg_s[1] * rotor_to_output, generation, command_deadline_ms); // 右前沿用同一截止点。
        const uint8_t lb_ok = DJIMotorSetChassisRefUntil(motor_lb, motion_output.motor_speed_deg_s[2] * rotor_to_output, generation, command_deadline_ms); // 左后沿用同一截止点。
        const uint8_t rb_ok = DJIMotorSetChassisRefUntil(motor_rb, motion_output.motor_speed_deg_s[3] * rotor_to_output, generation, command_deadline_ms); // 右后沿用同一截止点。
        if (!(lf_ok && rf_ok && lb_ok && rb_ok))
        {
            ChassisStopAndClear(); // 任一参考被拒绝时撤销全部轮组参考。
            command_allowed = 0u; // 继续发布反馈，但不重新使能轮组。
        }
#else
        DJIMotorSetRef(motor_lf, motion_output.motor_speed_deg_s[0] * rotor_to_output);
        DJIMotorSetRef(motor_rf, motion_output.motor_speed_deg_s[1] * rotor_to_output);
        DJIMotorSetRef(motor_lb, motion_output.motor_speed_deg_s[2] * rotor_to_output);
        DJIMotorSetRef(motor_rb, motion_output.motor_speed_deg_s[3] * rotor_to_output);
#endif
        if (command_allowed)
        {
            DJIMotorEnable(motor_lf);
            DJIMotorEnable(motor_rf);
            DJIMotorEnable(motor_lb);
            DJIMotorEnable(motor_rb);
            chassis_cmd_recv.wz = motion_output.wz_deg_s;
        }
    }

    // 全向轮估计独立于命令许可，停机滑行期间也能读取实测速度。
#if !defined(MC02_CONFIG_GENERATED) || MC02_CHASSIS_IS_OMNI
    ChassisVelocityFeedback_t velocity;
    ChassisWheelObservation_t wheels[CHASSIS_WHEEL_COUNT]; // 里程计对照上次积分使用的四轮反馈。
    const ChassisVelocityFeedback_t *feedback = chassis_config->wheel_type == OMNI_WHEEL &&
        EstimateOmniVelocity(&velocity, wheels) ? &velocity : NULL; // 不支持或轮速失效时，两个本地结果都撤销。
    PublishMotionFeedback(feedback, UpdateOdometry(feedback, wheels));
#endif

    // // 获取裁判系统数据   建议将裁判系统与底盘分离，所以此处数据应使用消息中心发送
    // // 我方颜色id小于7是红色,大于7是蓝色,注意这里发送的是对方的颜色, 0:blue , 1:red
    // // 当前只做了17mm热量的数据获取,后续根据robot_def中的宏切换双枪管和英雄42mm的情况

    // 推送反馈消息
#if defined(ONE_BOARD) || defined(CHASSIS_BOARD)
    (void)MessagePublishNow(chassis_pub, &chassis_feedback_data);
#endif
}

void Chassis_GetMotorSummary(Chassis_Motor_Summary_t *summary)
{
    if (summary == NULL)
        return;

    *summary = (Chassis_Motor_Summary_t){0};

    DJIMotorGetSummary(motor_lf, &summary->lf);
    DJIMotorGetSummary(motor_rf, &summary->rf);
    DJIMotorGetSummary(motor_lb, &summary->lb);
    DJIMotorGetSummary(motor_rb, &summary->rb);
}
