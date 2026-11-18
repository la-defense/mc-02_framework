/**
 * @file chassis.c
 * @author NeoZeng neozng1@hnu.edu.cn
 * @brief 底盘应用,负责接收robot_cmd的控制命令并根据命令进行运动学解算,得到输出
 *        注意底盘采取右手系,对于平面视图,底盘纵向运动的正前方为x正方向;横向运动的右侧为y正方向
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

// 更新结果和有效位时不允许其他任务读取未完成的数据。
static void PublishVelocity(const ChassisVelocityFeedback_t *velocity)
{
    PlatformTaskCriticalEnter(); // 进入短临界区，只交付数据。
    chassis_velocity = velocity != NULL ? *velocity : (ChassisVelocityFeedback_t){0}; // 无效时一起清除三个分量。
    chassis_velocity_valid = (uint8_t)(velocity != NULL); // 读者不会误用已清除的值。
    PlatformTaskCriticalExit();
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
static uint32_t chassis_command_max_age_ms = 0u;
#endif

uint8_t ChassisInit(void)
{
    PublishVelocity(NULL); // 重新初始化时清除旧轮组估计。
    chassis_rollback_failed = 0u; // 本次初始化从无回退故障开始。
    chassis_config = ChassisConfigFeed();
    if (chassis_config == NULL)
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
    MessageTopicDescriptor_t descriptor = {0}; // 在登记端点之前核对命令话题期限。
    if (MessageTopicGetDescriptor(ROBOT_TOPIC_CHASSIS_COMMAND, &descriptor) != MESSAGE_OK ||
        descriptor.max_age_ms == 0u) return 0u; // 无效话题不能开始申请电机。
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
    const uint8_t motor_ids[4] = {
        chassis_config->chassis_motor_id[MOTOR_LF], // 左前轮的设备编号。
        chassis_config->chassis_motor_id[MOTOR_RF], // 右前轮的设备编号。
        chassis_config->chassis_motor_id[MOTOR_LB], // 左后轮的设备编号。
        chassis_config->chassis_motor_id[MOTOR_RB], // 右后轮的设备编号。
    };
    for (uint8_t i = 0u; i < 4u; ++i)
    {
        chassis_motor_config.can_init_config.tx_id = motor_ids[i]; // 每轮使用对应配置编号。
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
#endif
#ifdef CHASSIS_BOARD
    chassis_command_max_age_ms = descriptor.max_age_ms; // 使用事先检查的命令期限。
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
    PublishVelocity(NULL); // 资源撤销后不能再交付旧轮速。
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

#if !defined(MC02_CONFIG_GENERATED) || MC02_CHASSIS_IS_OMNI
// 取四个已经换算到输出轴的轮速；不足四份时整个底盘速度无效。
static uint8_t EstimateOmniVelocity(ChassisVelocityFeedback_t *velocity)
{
    DJIMotorInstance *const motors[CHASSIS_WHEEL_COUNT] = {motor_lf, motor_rf, motor_lb, motor_rb}; // 统一轮序。
    ChassisWheelSpeeds_t speeds;
    uint32_t received_at[CHASSIS_WHEEL_COUNT];
    for (unsigned i = 0u; i < CHASSIS_WHEEL_COUNT; ++i)
    {
        Motor_Feedback_Snapshot_s feedback = {0};
        if (DJIMotorGetFeedbackSnapshot(motors[i], PlatformTimeNowMs(), &feedback) != MOTOR_FEEDBACK_OK ||
            feedback.speed.valid != 1u || feedback.speed.unit != MOTOR_UNIT_RAD_S ||
            feedback.speed.reference != MOTOR_REFERENCE_OUTPUT_AXIS)
            return 0u; // 未接收、过期或错误单位的轮速均不能用来估算。
        speeds.rad_s[i] = feedback.speed.value; // 驱动已按安装方向和减速比换算，不重复处理符号。
        received_at[i] = feedback.speed.source_time_ms; // 记录原始接收时间，不以本轮执行时间替代。
    }
    if (OmniEstimate(&speeds, &chassis_config->omni_geometry, &velocity->twist) != CHASSIS_KINEMATICS_OK)
        return 0u; // 几何退化或数值异常时不发布部分结果。
    const uint32_t now = PlatformTimeNowMs(); // 计算后检查最早轮速是否依然有效。
    uint32_t oldest_age = 0u;
    for (unsigned i = 0u; i < CHASSIS_WHEEL_COUNT; ++i)
    {
        const uint32_t age = now - received_at[i]; // 无符号差值处理本机毫秒回绕。
        if (age > DJI_FEEDBACK_MAX_AGE_MS) return 0u; // 错误的未来时间也无法通过。
        if (age > oldest_age) oldest_age = age; // 选择剩余有效时间最短的样本。
    }
    velocity->source_time_ms = now - oldest_age; // 保留这个样本的原始时刻。
    return 1u;
}
#endif

/* 机器人底盘控制核心任务 */
void ChassisTask()
{
    // 单板消息和双板通信都按 50 ms 检查命令，失败后清零，不沿用旧速度。
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
    const Wheel_Measure_s *wheel_measure = &chassis_config->wheel_measure;
    const ChassisMotionGeometry_s geometry = {
        .wheel_base_mm = wheel_measure->wheel_base,
        .track_width_mm = wheel_measure->track_width,
        .center_gimbal_offset_x_mm = wheel_measure->center_gimbal_offset_x,
        .center_gimbal_offset_y_mm = wheel_measure->center_gimbal_offset_y,
        .wheel_radius_mm = wheel_measure->radius_wheel,
        .reduction_ratio = wheel_measure->reduction_ratio_wheel,
        .model = chassis_config->wheel_type == OMNI_WHEEL ?
                     CHASSIS_MOTION_MODEL_OMNI : CHASSIS_MOTION_MODEL_MECANUM,
        .omni = chassis_config->omni_geometry,
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
#ifdef CHASSIS_BOARD
    if (command_allowed != 0u)
    {
        Chassis_Ctrl_Cmd_s latest_command = {0};
        MessageFreshness_t latest_freshness = {0};
        const uint8_t latest_is_same_and_fresh = (uint8_t)(
            MessageReadSnapshot(chassis_sub, &latest_command, sizeof(latest_command),
                                PlatformTimeNowMs(), &latest_freshness) == MESSAGE_OK &&
            latest_freshness.sequence == command_freshness.sequence &&
            latest_freshness.source_time_ms == command_freshness.source_time_ms);
        if (latest_is_same_and_fresh == 0u || RobotSafetyIsReadyAt(PlatformTimeNowMs()) == 0u ||
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
        const float rotor_to_output = -DEGREE_2_RAD / wheel_measure->reduction_ratio_wheel;
#if defined(ONE_BOARD)
        // 每台轮电机都用命令原有的代际；任何一台未获许可时清除四轮目标。
        const uint32_t generation = chassis_cmd_recv.safety_generation;
        const uint8_t lf_ok = DJIMotorSetRefForGeneration(motor_lf, motion_output.motor_speed_deg_s[0] * rotor_to_output, generation);
        const uint8_t rf_ok = DJIMotorSetRefForGeneration(motor_rf, motion_output.motor_speed_deg_s[1] * rotor_to_output, generation);
        const uint8_t lb_ok = DJIMotorSetRefForGeneration(motor_lb, motion_output.motor_speed_deg_s[2] * rotor_to_output, generation);
        const uint8_t rb_ok = DJIMotorSetRefForGeneration(motor_rb, motion_output.motor_speed_deg_s[3] * rotor_to_output, generation);
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
    if (chassis_config->wheel_type == OMNI_WHEEL && EstimateOmniVelocity(&velocity))
        PublishVelocity(&velocity);
    else
        PublishVelocity(NULL); // 当前轮组无估计功能或本轮数据失效。
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
