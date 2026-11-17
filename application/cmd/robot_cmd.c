/** @file robot_cmd.c
* @brief 每轮取得遥控、视觉和应用反馈，选择目标后让整车状态覆盖不允许执行的命令，再统一发布。
 */
#include "remote_safety.h"
static RemoteSafetyGestures_s remote_gestures;
// app
#include "robot_def.h"
#include "robot_cmd.h"
#include "config.h"
// module
#include "remote_control.h"
#include "ins_task.h"
#include "master_process.h"
#include "message_center.h"
#include "topics/robot_topics.h"
#include "general_def.h"
#include "dji_motor.h"
#include "robot_safety.h"
#if defined(MC02_CONFIG_GENERATED) && MC02_FEATURE_COMMAND && MC02_FEATURE_REMOTE
#include "std_cmd_remote.h"
#include "robot_manual_motion.h"
#include <limits.h>
#endif
#if defined(MC02_CONFIG_GENERATED) && defined(MC02_PROFILE_ROBOT) && defined(ONE_BOARD) && MC02_FEATURE_COMMAND && MC02_FEATURE_REMOTE && MC02_SELECTED_DRIVER_DJI
#include "robot_can_recovery_coordinator.h"
#if defined(MC02_VISION_PROTOCOL_V2)
#define ROBOT_RECOVERY_KEYBOARD_ALLOWED 0u // 新协议将该档位用于自主。
#else
#define ROBOT_RECOVERY_KEYBOARD_ALLOWED 1u // 旧协议将该档位用于人工键鼠。
#endif
#endif
#include "daemon.h"
#include "vision_control.h"
#include "../../bsp/os/platform_sync.h"
#if defined(MC02_CONFIG_GENERATED)
#include "config/vehicle_selection.h"
#endif
#if defined(MC02_VISION_PROTOCOL_V2)
#include "vision_protocol_v2.h"
#include "referee_snapshot.h"
#include "rm_referee.h"
#endif
#include <string.h>
#include <math.h>

// 编译时检查视觉与应用两处供弹模式的编号是否一致。
_Static_assert((int)LOAD_STOP == (int)VISION_CONTROL_LOAD_STOP,
               "Vision control loader mapping must match LOAD_STOP");
_Static_assert((int)LOAD_REVERSE == (int)VISION_CONTROL_LOAD_REVERSE,
               "Vision control loader mapping must match LOAD_REVERSE");
_Static_assert((int)LOAD_1_BULLET == (int)VISION_CONTROL_LOAD_SINGLE_BULLET,
               "Vision control loader mapping must match LOAD_1_BULLET");
_Static_assert((int)LOAD_3_BULLET == (int)VISION_CONTROL_LOAD_THREE_BULLETS,
               "Vision control loader mapping must match LOAD_3_BULLET");
_Static_assert((int)LOAD_BURSTFIRE == (int)VISION_CONTROL_LOAD_BURSTFIRE,
               "Vision control loader mapping must match LOAD_BURSTFIRE");
// bsp
#include "bsp_log.h"

// 私有宏,自动将编码器转换成角度值
#if !defined(MC02_CONFIG_GENERATED)
#define YAW_ALIGN_ANGLE (YAW_CHASSIS_ALIGN_ECD * ECD_ANGLE_COEF_DJI)
#endif

static VisionControlLimits_s vision_limits = {
    .configured = VISION_LIMITS_CONFIGURED,
    .yaw_constraint = VISION_YAW_CONSTRAINT,
#if !defined(MC02_CONFIG_GENERATED)
    .pitch_min_deg = PITCH_MIN_ANGLE,
    .pitch_max_deg = PITCH_MAX_ANGLE,
#endif
    .pitch_zero_offset_deg = VISION_PITCH_ZERO_OFFSET_DEG,
    .yaw_min_deg = VISION_YAW_MIN_DEG,
    .yaw_max_deg = VISION_YAW_MAX_DEG,
};
#if defined(MC02_CONFIG_GENERATED) && defined(MC02_VISION_PROTOCOL_V2)
static const Chassis_Config_s *command_chassis_config;
#endif
#if defined(MC02_CONFIG_GENERATED)
static float yaw_align_angle_deg;
#else
static float yaw_align_angle_deg = YAW_ALIGN_ANGLE;
#endif
static uint8_t previous_vision_active;
static uint8_t manual_reference_ready;

typedef struct {
    uint32_t source_ms; // 当前最先到期的原始来源时刻。
    uint32_t max_age_ms; // 该来源允许的总年龄。
    uint32_t remaining_ms; // 在本轮取时刻还可使用的时间。
    uint8_t valid; // 所有已合入来源都有效时为1。
} RobotCmdLease_t;

// 合并一个本机时间来源；任一依赖失效都使该命令不可发布为有效目标。
static void RobotCmdLeaseInclude(RobotCmdLease_t *lease, uint8_t available,
                                 uint32_t source_ms, uint32_t max_age_ms, uint32_t now_ms)
{
    if (!lease->valid) return; // 先前依赖已经失效时不能由新来源恢复。
    const uint32_t age = now_ms - source_ms; // 无符号差值允许本机时钟回绕。
    if (!available || max_age_ms == 0u || max_age_ms > INT32_MAX || age > max_age_ms)
    {
        lease->valid = 0u; // 缺失、将来时间和超期均不能授予目标。
        return;
    }
    const uint32_t remaining = max_age_ms - age; // 比较各依赖剩余时间而不是只比较原始时刻。
    if (remaining < lease->remaining_ms)
    {
        lease->source_ms = source_ms; // 保留最先到期依赖的原始时间。
        lease->max_age_ms = max_age_ms; // 同时保留它自己的期限。
        lease->remaining_ms = remaining; // 后续依赖可以进一步缩短允许时间。
    }
}

// 命令话题还按自己的发布期限检查；失败时撤销旧目标。
static uint8_t RobotCmdPublishBounded(MessagePublisher_t *publisher, const void *command,
                                      const RobotCmdLease_t *lease, uint32_t now_ms)
{
    MessageStatus_t status = MESSAGE_ERR_RANGE; // 来源缺失时直接撤销旧目标。
    if (lease->valid)
    {
#if defined(GIMBAL_BOARD)
        if (lease->max_age_ms >= MESSAGE_DEFAULT_MAX_AGE_MS &&
            (uint32_t)(now_ms - lease->source_ms) <= MESSAGE_DEFAULT_MAX_AGE_MS)
            status = MessagePublishAt(publisher, command, lease->source_ms, lease->source_ms,
                                      now_ms, MESSAGE_CLOCK_LOCAL); // 双板现有话题按来源固定 50 ms，不能延长当前来源。
#else
        status = MessagePublishAtBounded(publisher, command, lease->source_ms, lease->source_ms,
                                         now_ms, MESSAGE_CLOCK_LOCAL, lease->max_age_ms); // 单板分别保留来源和发布期限。
#endif
    }
    if (status != MESSAGE_OK)
    {
        (void)MessagePublisherInvalidate(publisher); // 不能把失败前的目标继续留给执行任务。
        return 0u; // 通知云台接续逻辑丢弃未成功发布的内部目标。
    }
    return 1u; // 本轮目标已按期限交付。
}

// 只接受配置确认且为正数的底盘最大角速度，旧主机夹具沿用宏输入。
#if defined(MC02_VISION_PROTOCOL_V2)
static uint8_t GetMaximumChassisYawRateDegS(float *maximum)
{
    if (maximum == NULL)
        return 0u;
#if defined(MC02_CONFIG_GENERATED)
    if (command_chassis_config == NULL ||
        command_chassis_config->motion_limits.configured == 0u ||
        !isfinite(command_chassis_config->motion_limits.max_wz_deg_s) ||
        command_chassis_config->motion_limits.max_wz_deg_s <= 0.0f)
        return 0u;
    *maximum = command_chassis_config->motion_limits.max_wz_deg_s;
#else
    if (CHASSIS_MOTION_LIMITS_CONFIGURED == 0u ||
        !isfinite(CHASSIS_MAX_WZ_DEG_S) || CHASSIS_MAX_WZ_DEG_S <= 0.0f)
        return 0u;
    *maximum = CHASSIS_MAX_WZ_DEG_S;
#endif
    return 1u;
}
#endif

/* cmd应用包含的模块实例指针和交互信息存储*/
#if defined(ONE_BOARD) || defined(GIMBAL_BOARD)
static MessagePublisher_t *chassis_cmd_pub; // 底盘控制消息发布者
#endif

static Chassis_Ctrl_Cmd_s chassis_cmd_send;      // 发送给底盘应用的信息,包括控制信息和UI绘制相关

static RemoteControlSnapshot_s remote_snapshot;
static RC_ctrl_t *const rc_data = &remote_snapshot.input;
static uint8_t remote_initialized;
#if !defined(MC02_VISION_PROTOCOL_V2)
static Vision_Recv_s vision_recv_snapshot;
#endif
static Vision_Status_t vision_status_snapshot;
#if defined(MC02_VISION_PROTOCOL_V2)
static VisionV2AimCommand_t vision_v2_aim_snapshot;
static VisionV2NavigationCommand_t vision_v2_navigation_snapshot;
static MessageFreshness_t vision_v2_aim_freshness;
static MessageFreshness_t vision_v2_navigation_freshness;
static MessageSubscriber_t *vision_v2_aim_subscriber;
static MessageSubscriber_t *vision_v2_navigation_subscriber;
static uint8_t vision_v2_aim_valid;
static uint8_t vision_v2_navigation_valid;
#endif

static MessagePublisher_t *gimbal_cmd_pub;            // 云台控制消息发布者
static MessageSubscriber_t *gimbal_feed_sub;          // 云台反馈信息订阅者
static Gimbal_Ctrl_Cmd_s gimbal_cmd_send;      // 传递给云台的控制信息
static Gimbal_Upload_Data_s gimbal_fetch_data; // 从云台获取的反馈信息
#if defined(MC02_CONFIG_GENERATED) && MC02_FEATURE_COMMAND && MC02_FEATURE_REMOTE
static const StdCmdRemoteMap_t *manual_input_map; // 车型确认后的只读输入映射。
static uint32_t manual_max_dt_ms, manual_last_ms, manual_source_generation; // 保存积分限制、上次时刻和来源代次。
static uint32_t manual_safety_generation; // 故障恢复后不能沿用禁止事件前的枪口参考。
static uint8_t manual_clock_valid, manual_intent_valid, manual_safety_generation_seen; // 区分尚未读取和代次恰好为零。
static StdCmdManualIntent_t manual_intent; // 保存本轮来源及期限，发布时再次检查。
#endif

static MessagePublisher_t *shoot_cmd_pub;           // 发射控制消息发布者
static Shoot_Ctrl_Cmd_s shoot_cmd_send;      // 传递给发射的控制信息

uint8_t RobotCMDInit(void)
{
    if (!RobotTopicsInit())
        return 0u;
#if defined(MC02_CONFIG_GENERATED)
    const Robot_Config_s *config = RobotConfigFeed();
    if (config == NULL)
        return 0u;
#if MC02_FEATURE_COMMAND && MC02_FEATURE_REMOTE
    manual_input_map = config->manual_input.map; // 只接受车型提供的人工比例和方向。
    manual_max_dt_ms = config->manual_input.max_dt_ms; // 积分超时上限也必须明确。
    if (!StdCmdRemoteMapIsValid(manual_input_map) || manual_max_dt_ms == 0u ||
        manual_max_dt_ms > INT32_MAX ||
        (manual_input_map->remote_frame != STD_CMD_FRAME_CHASSIS && manual_input_map->remote_frame != STD_CMD_FRAME_BIG_YAW) ||
        (manual_input_map->keyboard_frame != STD_CMD_FRAME_CHASSIS && manual_input_map->keyboard_frame != STD_CMD_FRAME_BIG_YAW))
        return 0u; // 缺少有效映射、积分上限或旧应用尚不支持的参照时不能启动命令模块。
    manual_clock_valid = manual_intent_valid = manual_safety_generation_seen = 0u; // 启动不继承旧命令、时间或安全代次。
#endif
    // 云台和底盘机械偏置从本车型唯一配置读取。
#if defined(MC02_VISION_PROTOCOL_V2)
    command_chassis_config = &config->chassis_param;
#endif
    yaw_align_angle_deg = config->gimbal_param.gimbal_offset.yaw_offset;
    vision_limits.pitch_min_deg = config->gimbal_param.gimbal_offset.pitch_min_angle;
    vision_limits.pitch_max_deg = config->gimbal_param.gimbal_offset.pitch_max_angle;
    if (!isfinite(yaw_align_angle_deg) ||
        !isfinite(vision_limits.pitch_min_deg) || !isfinite(vision_limits.pitch_max_deg) ||
        vision_limits.pitch_min_deg >= vision_limits.pitch_max_deg)
        return 0u;
#endif
    RemoteSafetyGesturesInit(&remote_gestures);
#if defined(MC02_CONFIG_GENERATED)
    // 车辆启动阶段已按模块依赖初始化输入源；命令模块只确认依赖，不重复注册设备。
    remote_initialized = RobotBuildModuleStarted(ROBOT_MODULE_REMOTE);
    if (!remote_initialized || !RobotBuildModuleStarted(ROBOT_MODULE_VISION))
        return 0u;
#else
    remote_initialized = (uint8_t)(RemoteControlInit(PLATFORM_UART_5) != NULL);
    if (!remote_initialized || VisionInit(PLATFORM_UART_9) == NULL)
        return 0u;
#endif
#if defined(MC02_VISION_PROTOCOL_V2)
    vision_v2_aim_subscriber = MessageSubscriberRegister(ROBOT_TOPIC_VISION_AIM,
        ROBOT_TOPIC_ENDPOINT_COMMAND);
    vision_v2_navigation_subscriber = MessageSubscriberRegister(ROBOT_TOPIC_VISION_NAVIGATION,
        ROBOT_TOPIC_ENDPOINT_COMMAND);
    if (vision_v2_aim_subscriber == NULL || vision_v2_navigation_subscriber == NULL)
        return 0u;
#endif

    gimbal_cmd_pub = MessagePublisherRegister(ROBOT_TOPIC_GIMBAL_COMMAND,
                                               ROBOT_TOPIC_ENDPOINT_COMMAND);
    gimbal_feed_sub = MessageSubscriberRegister(ROBOT_TOPIC_GIMBAL_FEEDBACK,
                                                 ROBOT_TOPIC_ENDPOINT_COMMAND);
    shoot_cmd_pub = MessagePublisherRegister(ROBOT_TOPIC_SHOOT_COMMAND,
                                              ROBOT_TOPIC_ENDPOINT_COMMAND);
#if defined(ONE_BOARD) || defined(GIMBAL_BOARD)
    chassis_cmd_pub = MessagePublisherRegister(ROBOT_TOPIC_CHASSIS_COMMAND,
                                                ROBOT_TOPIC_ENDPOINT_COMMAND);
#endif
    gimbal_cmd_send.pitch = 0;

    // 上电默认安全: 所有控制量保持0/失能, 由RobotSafety状态机确认后才会进入READY
    memset(&chassis_cmd_send, 0, sizeof(chassis_cmd_send));
    memset(&gimbal_cmd_send, 0, sizeof(gimbal_cmd_send));
    memset(&shoot_cmd_send, 0, sizeof(shoot_cmd_send));
    gimbal_cmd_send.gimbal_mode = GIMBAL_ZERO_FORCE;
    chassis_cmd_send.chassis_mode = CHASSIS_ZERO_FORCE;
    shoot_cmd_send.shoot_mode = SHOOT_OFF;
    shoot_cmd_send.friction_mode = FRICTION_OFF;
    shoot_cmd_send.load_mode = LOAD_STOP;
    previous_vision_active = 0u;
    manual_reference_ready = 0u;
    if (gimbal_cmd_pub == NULL || gimbal_feed_sub == NULL || shoot_cmd_pub == NULL)
        return 0u;
#if defined(ONE_BOARD) || defined(GIMBAL_BOARD)
    if (chassis_cmd_pub == NULL)
        return 0u;
#endif
    return 1u;
}

/**
 * @brief 根据gimbal app传回的当前电机角度计算和零位的误差
 *        单圈绝对角度的范围是0~360,说明文档中有图示
 *
 */
static void CalcOffsetAngle()
{
    const float angle = (float)gimbal_fetch_data.yaw_motor_single_round_angle;
    if (!isfinite(angle) || !isfinite(yaw_align_angle_deg))
    {
        chassis_cmd_send.offset_angle = 0.0f;
        return;
    }
    float offset = angle - yaw_align_angle_deg;
    if (offset > 180.0f)
        offset -= 360.0f;
    else if (offset < -180.0f)
        offset += 360.0f;
    chassis_cmd_send.offset_angle = offset;
}

#if !defined(MC02_VISION_PROTOCOL_V2)
// 将本轮一起读取的旧协议帧和链路状态配成输入。
static VisionControlInput_s GetVisionControlInput(void)
{
    const VisionControlInput_s input = {
        .online = vision_status_snapshot.online,
        .mode = vision_recv_snapshot.mode,
        .last_rx_ms = vision_status_snapshot.last_rx_ms,
        .now_ms = PlatformTimeNowMs(),
        .max_age_ms = VISION_CONTROL_MAX_AGE_MS,
        .limits = &vision_limits,
        .yaw = vision_recv_snapshot.yaw,
        .yaw_vel = vision_recv_snapshot.yaw_vel,
        .yaw_acc = vision_recv_snapshot.yaw_acc,
        .pitch = vision_recv_snapshot.pitch,
        .pitch_vel = vision_recv_snapshot.pitch_vel,
        .pitch_acc = vision_recv_snapshot.pitch_acc,
    };
    return input;
}
#else
// 瞄准快照只按其接收时间失效，导航帧不能替它续期。
static VisionControlInput_s GetVisionV2ControlInput(void)
{
    const VisionControlInput_s input = {
        .online = (uint8_t)(vision_status_snapshot.online && vision_v2_aim_valid),
        .mode = vision_v2_aim_valid ? vision_v2_aim_snapshot.mode : 0u,
        .last_rx_ms = vision_v2_aim_freshness.received_time_ms,
        .now_ms = PlatformTimeNowMs(),
        .max_age_ms = VISION_V2_AIM_MAX_AGE_MS,
        .limits = &vision_limits,
        .yaw = vision_v2_aim_snapshot.yaw_rad,
        .yaw_vel = vision_v2_aim_snapshot.yaw_rate_rad_s,
        .yaw_acc = vision_v2_aim_snapshot.yaw_accel_rad_s2,
        .pitch = vision_v2_aim_snapshot.pitch_rad,
        .pitch_vel = vision_v2_aim_snapshot.pitch_rate_rad_s,
        .pitch_acc = vision_v2_aim_snapshot.pitch_accel_rad_s2,
    };
    return input;
}

static uint8_t VisionV2NavigationAllowed(uint8_t autonomous_selected)
{
    float max_yaw_rate_deg_s;
    if (autonomous_selected == 0u || vision_status_snapshot.online == 0u ||
        vision_v2_navigation_valid == 0u || RobotSafetyIsReady() == 0u ||
        !GetMaximumChassisYawRateDegS(&max_yaw_rate_deg_s))
        return 0u;

    RefereeSnapshot_t referee;
    if (RefereeGetSnapshot(&referee) != REFEREE_OK)
        return 0u;
    const RefereeCommandSnapshot_t *robot = &referee.command[REF_CMD_ROBOT_STATE];
    const RefereeCommandSnapshot_t *power = &referee.command[REF_CMD_POWER_HEAT];
    return (uint8_t)(robot->state == REFEREE_DATA_FRESH &&
        power->state == REFEREE_DATA_FRESH &&
        (robot->data.robot_state.power_output_bits & REFEREE_OUTPUT_CHASSIS_MASK) != 0u);
}

static void ApplyVisionV2Navigation(void)
{
    const uint8_t flags = vision_v2_navigation_snapshot.valid_flags;
    const float vx_m_s = (flags & VISION_V2_NAV_TRANSLATION_VALID) != 0u ?
                         vision_v2_navigation_snapshot.vx_m_s : 0.0f;
    const float vy_m_s = (flags & VISION_V2_NAV_TRANSLATION_VALID) != 0u ?
                         vision_v2_navigation_snapshot.vy_m_s : 0.0f;
    const float wz_rad_s = (flags & VISION_V2_NAV_YAW_RATE_VALID) != 0u ?
                           vision_v2_navigation_snapshot.yaw_rate_rad_s : 0.0f;
    const float vx_mm_s = vx_m_s * 1000.0f;
    const float vy_mm_s = vy_m_s * 1000.0f;
    const float wz_deg_s = wz_rad_s * RAD_2_DEGREE;
    float max_yaw_rate_deg_s = 0.0f; // 获取配置失败时不会使用未初始化值。
    if (!isfinite(vx_mm_s) || !isfinite(vy_mm_s) || !isfinite(wz_deg_s) ||
        !GetMaximumChassisYawRateDegS(&max_yaw_rate_deg_s) ||
        fabsf(wz_deg_s) > max_yaw_rate_deg_s)
    {
        chassis_cmd_send.vx = 0.0f;
        chassis_cmd_send.vy = 0.0f;
        chassis_cmd_send.wz = 0.0f;
        chassis_cmd_send.chassis_mode = CHASSIS_ZERO_FORCE;
        return;
    }
    chassis_cmd_send.vx = vx_mm_s;
    chassis_cmd_send.vy = vy_mm_s;
    chassis_cmd_send.wz = wz_deg_s;
    chassis_cmd_send.chassis_mode = CHASSIS_NO_FOLLOW;
}

static void ApplyVisionV2GimbalHold(void)
{
    gimbal_cmd_send.gimbal_mode = GIMBAL_GYRO_MODE;
    gimbal_cmd_send.yaw = gimbal_fetch_data.gimbal_imu_data.YawTotalAngle;
    gimbal_cmd_send.pitch = gimbal_fetch_data.gimbal_imu_data.Pitch;
    gimbal_cmd_send.yaw_vel = 0.0f;
    gimbal_cmd_send.yaw_acc = 0.0f;
    gimbal_cmd_send.pitch_vel = 0.0f;
    gimbal_cmd_send.pitch_acc = 0.0f;
}
#endif

// 把已选择的发射请求转换为应用枚举；最终是否允许仍由 EmergencyHandler() 检查。
static void ApplyShootCommand(const VisionControlShootCommand_s *command)
{
    shoot_cmd_send.shoot_mode = command->shoot_enabled ? SHOOT_ON : SHOOT_OFF;
    shoot_cmd_send.friction_mode = command->friction_enabled ? FRICTION_ON : FRICTION_OFF;
    shoot_cmd_send.load_mode = (loader_mode_e)command->load_mode;
    shoot_cmd_send.shoot_rate = command->shoot_rate;
}

// 使用已经换成度、度每秒等单位的视觉结果，云台应用不再解释 SP 原始字节。
static void ApplyVisionGimbal(const VisionControlGimbalCommand_s *command)
{
    gimbal_cmd_send.gimbal_mode = GIMBAL_GYRO_MODE;
    gimbal_cmd_send.yaw = command->yaw;
    gimbal_cmd_send.pitch = command->pitch;
    gimbal_cmd_send.yaw_vel = command->yaw_vel;
    gimbal_cmd_send.yaw_acc = command->yaw_acc;
    gimbal_cmd_send.pitch_vel = command->pitch_vel;
    gimbal_cmd_send.pitch_acc = command->pitch_acc;
}

/**
 * @brief 控制输入为遥控器(调试时)的模式和控制量设置
 *
 */
static void RemoteControlSet(uint8_t vision_active)
{
    chassis_cmd_send.wz = 0.0f;
#if !defined(MC02_CONFIG_GENERATED)
    if (!vision_active)
    {
        // 纯遥控器拨杆控制云台角度增量
        gimbal_cmd_send.yaw += RC_TO_YAW_ANGLE * (float)rc_data[TEMP].rc.rocker_l_;
        gimbal_cmd_send.pitch += RC_TO_PITCH_ANGLE * (float)rc_data[TEMP].rc.rocker_l1;
    }
#endif

    if (switch_is_down(rc_data[TEMP].rc.switch_right)) // 右侧开关向下：选择底盘自转。
    {
        chassis_cmd_send.chassis_mode = CHASSIS_ROTATE;
        if (!vision_active)
            gimbal_cmd_send.gimbal_mode = GIMBAL_GYRO_MODE;
    }
    else if (switch_is_mid(rc_data[TEMP].rc.switch_right)) // 右侧开关状态[中],底盘和云台分离,底盘保持不转动
    {
        chassis_cmd_send.chassis_mode = CHASSIS_NO_FOLLOW;
        if (!vision_active)
            gimbal_cmd_send.gimbal_mode = GIMBAL_GYRO_MODE;
    }

    // 云台软件限位

    // 摇杆读数换成平移目标，当前系数仍需按实际底盘确认。
#if !defined(MC02_CONFIG_GENERATED)
    chassis_cmd_send.vx = 10.0f * (float)rc_data[TEMP].rc.rocker_r_; // _水平方向
    chassis_cmd_send.vy = 10.0f * (float)rc_data[TEMP].rc.rocker_r1; // 1数值方向
#endif

    // 发射参数
    if (switch_is_up(rc_data[TEMP].rc.switch_right)) // 右侧开关状态[上],弹舱打开
    {
        /* 舵机模块尚未接入，本分支保持无动作。 */
    }
    else
    {
        /* 舵机模块尚未接入，本分支保持无动作。 */
    }

    // 摩擦轮控制,拨轮向上打为负,向下为正
    VisionControlShootCommand_s manual_command;
    VisionControlBuildRemoteShootCommand(rc_data[TEMP].rc.dial, &manual_command);
    ApplyShootCommand(&manual_command);
}

/**
 * @brief 输入为键鼠时模式和控制量设置
 *
 */
static void MouseKeySet(uint8_t vision_active, const RemoteMouseBatch_t *mouse)
{
    chassis_cmd_send.wz = 0.0f;
    // 键盘模式直接使用陀螺仪反馈模式控制云台, 避免初值 GIMBAL_ZERO_FORCE 导致云台停电机
    gimbal_cmd_send.gimbal_mode = GIMBAL_GYRO_MODE;
#if !defined(MC02_CONFIG_GENERATED)
    chassis_cmd_send.vx = rc_data[TEMP].key[KEY_PRESS].w * 300 - rc_data[TEMP].key[KEY_PRESS].s * 300; // 系数待测
    chassis_cmd_send.vy = rc_data[TEMP].key[KEY_PRESS].d * 300 - rc_data[TEMP].key[KEY_PRESS].a * 300; // 系数待测, D右移 A左移
#endif

    if (!vision_active && mouse->count != 0u)
    {
        gimbal_cmd_send.yaw += (float)mouse->x / 660 * 10; // 本批有效位移只执行一次，系数待测。
        gimbal_cmd_send.pitch += (float)mouse->y / 660 * 10; // 多份位移已按接收顺序消费。
    }

    switch (rc_data[TEMP].key_count[KEY_PRESS][Key_Z] % 3) // Z键设置弹速
    {
    case 0:
        shoot_cmd_send.bullet_speed = 15;
        break;
    case 1:
        shoot_cmd_send.bullet_speed = 18;
        break;
    default:
        shoot_cmd_send.bullet_speed = 30;
        break;
    }
    switch (rc_data[TEMP].key_count[KEY_PRESS][Key_R] % 2) // R键开关弹舱
    {
    case 0:
        shoot_cmd_send.lid_mode = LID_OPEN;
        break;
    default:
        shoot_cmd_send.lid_mode = LID_CLOSE;
        break;
    }
    VisionControlShootCommand_s manual_command;
    VisionControlBuildMouseKeyShootCommand(
        (uint8_t)(rc_data[TEMP].key_count[KEY_PRESS][Key_E] % 4),
        (uint8_t)(rc_data[TEMP].key_count[KEY_PRESS][Key_F] % 2),
        &manual_command);
    ApplyShootCommand(&manual_command);
    switch (rc_data[TEMP].key_count[KEY_PRESS][Key_C] % 4) // C键设置底盘速度
    {
    case 0:
        chassis_cmd_send.chassis_speed_buff = 40;
        break;
    case 1:
        chassis_cmd_send.chassis_speed_buff = 60;
        break;
    case 2:
        chassis_cmd_send.chassis_speed_buff = 80;
        break;
    default:
        chassis_cmd_send.chassis_speed_buff = 100;
        break;
    }
    switch (rc_data[TEMP].key[KEY_PRESS].shift) // 待添加 按shift允许超功率 消耗缓冲能量
    {
    case 1:

        break;

    default:

        break;
    }
}

#if defined(MC02_CONFIG_GENERATED) && MC02_FEATURE_COMMAND && MC02_FEATURE_REMOTE
// 单一命令任务把已选择人工来源转换为现有应用单位；不会授予运行许可。
static uint8_t ApplyConfiguredManual(uint8_t vision_active)
{
    const uint32_t now = PlatformTimeNowMs(); // 输入期限和积分间隔使用同一时基。
    manual_intent_valid = 0u; // 失败时不能继续发布上一轮的人工来源。
    if (StdCmdAdaptRemote(manual_input_map, &remote_snapshot, now, &manual_intent) != STD_CMD_OK)
    {
        manual_clock_valid = 0u; // 来源无效后必须重新建立积分起点。
        return 0u;
    }
    uint32_t dt_ms = 0u; // 首次接管不积分，以当前姿态建立参考。
    if (manual_clock_valid && manual_reference_ready &&
        manual_source_generation == remote_snapshot.source_selection_generation)
        dt_ms = now - manual_last_ms; // 后续周期使用实际间隔，支持计时回绕。
    RobotManualMotion_t motion = {0}; // 先在局部构造整份新控制量。
    if (!RobotManualMotionProject(&manual_intent, now, dt_ms, manual_max_dt_ms,
        chassis_cmd_send.offset_angle, gimbal_cmd_send.yaw, gimbal_cmd_send.pitch, &motion))
    {
        manual_clock_valid = 0u; // 期限、数值或间隔失败后不得复用参考。
        return 0u;
    }
    if (!vision_active && manual_reference_ready &&
        (motion.pitch_deg < vision_limits.pitch_min_deg || motion.pitch_deg > vision_limits.pitch_max_deg))
    {
        manual_clock_valid = 0u; // 超出车型 pitch 目标范围时不能继续沿用本次参考。
        return 0u;
    }
    manual_last_ms = now; // 只有成功后才推进人工积分时刻。
    manual_source_generation = remote_snapshot.source_selection_generation; // 切档或重连后重新接续。
    manual_clock_valid = 1u; // 后续周期可以计算实测间隔。
    manual_intent_valid = 1u; // 发布前还需再次检查原始期限。
    chassis_cmd_send.vx = motion.vx_mm_s; // 按旧应用消息交付前进速度。
    chassis_cmd_send.vy = motion.vy_right_mm_s; // 显式交付向右为正的横移速度。
    chassis_cmd_send.wz = motion.wz_deg_s; // 当前应用使用度每秒。
    chassis_cmd_send.offset_angle = motion.offset_deg; // 底盘参照不再重复按云台角度转换。
    if (!vision_active && manual_reference_ready)
    {
        gimbal_cmd_send.yaw = motion.yaw_deg; // 仅在人工控制云台时应用积分结果。
        gimbal_cmd_send.pitch = motion.pitch_deg; // 视觉辅助的瞄准目标继续由视觉提供。
    }
    return 1u;
}
#endif

/* 把控制量强制置为失能状态, 用于SAFE/CALIB/FAULT/ESTOP */
static void ApplySafeCommands(void)
{
    memset(&gimbal_cmd_send, 0, sizeof(gimbal_cmd_send));
    manual_reference_ready = 0u;
    gimbal_cmd_send.gimbal_mode = GIMBAL_ZERO_FORCE;
    chassis_cmd_send.chassis_mode = CHASSIS_ZERO_FORCE;
    chassis_cmd_send.vx = 0.0f;
    chassis_cmd_send.vy = 0.0f;
    chassis_cmd_send.wz = 0.0f;
    chassis_cmd_send.offset_angle = 0.0f;
    shoot_cmd_send.shoot_mode = SHOOT_OFF;
    shoot_cmd_send.friction_mode = FRICTION_OFF;
    shoot_cmd_send.load_mode = LOAD_STOP;
    shoot_cmd_send.shoot_rate = 0.0f;
}

/**
 * @brief  安全状态处理与遥控器组合键.
 *         组合键(都需要持续1秒):
 *         - 左右开关同时向下: 请求使能(进入READY)
 *         - 左右开关同时向上: 请求失能(回到SAFE)
 *         - 左右开关同时居中: 清除已锁存的急停
 *         拨轮向下打满(dial > 300)触发手动急停锁存.
 */
static void EmergencyHandler(uint8_t remote_online)
{
    RemoteSafetyGesturesUpdate(&remote_gestures, remote_initialized ? &rc_data[TEMP] : NULL,
                               remote_online, PlatformTimeNowMs(), 1u);
    RobotSafetyUpdate();
    if (!RobotSafetyIsReady()) ApplySafeCommands();
}

/* 机器人核心控制任务,200Hz频率运行(必须高于视觉发送频率) */
void RobotCMDTask()
{
#if defined(MC02_CONFIG_GENERATED) && MC02_FEATURE_COMMAND && MC02_FEATURE_REMOTE
    manual_intent_valid = 0u; // 每轮必须重新验证实际输入。
#endif
    RobotSafetySnapshot_t command_safety = {0}; // 先记录整轮选择命令时所属的安全代际。
    const uint8_t command_safety_valid = RobotSafetyGetSnapshot(&command_safety); // 读取失败时不能发布发射动作。
#if defined(MC02_CONFIG_GENERATED) && MC02_FEATURE_COMMAND && MC02_FEATURE_REMOTE
    if (!command_safety_valid || (manual_safety_generation_seen &&
        manual_safety_generation != command_safety.control_generation))
    {
        manual_clock_valid = 0u; // 周期之间发生短故障时不能跨禁止事件积分。
        manual_reference_ready = 0u; // 新代次必须重新使用实际姿态作为目标起点。
    }
    if (command_safety_valid)
    {
        manual_safety_generation = command_safety.control_generation; // 保存当前控制代次供下轮比较。
        manual_safety_generation_seen = 1u; // 首次读取后零也是有效代次。
    }
#endif
   // 来源离线事件在整车任务中执行，确保撤销遥控许可与视觉清理不挤入监控任务。
   (void)DaemonProcessTaskEvents(DAEMON_CONSUMER_ROBOT_TASK, DAEMON_EVENT_MAX_DEPTH, NULL);
    // 控制选择不读取底盘回传，避免单板复制未使用的反馈或双板为它加锁。
    MessageFreshness_t gimbal_feedback_time = {0}; // 本地保持姿态时也要保留反馈的原始时间。
    const uint8_t feedback_received = (uint8_t)(MessageReadSnapshot(gimbal_feed_sub,
        &gimbal_fetch_data, sizeof(gimbal_fetch_data), PlatformTimeNowMs(), &gimbal_feedback_time) == MESSAGE_OK);

    // 每轮读取一次输入，即使没有新帧也会检查键鼠是否超过使用时间。
    if (remote_initialized) RemoteControlGetSnapshot(&remote_snapshot);
    else memset(&remote_snapshot, 0, sizeof(remote_snapshot));
    RemoteMouseBatch_t mouse_batch = {0}; // 按次位移与持续按键状态分别处理。
    const RemoteMouseStatus_t mouse_status = RemoteControlConsumeMouse(
        remote_snapshot.pc_valid ? remote_snapshot.pc_sample_sequence : 0u, &mouse_batch); // 只消费本次读取以前的事件。

    // 新协议分别复制瞄准与导航快照；彼此的更新不能替对方续期。
#if defined(MC02_VISION_PROTOCOL_V2)
    VisionGetStatus(&vision_status_snapshot);
    const uint32_t vision_now_ms = PlatformTimeNowMs();
    vision_v2_aim_valid = (uint8_t)(MessageReadSnapshot(vision_v2_aim_subscriber,
        &vision_v2_aim_snapshot, sizeof(vision_v2_aim_snapshot), vision_now_ms,
        &vision_v2_aim_freshness) == MESSAGE_OK);
    vision_v2_navigation_valid = (uint8_t)(MessageReadSnapshot(vision_v2_navigation_subscriber,
        &vision_v2_navigation_snapshot, sizeof(vision_v2_navigation_snapshot), vision_now_ms,
        &vision_v2_navigation_freshness) == MESSAGE_OK);
#else
    // SP旧协议帧和链路状态一起复制，避免一份来自更新前、另一份来自更新后。
    VisionGetSnapshot(&vision_recv_snapshot, &vision_status_snapshot);
#endif
    // 没有当前姿态或角度为 NaN、无穷大时，不建立手动参考，先发布停止命令。
    if (!feedback_received || !isfinite(gimbal_fetch_data.gimbal_imu_data.YawTotalAngle) ||
        !isfinite(gimbal_fetch_data.gimbal_imu_data.Pitch))
    {
#if defined(MC02_CONFIG_GENERATED) && MC02_FEATURE_COMMAND && MC02_FEATURE_REMOTE
        manual_clock_valid = 0u; // 失去姿态后不能跨失效间隔积分。
#endif
        manual_reference_ready = 0u;
        previous_vision_active = 0u;
        RobotSafetySetRcOnline(remote_snapshot.remote_online);
        EmergencyHandler(remote_snapshot.remote_online);
#if defined(MC02_CONFIG_GENERATED) && defined(MC02_PROFILE_ROBOT) && defined(ONE_BOARD) && MC02_FEATURE_COMMAND && MC02_FEATURE_REMOTE && MC02_SELECTED_DRIVER_DJI
        (void)RobotCanRecoveryPoll(&remote_snapshot, ROBOT_RECOVERY_KEYBOARD_ALLOWED, NULL); // 应用反馈失效时中断连续等待。
#endif
        ApplySafeCommands();
        VisionSend();
#if defined(ONE_BOARD) || defined(GIMBAL_BOARD)
        (void)MessagePublishNow(chassis_cmd_pub, &chassis_cmd_send);
#endif
        (void)MessagePublishNow(shoot_cmd_pub, &shoot_cmd_send);
        (void)MessagePublishNow(gimbal_cmd_pub, &gimbal_cmd_send);
        return;
    }

    // 根据gimbal的反馈值计算云台和底盘正方向的夹角,不需要传参,通过static私有变量完成
    CalcOffsetAngle();
    const uint8_t remote_online = remote_snapshot.remote_online;
#if defined(MC02_VISION_PROTOCOL_V2)
    const uint8_t autonomous_selected = (uint8_t)(remote_online &&
        switch_is_up(rc_data[TEMP].rc.switch_left) && remote_snapshot.pc_valid);
    VisionControlInput_s vision_input = GetVisionV2ControlInput();
    vision_input.online = (uint8_t)(vision_input.online && autonomous_selected);
#else
    const VisionControlInput_s vision_input = GetVisionControlInput();
#endif
    VisionControlGimbalCommand_s vision_gimbal;
    uint8_t vision_active = VisionControlBuildGimbalCommand(&vision_input, &vision_gimbal);
    if (previous_vision_active && !vision_active)
        memset(&mouse_batch, 0, sizeof(mouse_batch)); // 接管时无法确认积压位移发生在视觉退出前还是之后，先丢弃并以姿态接续。
    RobotSafetySetRcOnline(remote_online);
#if defined(MC02_VISION_PROTOCOL_V2)
    const uint8_t manual_control_active = (uint8_t)(remote_online &&
        switch_is_down(rc_data[TEMP].rc.switch_left));
#else
    const uint8_t manual_control_active =
        (uint8_t)(remote_online && (switch_is_down(rc_data[TEMP].rc.switch_left) ||
                                   (switch_is_up(rc_data[TEMP].rc.switch_left) && remote_snapshot.pc_valid)));
#endif
    gimbal_cmd_send.yaw_vel = gimbal_cmd_send.yaw_acc = 0.0f;
    gimbal_cmd_send.pitch_vel = gimbal_cmd_send.pitch_acc = 0.0f;
    uint8_t manual_allowed = manual_control_active;
#if defined(MC02_CONFIG_GENERATED) && MC02_FEATURE_COMMAND && MC02_FEATURE_REMOTE
    if (!manual_control_active || (previous_vision_active && !vision_active) || (manual_clock_valid &&
        manual_source_generation != remote_snapshot.source_selection_generation))
    {
        manual_clock_valid = 0u; // 切换来源或退出人工模式时停止旧积分。
        manual_reference_ready = 0u; // 下次人工接管以真实姿态建立参考。
    }
#endif
    // 视觉刚退出或首次进入手动时，以当前姿态接续目标，避免跳回上次手动留下的角度。
    if (!vision_active && manual_control_active && (previous_vision_active || !manual_reference_ready))
    {
        if (feedback_received && isfinite(gimbal_fetch_data.gimbal_imu_data.YawTotalAngle) &&
            isfinite(gimbal_fetch_data.gimbal_imu_data.Pitch))
        {
            gimbal_cmd_send.yaw = gimbal_fetch_data.gimbal_imu_data.YawTotalAngle;
            gimbal_cmd_send.pitch = gimbal_fetch_data.gimbal_imu_data.Pitch;
            gimbal_cmd_send.gimbal_mode = GIMBAL_GYRO_MODE;
            manual_reference_ready = 1u;
        }
        else
            manual_allowed = 0u;
    }
    if (manual_allowed)
    {
        if (switch_is_down(rc_data[TEMP].rc.switch_left))
            RemoteControlSet(vision_active);
        else
        {
            MouseKeySet(vision_active, &mouse_batch); // 本轮已消费位移只应用一次。
            if (!vision_active && mouse_status != REMOTE_MOUSE_OK && mouse_status != REMOTE_MOUSE_EMPTY)
            {
                gimbal_cmd_send.gimbal_mode = GIMBAL_ZERO_FORCE; // 位移不完整时本轮撤销云台目标。
                manual_reference_ready = 0u; // 下次人工控制从真实姿态重新接续。
            }
        }
#if defined(MC02_CONFIG_GENERATED) && MC02_FEATURE_COMMAND && MC02_FEATURE_REMOTE
        if (!ApplyConfiguredManual(vision_active))
        {
            manual_allowed = 0u; // 未确认配置、来源过期或计算失败均不得执行旧目标。
            vision_active = 0u; // 人工来源失效时不能在本轮重新应用辅助瞄准。
            ApplySafeCommands(); // 本轮撤销运动和发射请求。
        }
#endif
    }
    else
        ApplySafeCommands();
    if (vision_active)
        ApplyVisionGimbal(&vision_gimbal);
#if defined(MC02_VISION_PROTOCOL_V2)
    else if (autonomous_selected && RobotSafetyIsReady())
        ApplyVisionV2GimbalHold();

    const uint8_t navigation_active = VisionV2NavigationAllowed(autonomous_selected); // 保存本轮实际采用的底盘来源。
    if (navigation_active)
        ApplyVisionV2Navigation();
    else if (autonomous_selected)
    {
        // 自主模式缺少新导航或裁判许可时，清掉上周期的平移和角速度。
        chassis_cmd_send.vx = 0.0f;
        chassis_cmd_send.vy = 0.0f;
        chassis_cmd_send.wz = 0.0f;
        chassis_cmd_send.chassis_mode = CHASSIS_ZERO_FORCE;
    }
#endif

    const VisionControlShootCommand_s manual_shoot_command = {
        .shoot_enabled = (uint8_t)(shoot_cmd_send.shoot_mode == SHOOT_ON),
        .friction_enabled = (uint8_t)(shoot_cmd_send.friction_mode == FRICTION_ON),
        .load_mode = (uint8_t)shoot_cmd_send.load_mode,
        .shoot_rate = shoot_cmd_send.shoot_rate,
    };
    VisionControlShootCommand_s selected_shoot_command;
#if defined(MC02_VISION_PROTOCOL_V2)
    // 新协议没有击发字段；视觉只能给云台目标，发射仍只接收本地手动输入。
    const uint8_t vision_shoot_selected = VisionControlSelectShootCommand(NULL,
                                    manual_control_active ? &manual_shoot_command : NULL,
                                    &selected_shoot_command);
#else
    RobotControlPermits_t shoot_permits = {0}; // 没有完整自主许可时不让视觉取得击发权。
    const uint8_t formal_autonomous = RobotSafetyIsFormalAutonomous(); // 读取安全管理确认的车型自主资格和所选模式。
    const uint8_t vision_fire_allowed = (uint8_t)(formal_autonomous &&
        RobotSafetyGetFunctionPermitsAt(PlatformTimeNowMs(), &shoot_permits) &&
        shoot_permits.fire_allowed && shoot_permits.fire_source == ROBOT_CONTROL_SOURCE_VISION); // 自主击发须有当前有效的独立许可。
    const uint8_t vision_shoot_selected = VisionControlSelectShootCommand(
                                    vision_fire_allowed ? &vision_input : NULL, // 仅正式自主且获许可时使用视觉击发。
                                    (!formal_autonomous && manual_allowed) ? &manual_shoot_command : NULL, // 人工及辅助模式保留人工击发权。
                                    &selected_shoot_command);
#endif
    ApplyShootCommand(&selected_shoot_command);
#if defined(MC02_CONFIG_GENERATED) && MC02_FEATURE_COMMAND && MC02_FEATURE_REMOTE
    if (manual_control_active && !manual_allowed)
        ApplySafeCommands(); // 人工输入失败后，后续来源选择不能重新填入本轮运动或发射目标。
#endif

    EmergencyHandler(remote_online); // 最终安全状态始终覆盖命令选择
#if defined(MC02_CONFIG_GENERATED) && defined(MC02_PROFILE_ROBOT) && defined(ONE_BOARD) && MC02_FEATURE_COMMAND && MC02_FEATURE_REMOTE && MC02_SELECTED_DRIVER_DJI
    if (RobotCanRecoveryPoll(&remote_snapshot, ROBOT_RECOVERY_KEYBOARD_ALLOWED, &gimbal_feedback_time))
        RobotSafetyUpdate(); // 故障解除后推进状态；本轮旧命令已经被安全处理覆盖。
#endif
    RobotControlPermits_t current_permits = {0}; // 默认不授予发射许可。
    uint8_t policy_active = 0u; // 同次查询区分旧整车许可与分功能策略。
    const uint8_t permits_valid = (uint8_t)(command_safety_valid &&
        RobotSafetyGetOutputPermitsForGeneration(PlatformTimeNowMs(), command_safety.control_generation,
                                                  &current_permits, &policy_active)); // 许可和代际必须同时有效。
    chassis_cmd_send.safety_generation = command_safety.control_generation; // 底盘继续检查选择命令前的代际。
    gimbal_cmd_send.safety_generation = command_safety.control_generation; // 云台使用同一轮命令的代际。
#if defined(ONE_BOARD)
    if (!permits_valid || (current_permits.functions & ROBOT_CONTROL_CHASSIS) == 0u)
        chassis_cmd_send.chassis_mode = CHASSIS_ZERO_FORCE; // 本轮未获底盘许可时不发布运动目标。
    if (!permits_valid || (current_permits.functions & ROBOT_CONTROL_GIMBAL) == 0u)
        gimbal_cmd_send.gimbal_mode = GIMBAL_ZERO_FORCE; // 云台同样不能接受失效代际。
#endif
    shoot_cmd_send.safety_generation = command_safety.control_generation; // 发射任务继续检查本轮代际。
    shoot_cmd_send.fire_source = (uint8_t)(vision_shoot_selected ? ROBOT_CONTROL_SOURCE_VISION : ROBOT_CONTROL_SOURCE_MANUAL); // 记录实际选择的击发来源。
    if (!permits_valid || (current_permits.functions & ROBOT_CONTROL_SHOOTER) == 0u)
    {
        shoot_cmd_send.shoot_mode = SHOOT_OFF; // 查询失败、代际变化或机构禁止均关闭发射。
        shoot_cmd_send.friction_mode = FRICTION_OFF; // 不保留受禁止机构的摩擦轮目标。
        shoot_cmd_send.load_mode = LOAD_STOP; // 拨弹目标同时清零。
        shoot_cmd_send.shoot_rate = 0.0f; // 不把旧射速留给下次接管。
    }
    else if (policy_active && (current_permits.fire_allowed == 0u ||
             current_permits.fire_source != (RobotControlSource_t)shoot_cmd_send.fire_source))
    {
        shoot_cmd_send.load_mode = LOAD_STOP; // 机构许可不等于允许本次击发。
        shoot_cmd_send.shoot_rate = 0.0f; // 击发来源不匹配时撤销旧射速。
    }
    previous_vision_active = (uint8_t)(vision_active && RobotSafetyIsReady());

    // 设置视觉发送数据(四元数+yaw/pitch+弹速+弹计数), 通过 VisionUpdateTx + VisionSend 回传上位机
    // VisionUpdateTx 由 ins_task 完成姿态解算后填充, 这里只负责发送
    VisionSend();

    const uint32_t publish_ms = PlatformTimeNowMs(); // 本轮分别保留来源和发布时刻。
    RobotCmdLease_t radio = {.remaining_ms = UINT32_MAX, .valid = 1u}; // 所有现有模式均依赖无线选择和使能。
    RobotCmdLeaseInclude(&radio, remote_online, remote_snapshot.radio_source_time_ms,
                         REMOTE_RADIO_LEASE_MS, publish_ms);
    RobotCmdLease_t manual = radio; // 键鼠模式还依赖独立的 PC 心跳。
    if (switch_is_up(rc_data[TEMP].rc.switch_left))
        RobotCmdLeaseInclude(&manual, remote_snapshot.pc_valid, remote_snapshot.pc_source_time_ms,
                             REMOTE_PC_LEASE_MS, publish_ms);
#if defined(MC02_CONFIG_GENERATED) && MC02_FEATURE_COMMAND && MC02_FEATURE_REMOTE
    if (manual_control_active)
        RobotCmdLeaseInclude(&manual, manual_intent_valid, manual_intent.source_time_ms,
                             manual_intent.max_age_ms, publish_ms); // 配置期限不能因转成旧消息而丢失。
#endif
    RobotCmdLease_t manual_gimbal = manual; // 本轮若使用了鼠标事件，还须检查最早事件的期限。
    if (manual_allowed && !vision_active && switch_is_up(rc_data[TEMP].rc.switch_left) && mouse_batch.count != 0u)
        RobotCmdLeaseInclude(&manual_gimbal, mouse_status == REMOTE_MOUSE_OK, mouse_batch.oldest_source_ms,
                             REMOTE_PC_LEASE_MS, publish_ms);
    RobotCmdLease_t vision = radio; // 视觉目标仍依赖本轮实际选择及自身接收时间。
#if defined(MC02_VISION_PROTOCOL_V2)
    RobotCmdLeaseInclude(&vision, vision_active, vision_v2_aim_freshness.received_time_ms,
                         VISION_V2_AIM_MAX_AGE_MS, publish_ms);
    RobotCmdLease_t navigation = manual; // 现有 V2 自主选择还要求无线和键鼠状态有效。
    RobotCmdLeaseInclude(&navigation, navigation_active, vision_v2_navigation_freshness.received_time_ms,
                         VISION_V2_NAV_MAX_AGE_MS, publish_ms);
    RobotCmdLease_t local_hold = manual; // 失去瞄准后的本地保持依赖新鲜云台反馈。
    RobotCmdLeaseInclude(&local_hold, feedback_received, gimbal_feedback_time.source_time_ms,
                         MESSAGE_DEFAULT_MAX_AGE_MS, publish_ms);
#else
    RobotCmdLeaseInclude(&vision, vision_active, vision_status_snapshot.last_rx_ms,
                         VISION_CONTROL_MAX_AGE_MS, publish_ms);
#endif
    // 停止命令是本机新决定；运动目标需要保留实际来源期限。
#if defined(ONE_BOARD) || defined(GIMBAL_BOARD)
    if (chassis_cmd_send.chassis_mode == CHASSIS_ZERO_FORCE)
        (void)MessagePublishNow(chassis_cmd_pub, &chassis_cmd_send);
    else
#if defined(MC02_VISION_PROTOCOL_V2)
        (void)RobotCmdPublishBounded(chassis_cmd_pub, &chassis_cmd_send, navigation_active ? &navigation : &manual, publish_ms);
#else
        (void)RobotCmdPublishBounded(chassis_cmd_pub, &chassis_cmd_send, &manual, publish_ms);
#endif
#endif // ONE_BOARD
    if (shoot_cmd_send.shoot_mode == SHOOT_OFF && shoot_cmd_send.friction_mode == FRICTION_OFF && shoot_cmd_send.load_mode == LOAD_STOP)
        (void)MessagePublishNow(shoot_cmd_pub, &shoot_cmd_send);
    else
        (void)RobotCmdPublishBounded(shoot_cmd_pub, &shoot_cmd_send, vision_shoot_selected ? &vision : &manual, publish_ms);
    if (gimbal_cmd_send.gimbal_mode == GIMBAL_ZERO_FORCE)
        (void)MessagePublishNow(gimbal_cmd_pub, &gimbal_cmd_send);
    else
#if defined(MC02_VISION_PROTOCOL_V2)
        if (!RobotCmdPublishBounded(gimbal_cmd_pub, &gimbal_cmd_send,
            vision_active ? &vision : (autonomous_selected ? &local_hold : &manual_gimbal), publish_ms))
            manual_reference_ready = 0u; // 本轮未交付的内部角度不能在下一轮复用。
#else
        if (!RobotCmdPublishBounded(gimbal_cmd_pub, &gimbal_cmd_send, vision_active ? &vision : &manual_gimbal, publish_ms))
            manual_reference_ready = 0u; // 恢复后必须从实时姿态重新接续。
#endif
}
