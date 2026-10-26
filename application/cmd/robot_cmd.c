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
#include "bmi088.h"
#include "robot_safety.h"
#include "daemon.h"
#include "vision_control.h"
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
#include "bsp_dwt.h"
#include "bsp_log.h"

// 私有宏,自动将编码器转换成角度值
#define YAW_ALIGN_ANGLE (YAW_CHASSIS_ALIGN_ECD * ECD_ANGLE_COEF_DJI) // 对齐时的角度,0-360
#define PTICH_HORIZON_ANGLE (PITCH_HORIZON_ECD * ECD_ANGLE_COEF_DJI) // pitch水平时电机的角度,0-360

static const VisionControlLimits_s vision_limits = {
    .configured = VISION_LIMITS_CONFIGURED,
    .yaw_constraint = VISION_YAW_CONSTRAINT,
    .pitch_min_deg = PITCH_MIN_ANGLE,
    .pitch_max_deg = PITCH_MAX_ANGLE,
    .pitch_zero_offset_deg = VISION_PITCH_ZERO_OFFSET_DEG,
    .yaw_min_deg = VISION_YAW_MIN_DEG,
    .yaw_max_deg = VISION_YAW_MAX_DEG,
};
static uint8_t previous_vision_active;
static uint8_t manual_reference_ready;

/* cmd应用包含的模块实例指针和交互信息存储*/
#ifdef GIMBAL_BOARD // 对双板的兼容,条件编译
#include "can_comm.h"
static CANCommInstance *cmd_can_comm; // 双板通信
#endif
#ifdef ONE_BOARD
static MessagePublisher_t *chassis_cmd_pub; // 底盘控制消息发布者
#endif                                 // ONE_BOARD

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

static MessagePublisher_t *shoot_cmd_pub;           // 发射控制消息发布者
static Shoot_Ctrl_Cmd_s shoot_cmd_send;      // 传递给发射的控制信息

BMI088Instance *bmi088_test; // 云台IMU
BMI088_Data_t bmi088_data;
uint8_t RobotCMDInit(void)
{
    if (!RobotTopicsInit())
        return 0u;
    RemoteSafetyGesturesInit(&remote_gestures);
    remote_initialized = (uint8_t)(RemoteControlInit(&huart5) != NULL);
    if (!remote_initialized || VisionInit(&huart9) == NULL)
        return 0u;
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
#ifdef ONE_BOARD // 双板兼容
    chassis_cmd_pub = MessagePublisherRegister(ROBOT_TOPIC_CHASSIS_COMMAND,
                                                ROBOT_TOPIC_ENDPOINT_COMMAND);
#endif // ONE_BOARD
#ifdef GIMBAL_BOARD
    CANComm_Init_Config_s comm_conf = {
        .can_config = {
            .can_handle = &hcan1,
            .tx_id = 0x312,
            .rx_id = 0x311,
        },
        .recv_data_len = sizeof(Chassis_Upload_Data_s),
        .send_data_len = sizeof(Chassis_Ctrl_Cmd_s),
    };
    cmd_can_comm = CANCommInit(&comm_conf);
    if (cmd_can_comm == NULL)
        return 0u;
#endif // GIMBAL_BOARD
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
#ifdef ONE_BOARD
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
    // 单圈角度与归中位置相减，跨过一圈边界时补减 360 度。
    static float angle;
    angle = gimbal_fetch_data.yaw_motor_single_round_angle; // 从云台获取的当前yaw电机单圈角度
#if YAW_ECD_GREATER_THAN_4096                               // 如果大于180度
    if (angle > YAW_ALIGN_ANGLE && angle <= 180.0f + YAW_ALIGN_ANGLE)
        chassis_cmd_send.offset_angle = angle - YAW_ALIGN_ANGLE;
    else if (angle > 180.0f + YAW_ALIGN_ANGLE)
        chassis_cmd_send.offset_angle = angle - YAW_ALIGN_ANGLE - 360.0f;
    else
        chassis_cmd_send.offset_angle = angle - YAW_ALIGN_ANGLE;
#else // 小于180度
    if (angle > YAW_ALIGN_ANGLE)
        chassis_cmd_send.offset_angle = angle - YAW_ALIGN_ANGLE;
    else if (angle <= YAW_ALIGN_ANGLE && angle >= YAW_ALIGN_ANGLE - 180.0f)
        chassis_cmd_send.offset_angle = angle - YAW_ALIGN_ANGLE;
    else
        chassis_cmd_send.offset_angle = angle - YAW_ALIGN_ANGLE + 360.0f;
#endif
}

#if !defined(MC02_VISION_PROTOCOL_V2)
// 将本轮一起读取的旧协议帧和链路状态配成输入。
static VisionControlInput_s GetVisionControlInput(void)
{
    const VisionControlInput_s input = {
        .online = vision_status_snapshot.online,
        .mode = vision_recv_snapshot.mode,
        .last_rx_ms = vision_status_snapshot.last_rx_ms,
        .now_ms = HAL_GetTick(),
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
        .now_ms = HAL_GetTick(),
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
    if (autonomous_selected == 0u || vision_status_snapshot.online == 0u ||
        vision_v2_navigation_valid == 0u || RobotSafetyIsReady() == 0u ||
        CHASSIS_MOTION_LIMITS_CONFIGURED == 0u)
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
    if (!isfinite(vx_mm_s) || !isfinite(vy_mm_s) || !isfinite(wz_deg_s) ||
        fabsf(wz_deg_s) > CHASSIS_MAX_WZ_DEG_S)
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
    if (!vision_active)
    {
        // 纯遥控器拨杆控制云台角度增量
        gimbal_cmd_send.yaw += RC_TO_YAW_ANGLE * (float)rc_data[TEMP].rc.rocker_l_;
        gimbal_cmd_send.pitch += RC_TO_PITCH_ANGLE * (float)rc_data[TEMP].rc.rocker_l1;
    }

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
    chassis_cmd_send.vx = 10.0f * (float)rc_data[TEMP].rc.rocker_r_; // _水平方向
    chassis_cmd_send.vy = 10.0f * (float)rc_data[TEMP].rc.rocker_r1; // 1数值方向

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
static void MouseKeySet(uint8_t vision_active)
{
    chassis_cmd_send.wz = 0.0f;
    // 键盘模式直接使用陀螺仪反馈模式控制云台, 避免初值 GIMBAL_ZERO_FORCE 导致云台停电机
    gimbal_cmd_send.gimbal_mode = GIMBAL_GYRO_MODE;
    chassis_cmd_send.vx = rc_data[TEMP].key[KEY_PRESS].w * 300 - rc_data[TEMP].key[KEY_PRESS].s * 300; // 系数待测
    chassis_cmd_send.vy = rc_data[TEMP].key[KEY_PRESS].d * 300 - rc_data[TEMP].key[KEY_PRESS].a * 300; // 系数待测, D右移 A左移

    if (!vision_active)
    {
        gimbal_cmd_send.yaw += (float)rc_data[TEMP].mouse.x / 660 * 10; // 系数待测
        gimbal_cmd_send.pitch += (float)rc_data[TEMP].mouse.y / 660 * 10;
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
                               remote_online, HAL_GetTick(), 1u);
    RobotSafetyUpdate();
    if (!RobotSafetyIsReady()) ApplySafeCommands();
}

/* 机器人核心控制任务,200Hz频率运行(必须高于视觉发送频率) */
void RobotCMDTask()
{
   // 来源离线事件在整车任务中执行，确保撤销遥控许可与视觉清理不挤入监控任务。
   (void)DaemonProcessTaskEvents(DAEMON_CONSUMER_ROBOT_TASK, DAEMON_EVENT_MAX_DEPTH, NULL);
   // BMI088Acquire(bmi088_test,&bmi088_data) ;
    // 控制选择不读取底盘回传；避免单板消息中心复制或双板 CAN 快照锁定。
    const uint8_t feedback_received = (uint8_t)(MessageReadSnapshot(gimbal_feed_sub,
        &gimbal_fetch_data, sizeof(gimbal_fetch_data), HAL_GetTick(), NULL) == MESSAGE_OK);

    // 每轮读取一次输入，即使没有新帧也会检查键鼠是否超过使用时间。
    if (remote_initialized) RemoteControlGetSnapshot(&remote_snapshot);
    else memset(&remote_snapshot, 0, sizeof(remote_snapshot));

    // 新协议分别复制瞄准与导航快照；彼此的更新不能替对方续期。
#if defined(MC02_VISION_PROTOCOL_V2)
    VisionGetStatus(&vision_status_snapshot);
    const uint32_t vision_now_ms = HAL_GetTick();
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
        manual_reference_ready = 0u;
        previous_vision_active = 0u;
        RobotSafetySetRcOnline(remote_snapshot.remote_online);
        EmergencyHandler(remote_snapshot.remote_online);
        ApplySafeCommands();
        VisionSend();
#ifdef ONE_BOARD
        (void)MessagePublishNow(chassis_cmd_pub, &chassis_cmd_send);
#endif
#ifdef GIMBAL_BOARD
        CANCommSend(cmd_can_comm, (void *)&chassis_cmd_send);
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
    const uint8_t vision_active = VisionControlBuildGimbalCommand(&vision_input, &vision_gimbal);
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
            MouseKeySet(vision_active);
    }
    else
        ApplySafeCommands();
    if (vision_active)
        ApplyVisionGimbal(&vision_gimbal);
#if defined(MC02_VISION_PROTOCOL_V2)
    else if (autonomous_selected && RobotSafetyIsReady())
        ApplyVisionV2GimbalHold();

    if (VisionV2NavigationAllowed(autonomous_selected))
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
    VisionControlSelectShootCommand(NULL,
                                    manual_control_active ? &manual_shoot_command : NULL,
                                    &selected_shoot_command);
#else
    VisionControlSelectShootCommand(&vision_input,
                                    manual_allowed ? &manual_shoot_command : NULL,
                                    &selected_shoot_command);
#endif
    ApplyShootCommand(&selected_shoot_command);

    EmergencyHandler(remote_online); // 最终安全状态始终覆盖命令选择
    previous_vision_active = (uint8_t)(vision_active && RobotSafetyIsReady());

    // 设置视觉发送数据(四元数+yaw/pitch+弹速+弹计数), 通过 VisionUpdateTx + VisionSend 回传上位机
    // VisionUpdateTx 由 ins_task 完成姿态解算后填充, 这里只负责发送
    VisionSend();

    // 推送消息,双板通信等
    // 其他应用所需的控制数据在remotecontrolsetmode和mousekeysetmode中完成设置
#ifdef ONE_BOARD
    (void)MessagePublishNow(chassis_cmd_pub, &chassis_cmd_send);
#endif // ONE_BOARD
#ifdef GIMBAL_BOARD
    CANCommSend(cmd_can_comm, (void *)&chassis_cmd_send);
#endif // GIMBAL_BOARD
    (void)MessagePublishNow(shoot_cmd_pub, &shoot_cmd_send);
    (void)MessagePublishNow(gimbal_cmd_pub, &gimbal_cmd_send);
}
