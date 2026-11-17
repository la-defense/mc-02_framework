/** @file gimbal.c
* @brief 接收云台目标，设置 yaw、pitch 的电机参考和反馈来源，并回传姿态及电机单圈角度。
*/
#include "gimbal.h"
#include "robot_def.h"
#include "config.h"
#include "dji_motor.h"
#include "ins_task.h"
#include "message_center.h"
#include "topics/robot_topics.h"
#include "general_def.h"
#include "platform_sync.h"
#include <string.h>

static attitude_t *gimba_IMU_data; // 云台IMU数据
static DJIMotorInstance *yaw_motor, *pitch_motor;
static const Gimbal_Config_s *gimbal_config;

typedef enum
{
    GIMBAL_SOURCE_YAW_POSITION = 0,
    GIMBAL_SOURCE_YAW_SPEED,
    GIMBAL_SOURCE_PITCH_POSITION,
    GIMBAL_SOURCE_PITCH_SPEED,
} GimbalSourceField_e;

typedef struct
{
    GimbalSourceField_e field;
} GimbalSourceContext_s;

static const GimbalSourceContext_s yaw_position_context = {GIMBAL_SOURCE_YAW_POSITION};
static const GimbalSourceContext_s yaw_speed_context = {GIMBAL_SOURCE_YAW_SPEED};
static const GimbalSourceContext_s pitch_position_context = {GIMBAL_SOURCE_PITCH_POSITION};
static const GimbalSourceContext_s pitch_speed_context = {GIMBAL_SOURCE_PITCH_SPEED};

static Motor_Feedback_Status_e ReadGimbalSignal(const void *context,
                                                Motor_Signal_Snapshot_s *snapshot)
{
    if (context == NULL || snapshot == NULL)
        return MOTOR_FEEDBACK_BAD_ARGUMENT;

    INS_Control_Snapshot_s ins;
    if (INSGetControlSnapshot(&ins) == 0u || ins.sequence == 0u)
        return MOTOR_FEEDBACK_NOT_READY;

    const GimbalSourceContext_s *source = (const GimbalSourceContext_s *)context;
    Motor_Signal_Snapshot_s candidate = {
        .reference = MOTOR_REFERENCE_EXTERNAL,
        .source_time_ms = ins.timestamp_ms,
        .sequence = ins.sequence,
        .valid = ins.valid,
    };
    switch (source->field)
    {
    case GIMBAL_SOURCE_YAW_POSITION:
        candidate.value = ins.yaw_total_rad;
        candidate.unit = MOTOR_UNIT_RAD;
        break;
    case GIMBAL_SOURCE_YAW_SPEED:
        candidate.value = ins.gyro_rad_s[Z];
        candidate.unit = MOTOR_UNIT_RAD_S;
        break;
    case GIMBAL_SOURCE_PITCH_POSITION:
        candidate.value = ins.pitch_rad;
        candidate.unit = MOTOR_UNIT_RAD;
        break;
    case GIMBAL_SOURCE_PITCH_SPEED:
        candidate.value = ins.gyro_rad_s[X];
        candidate.unit = MOTOR_UNIT_RAD_S;
        break;
    default:
        return MOTOR_FEEDBACK_BAD_CONFIG;
    }
    *snapshot = candidate;
    return MOTOR_FEEDBACK_OK;
}

static MessagePublisher_t *gimbal_pub;             // 云台应用消息发布者(云台反馈给cmd)
static MessageSubscriber_t *gimbal_sub;            // cmd控制消息订阅者
static Gimbal_Upload_Data_s gimbal_feedback_data; // 回传给cmd的云台状态信息
static Gimbal_Ctrl_Cmd_s gimbal_cmd_recv;         // 来自cmd的控制信息
static uint8_t gimbal_rollback_failed; // 记录启动资源无法撤销的故障。

uint8_t GimbalInitRollbackFailed(void)
{
    return gimbal_rollback_failed; // 启动协调任务读取最近一次回退结果。
}

uint8_t GimbalInit(void)
{   
    gimbal_rollback_failed = 0u; // 每次初始化从无回退故障开始。
    if (!RobotTopicsInit())
        return 0u;
    gimbal_config = GimbalConfigFeed();
    if (gimbal_config == NULL || gimbal_config->gimbal_type != SINGLE_GIMBAL)
        return 0u;
    attitude_t *pending_imu = INS_GetStorage(); // INS 存储必须存在，姿态初始化由协调任务继续执行。
    if (pending_imu == NULL) return 0u; // 未取得存储时不注册任何电机。
    Motor_Init_Config_s yaw_config = gimbal_config->yaw_motor_config;
    Motor_Init_Config_s pitch_config = gimbal_config->pitch_motor_config;
    yaw_config.output_role = MOTOR_OUTPUT_GIMBAL; // 大 yaw 使用云台功能许可。
    pitch_config.output_role = MOTOR_OUTPUT_GIMBAL; // pitch 同样使用云台许可。
    yaw_config.controller_param_init_config.other_angle_feedback =
        (Motor_Signal_Source_s){ReadGimbalSignal, &yaw_position_context,
                                5u, MOTOR_UNIT_RAD, MOTOR_REFERENCE_EXTERNAL};
    yaw_config.controller_param_init_config.other_speed_feedback =
        (Motor_Signal_Source_s){ReadGimbalSignal, &yaw_speed_context,
                                5u, MOTOR_UNIT_RAD_S, MOTOR_REFERENCE_EXTERNAL};
    pitch_config.controller_param_init_config.other_angle_feedback =
        (Motor_Signal_Source_s){ReadGimbalSignal, &pitch_position_context,
                                5u, MOTOR_UNIT_RAD, MOTOR_REFERENCE_EXTERNAL};
    pitch_config.controller_param_init_config.other_speed_feedback =
        (Motor_Signal_Source_s){ReadGimbalSignal, &pitch_speed_context,
                                5u, MOTOR_UNIT_RAD_S, MOTOR_REFERENCE_EXTERNAL};
    yaw_config.controller_setting_init_config.angle_feedback_source = OTHER_FEED;
    yaw_config.controller_setting_init_config.speed_feedback_source = OTHER_FEED;
    pitch_config.controller_setting_init_config.angle_feedback_source = OTHER_FEED;
    pitch_config.controller_setting_init_config.speed_feedback_source = OTHER_FEED;
    // 这里只注册电机；实际是否运行由周期命令中的模式决定。
    DJIMotorInstance *pending_yaw = DJIMotorInit(&yaw_config); // 先暂存大 yaw 实例。
    if (pending_yaw == NULL) return 0u; // 第一台失败时没有已取得的电机。
    DJIMotorInstance *pending_pitch = DJIMotorInit(&pitch_config); // 再申请 pitch 实例。
    if (pending_pitch == NULL) goto rollback; // 第二台失败时撤销第一台。
    MessagePublisher_t *pending_pub = NULL; // 准备一次提交的发布句柄。
    MessageSubscriber_t *pending_sub = NULL; // 准备一次提交的订阅句柄。
    if (MessageEndpointPairRegister(ROBOT_TOPIC_GIMBAL_FEEDBACK, ROBOT_TOPIC_GIMBAL_COMMAND,
                                    ROBOT_TOPIC_ENDPOINT_GIMBAL, &pending_pub, &pending_sub) != MESSAGE_OK)
        goto rollback; // 任一话题不足时两个端点均未新增。
    gimba_IMU_data = pending_imu; // 全部成功后发布姿态存储。
    yaw_motor = pending_yaw; // 发布 yaw 电机。
    pitch_motor = pending_pitch; // 发布 pitch 电机。
    gimbal_pub = pending_pub; // 发布反馈话题句柄。
    gimbal_sub = pending_sub; // 发布控制话题句柄。
    return 1u; // 启动计划可接受云台模块。

rollback:
    if (pending_pitch != NULL && !DJIMotorRollbackStartup(pending_pitch))
        gimbal_rollback_failed = 1u; // pitch 无法撤销时必须通知整车。
    if (!DJIMotorRollbackStartup(pending_yaw))
        gimbal_rollback_failed = 1u; // 即使前一步失败也尽力撤销 yaw。
    return 0u; // 不交付未完成的云台模块。
}

uint8_t GimbalReleaseStartupDevices(void)
{
    uint8_t released = 1u; // 即使一个撤销失败，也继续处理另一个实例。
    if (pitch_motor != NULL && !DJIMotorRollbackStartup(pitch_motor)) released = 0u; // 先撤销后登记的 pitch。
    pitch_motor = NULL; // 底层失败时也可能已释放软件实例，不保留旧指针。
    if (yaw_motor != NULL && !DJIMotorRollbackStartup(yaw_motor)) released = 0u; // 再撤销 yaw。
    yaw_motor = NULL; // 后续诊断不能再解引用原实例。
    return released; // 无法撤销时由协调者禁止任务放行。
}

/* 机器人云台控制核心任务,后续考虑只保留IMU控制,不再需要电机的反馈 */
void GimbalTask()
{
    // 只采用不超过 50 ms 的控制消息；失败清零后进入停机分支。
    if (MessageReadSnapshot(gimbal_sub, &gimbal_cmd_recv, sizeof(gimbal_cmd_recv),
                            PlatformTimeNowMs(), NULL) != MESSAGE_OK)
        memset(&gimbal_cmd_recv, 0, sizeof(gimbal_cmd_recv));

    // 闭环反馈使用外部 IMU；上传给上层的状态仍包含 yaw 电机单圈角度。
    // GYRO 与 FREE 当前使用同一套外部 IMU 反馈；ZERO_FORCE 和未知模式停止输出。
    switch (gimbal_cmd_recv.gimbal_mode)
    {
    // 停止
    case GIMBAL_ZERO_FORCE:
        DJIMotorStop(yaw_motor);
        DJIMotorStop(pitch_motor);
        DJIMotorSetRef(yaw_motor, 0.0f);
        DJIMotorSetRef(pitch_motor, 0.0f);
        break;
    // yaw、pitch 命令以度接收，按电机配置的弧度参考转换后送入闭环。
    case GIMBAL_GYRO_MODE:
        DJIMotorEnable(yaw_motor);
        DJIMotorEnable(pitch_motor);
        DJIMotorChangeFeed(yaw_motor, ANGLE_LOOP, OTHER_FEED);
        DJIMotorChangeFeed(yaw_motor, SPEED_LOOP, OTHER_FEED);
        DJIMotorChangeFeed(pitch_motor, ANGLE_LOOP, OTHER_FEED);
        DJIMotorChangeFeed(pitch_motor, SPEED_LOOP, OTHER_FEED);
#if defined(ONE_BOARD)
        if (!DJIMotorSetRefForGeneration(yaw_motor, gimbal_cmd_recv.yaw * DEGREE_2_RAD, gimbal_cmd_recv.safety_generation) ||
            !DJIMotorSetRefForGeneration(pitch_motor, gimbal_cmd_recv.pitch * DEGREE_2_RAD, gimbal_cmd_recv.safety_generation))
        {
            DJIMotorStop(yaw_motor); // 任一轴未获许可时停止两轴。
            DJIMotorStop(pitch_motor); // 避免只执行部分目标。
        }
#else
        DJIMotorSetRef(yaw_motor, gimbal_cmd_recv.yaw * DEGREE_2_RAD);
        DJIMotorSetRef(pitch_motor, gimbal_cmd_recv.pitch * DEGREE_2_RAD);
#endif
        break;
    // FREE 目前与 GYRO 行为相同，尚未实现独立的底盘跟随控制。
    case GIMBAL_FREE_MODE:
        DJIMotorEnable(yaw_motor);
        DJIMotorEnable(pitch_motor);
        DJIMotorChangeFeed(yaw_motor, ANGLE_LOOP, OTHER_FEED);
        DJIMotorChangeFeed(yaw_motor, SPEED_LOOP, OTHER_FEED);
        DJIMotorChangeFeed(pitch_motor, ANGLE_LOOP, OTHER_FEED);
        DJIMotorChangeFeed(pitch_motor, SPEED_LOOP, OTHER_FEED);
#if defined(ONE_BOARD)
        if (!DJIMotorSetRefForGeneration(yaw_motor, gimbal_cmd_recv.yaw * DEGREE_2_RAD, gimbal_cmd_recv.safety_generation) ||
            !DJIMotorSetRefForGeneration(pitch_motor, gimbal_cmd_recv.pitch * DEGREE_2_RAD, gimbal_cmd_recv.safety_generation))
        {
            DJIMotorStop(yaw_motor); // 任一轴未获许可时停止两轴。
            DJIMotorStop(pitch_motor); // 避免只执行部分目标。
        }
#else
        DJIMotorSetRef(yaw_motor, gimbal_cmd_recv.yaw * DEGREE_2_RAD);
        DJIMotorSetRef(pitch_motor, gimbal_cmd_recv.pitch * DEGREE_2_RAD);
#endif
        break;
    default:
        DJIMotorStop(yaw_motor);
        DJIMotorStop(pitch_motor);
        DJIMotorSetRef(yaw_motor, 0.0f);
        DJIMotorSetRef(pitch_motor, 0.0f);
        break;
    }

    // 在合适的地方添加pitch重力补偿前馈力矩
    // 根据IMU姿态/pitch电机角度反馈计算出当前配重下的重力矩
    // ...

    // 设置反馈数据,主要是imu和yaw的ecd
    gimbal_feedback_data.gimbal_imu_data = *gimba_IMU_data;
    gimbal_feedback_data.yaw_motor_single_round_angle = yaw_motor->measure.angle_single_round;

    // 推送消息
    (void)MessagePublishNow(gimbal_pub, &gimbal_feedback_data);
}

void Gimbal_GetMotorSummary(Gimbal_Motor_Summary_t *summary)
{
    if (summary == NULL)
        return;

    *summary = (Gimbal_Motor_Summary_t){0};

    DJIMotorGetSummary(yaw_motor, &summary->yaw);
    DJIMotorGetSummary(pitch_motor, &summary->pitch);
}
