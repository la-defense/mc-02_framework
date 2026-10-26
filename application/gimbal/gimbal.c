/** @file gimbal.c
* @brief 接收云台目标，设置 yaw、pitch 的电机参考和反馈来源，并回传姿态及电机单圈角度。
*/
#include "gimbal.h"
#include "robot_def.h"
#include "dji_motor.h"
#include "ins_task.h"
#include "message_center.h"
#include "topics/robot_topics.h"
#include "general_def.h"
#include "main.h"
#include <string.h>

static attitude_t *gimba_IMU_data; // 云台IMU数据
static DJIMotorInstance *yaw_motor, *pitch_motor;

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

uint8_t GimbalInit(void)
{   
    if (!RobotTopicsInit())
        return 0u;
    gimba_IMU_data = INS_GetStorage(); // INS 由调度器启动后的专属任务初始化
    // YAW
    Motor_Init_Config_s yaw_config = {
        .can_init_config = {
            .can_handle = &hcan3,
            .tx_id = 1,
        },
        .controller_param_init_config = {
            .angle_PID = {
                .DtMin_s = MOTOR_PID_DT_MIN_S,
                .DtMax_s = MOTOR_PID_DT_MAX_S,
                .Kp = 8, // rad/s per rad; gains retain their ratio when both angle axes use radians.
                .Ki = 0,
                .Kd = 0,
                .DeadBand = 0.1f * DEGREE_2_RAD,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .IntegralLimit = 100.0f * DEGREE_2_RAD,

                .MaxOut = 500.0f * DEGREE_2_RAD,
            },
            .speed_PID = {
                .DtMin_s = MOTOR_PID_DT_MIN_S,
                .DtMax_s = MOTOR_PID_DT_MAX_S,
                .Kp = 50,  // 50
                .Ki = 200, // 200
                .Kd = 0,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .IntegralLimit = 3000,
                .MaxOut = 20000,
            },
            .other_angle_feedback = {
                .read = ReadGimbalSignal, .context = &yaw_position_context, .max_age_ms = 5u,
                .unit = MOTOR_UNIT_RAD, .reference = MOTOR_REFERENCE_EXTERNAL,
            },
            .other_speed_feedback = {
                .read = ReadGimbalSignal, .context = &yaw_speed_context, .max_age_ms = 5u,
                .unit = MOTOR_UNIT_RAD_S, .reference = MOTOR_REFERENCE_EXTERNAL,
            },
        },
        .controller_setting_init_config = {
            .angle_feedback_source = OTHER_FEED,
            .speed_feedback_source = OTHER_FEED,
            .outer_loop_type = ANGLE_LOOP,
            .close_loop_type = ANGLE_LOOP | SPEED_LOOP,
            .motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
        },
        .output_axis = {1.0f, MOTOR_AXIS_DIRECTION_NORMAL},
        .motor_type = GM6020};
    // PITCH
    Motor_Init_Config_s pitch_config = {
        .can_init_config = {
            .can_handle = &hcan3,
            .tx_id = 2,
        },
        .controller_param_init_config = {
            .angle_PID = {
                .DtMin_s = MOTOR_PID_DT_MIN_S,
                .DtMax_s = MOTOR_PID_DT_MAX_S,
                .Kp = 10,
                .Ki = 0,
                .Kd = 0,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .IntegralLimit = 100.0f * DEGREE_2_RAD,
                .MaxOut = 500.0f * DEGREE_2_RAD,
            },
            .speed_PID = {
                .DtMin_s = MOTOR_PID_DT_MIN_S,
                .DtMax_s = MOTOR_PID_DT_MAX_S,
                .Kp = 50,  // 50
                .Ki = 350, // 350
                .Kd = 0,   // 0
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .IntegralLimit = 2500,
                .MaxOut = 20000,
            },
            .other_angle_feedback = {
                .read = ReadGimbalSignal, .context = &pitch_position_context, .max_age_ms = 5u,
                .unit = MOTOR_UNIT_RAD, .reference = MOTOR_REFERENCE_EXTERNAL,
            },
            .other_speed_feedback = {
                .read = ReadGimbalSignal, .context = &pitch_speed_context, .max_age_ms = 5u,
                .unit = MOTOR_UNIT_RAD_S, .reference = MOTOR_REFERENCE_EXTERNAL,
            },
        },
        .controller_setting_init_config = {
            .angle_feedback_source = OTHER_FEED,
            .speed_feedback_source = OTHER_FEED,
            .outer_loop_type = ANGLE_LOOP,
            .close_loop_type = SPEED_LOOP | ANGLE_LOOP,
            .motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
        },
        .output_axis = {1.0f, MOTOR_AXIS_DIRECTION_NORMAL},
        .motor_type = GM6020,
    };
    // 这里只注册电机；实际是否运行由周期命令中的模式决定。
    yaw_motor = DJIMotorInit(&yaw_config);
    pitch_motor = DJIMotorInit(&pitch_config);

    gimbal_pub = MessagePublisherRegister(ROBOT_TOPIC_GIMBAL_FEEDBACK,
                                          ROBOT_TOPIC_ENDPOINT_GIMBAL);
    gimbal_sub = MessageSubscriberRegister(ROBOT_TOPIC_GIMBAL_COMMAND,
                                           ROBOT_TOPIC_ENDPOINT_GIMBAL);
    return (uint8_t)(gimba_IMU_data != NULL && yaw_motor != NULL && pitch_motor != NULL &&
                     gimbal_pub != NULL && gimbal_sub != NULL);
}

/* 机器人云台控制核心任务,后续考虑只保留IMU控制,不再需要电机的反馈 */
void GimbalTask()
{
    // 只采用不超过 50 ms 的控制消息；失败清零后进入停机分支。
    if (MessageReadSnapshot(gimbal_sub, &gimbal_cmd_recv, sizeof(gimbal_cmd_recv),
                            HAL_GetTick(), NULL) != MESSAGE_OK)
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
        DJIMotorSetRef(yaw_motor, gimbal_cmd_recv.yaw * DEGREE_2_RAD);
        DJIMotorSetRef(pitch_motor, gimbal_cmd_recv.pitch * DEGREE_2_RAD);
        break;
    // FREE 目前与 GYRO 行为相同，尚未实现独立的底盘跟随控制。
    case GIMBAL_FREE_MODE:
        DJIMotorEnable(yaw_motor);
        DJIMotorEnable(pitch_motor);
        DJIMotorChangeFeed(yaw_motor, ANGLE_LOOP, OTHER_FEED);
        DJIMotorChangeFeed(yaw_motor, SPEED_LOOP, OTHER_FEED);
        DJIMotorChangeFeed(pitch_motor, ANGLE_LOOP, OTHER_FEED);
        DJIMotorChangeFeed(pitch_motor, SPEED_LOOP, OTHER_FEED);
        DJIMotorSetRef(yaw_motor, gimbal_cmd_recv.yaw * DEGREE_2_RAD);
        DJIMotorSetRef(pitch_motor, gimbal_cmd_recv.pitch * DEGREE_2_RAD);
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
