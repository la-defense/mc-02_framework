/**
 * @file gimbal_mode_test.c
 * @brief 通过 main() 调用生产 GimbalTask，检查模式、参考单位和失效命令处理。
 * @note 只替换 INS、消息中心和电机接口；由独立主机测试运行，不启动 RTOS 或连接电机。
 */
#include "test_check.h"
#include "gimbal.h"
#include "config.h"
#include "main.h"
#include "message_center.h"
#include "robot_def.h"
#include "topics/robot_topics.h"
#include "general_def.h"

#include <stddef.h>
#include <stdint.h>
#include <math.h>
#include <string.h>

/* 消息中心对象是不透明类型；测试只需稳定的占位存储。 */
struct MessagePublisher
{
    uint8_t reserved;
};

struct MessageSubscriber
{
    uint8_t reserved;
};

typedef enum
{
    MOTOR_ACTION_STOP = 1,
    MOTOR_ACTION_ENABLE,
    MOTOR_ACTION_CHANGE_FEED,
    MOTOR_ACTION_SET_REF
} MotorAction_e;

typedef struct
{
    MotorAction_e action;
    uint8_t motor_index;
    Closeloop_Type_e loop;
    Feedback_Source_e source;
    float reference;
} MotorAction_s;

CAN_HandleTypeDef hcan3 = {3u};

static Gimbal_Config_s gimbal_configuration = {
    .gimbal_type = SINGLE_GIMBAL,
    .yaw_motor_config = {
        .can_init_config = {.can_handle = &hcan3, .tx_id = 1u},
        .controller_param_init_config = {
            .angle_PID = {.Kp = 3.25f}, .speed_PID = {.Kp = 4.5f},
        },
        .controller_setting_init_config = {
            .angle_feedback_source = MOTOR_FEED,
            .speed_feedback_source = MOTOR_FEED,
            .outer_loop_type = ANGLE_LOOP,
            .close_loop_type = ANGLE_LOOP | SPEED_LOOP,
            .motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
        },
        .output_axis = {1.0f, MOTOR_AXIS_DIRECTION_NORMAL},
        .motor_type = GM6020,
    },
    .pitch_motor_config = {
        .can_init_config = {.can_handle = &hcan3, .tx_id = 2u},
        .controller_param_init_config = {
            .angle_PID = {.Kp = 6.25f}, .speed_PID = {.Kp = 7.5f},
        },
        .controller_setting_init_config = {
            .angle_feedback_source = MOTOR_FEED,
            .speed_feedback_source = MOTOR_FEED,
            .outer_loop_type = ANGLE_LOOP,
            .close_loop_type = ANGLE_LOOP | SPEED_LOOP,
            .motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
        },
        .output_axis = {1.0f, MOTOR_AXIS_DIRECTION_NORMAL},
        .motor_type = GM6020,
    },
};
const Gimbal_Config_s *GimbalConfigFeed(void) { return &gimbal_configuration; }

static attitude_t imu_storage = {.Roll = 1.0f, .Pitch = 2.0f, .Yaw = 3.0f};
static INS_Control_Snapshot_s ins_snapshot = {
    .yaw_rad = 0.25f,
    .yaw_total_rad = 6.5f,
    .pitch_rad = -0.5f,
    .gyro_rad_s = {1.25f, -2.5f, 3.75f},
    .timestamp_ms = 1234u,
    .sequence = 17u,
    .valid = 1u};
static DJIMotorInstance motors[2];
static Motor_Init_Config_s motor_configs[2];
static unsigned motor_init_count;
static MotorAction_s actions[16];
static size_t action_count;
static Gimbal_Ctrl_Cmd_s incoming_command;
static MessageStatus_t incoming_status = MESSAGE_ERR_EMPTY;
static uint32_t current_time_ms;
static Gimbal_Upload_Data_s published_feedback;
static unsigned publish_count;
static unsigned topics_init_count;
static struct MessagePublisher publisher_storage;
static struct MessageSubscriber subscriber_storage;

static uint8_t MotorIndex(const DJIMotorInstance *motor)
{
    TEST_CHECK(motor == &motors[0] || motor == &motors[1]);
    return (uint8_t)(motor == &motors[1]);
}

static void RecordMotorAction(MotorAction_s action)
{
    TEST_CHECK(action_count < sizeof(actions) / sizeof(actions[0]));
    actions[action_count++] = action;
}

attitude_t *INS_GetStorage(void)
{
    return &imu_storage;
}

uint8_t INSGetControlSnapshot(INS_Control_Snapshot_s *snapshot)
{
    if (snapshot == NULL)
        return 0u;
    *snapshot = ins_snapshot;
    return 1u;
}

uint32_t HAL_GetTick(void)
{
    return current_time_ms;
}

uint8_t RobotTopicsInit(void)
{
    ++topics_init_count;
    return 1u;
}

DJIMotorInstance *DJIMotorInit(const Motor_Init_Config_s *config)
{
    TEST_CHECK(config != NULL);
    TEST_CHECK(motor_init_count < 2u);
    TEST_CHECK(config->can_init_config.can_handle == &hcan3);
    TEST_CHECK(config->can_init_config.tx_id == motor_init_count + 1u);
    motor_configs[motor_init_count] = *config;
    motors[motor_init_count].measure.angle_single_round =
        motor_init_count == 0u ? 42.0f : 17.0f;
    return &motors[motor_init_count++];
}

void DJIMotorStop(DJIMotorInstance *motor)
{
    RecordMotorAction((MotorAction_s){.action = MOTOR_ACTION_STOP,
                                      .motor_index = MotorIndex(motor)});
}

void DJIMotorEnable(DJIMotorInstance *motor)
{
    RecordMotorAction((MotorAction_s){.action = MOTOR_ACTION_ENABLE,
                                      .motor_index = MotorIndex(motor)});
}

void DJIMotorChangeFeed(DJIMotorInstance *motor, Closeloop_Type_e loop,
                        Feedback_Source_e source)
{
    RecordMotorAction((MotorAction_s){.action = MOTOR_ACTION_CHANGE_FEED,
                                      .motor_index = MotorIndex(motor),
                                      .loop = loop,
                                      .source = source});
}

void DJIMotorSetRef(DJIMotorInstance *motor, float reference)
{
    RecordMotorAction((MotorAction_s){.action = MOTOR_ACTION_SET_REF,
                                      .motor_index = MotorIndex(motor),
                                      .reference = reference});
}

void DJIMotorGetSummary(DJIMotorInstance *motor, DJIMotorSummary_t *summary)
{
    (void)motor;
    (void)summary;
}

MessagePublisher_t *MessagePublisherRegister(MessageTopicId_t topic_id,
                                             MessageEndpointId_t endpoint_id)
{
    TEST_CHECK(topic_id == ROBOT_TOPIC_GIMBAL_FEEDBACK);
    TEST_CHECK(endpoint_id == ROBOT_TOPIC_ENDPOINT_GIMBAL);
    return &publisher_storage;
}

MessageSubscriber_t *MessageSubscriberRegister(MessageTopicId_t topic_id,
                                               MessageEndpointId_t endpoint_id)
{
    TEST_CHECK(topic_id == ROBOT_TOPIC_GIMBAL_COMMAND);
    TEST_CHECK(endpoint_id == ROBOT_TOPIC_ENDPOINT_GIMBAL);
    return &subscriber_storage;
}

MessageStatus_t MessageReadSnapshot(MessageSubscriber_t *subscriber, void *output,
                                    size_t output_size, uint32_t now_ms,
                                    MessageFreshness_t *freshness)
{
    TEST_CHECK(subscriber == &subscriber_storage);
    TEST_CHECK(output != NULL && output_size == sizeof(incoming_command));
    TEST_CHECK(now_ms == current_time_ms);
    TEST_CHECK(freshness == NULL);
    if (incoming_status == MESSAGE_OK)
        memcpy(output, &incoming_command, sizeof(incoming_command));
    return incoming_status;
}

MessageStatus_t MessagePublishNow(MessagePublisher_t *publisher, const void *data)
{
    TEST_CHECK(publisher == &publisher_storage);
    TEST_CHECK(data != NULL);
    memcpy(&published_feedback, data, sizeof(published_feedback));
    ++publish_count;
    return MESSAGE_OK;
}

static void ResetActions(void)
{
    memset(actions, 0, sizeof(actions));
    action_count = 0u;
}

static const MotorAction_s *FindAction(MotorAction_e action, uint8_t motor_index,
                                       Closeloop_Type_e loop)
{
    for (size_t index = 0u; index < action_count; ++index)
    {
        if (actions[index].action == action &&
            actions[index].motor_index == motor_index &&
            (action != MOTOR_ACTION_CHANGE_FEED || actions[index].loop == loop))
            return &actions[index];
    }
    return NULL;
}

static void CheckActiveMode(gimbal_mode_e mode, float yaw_deg, float pitch_deg)
{
    incoming_command = (Gimbal_Ctrl_Cmd_s){
        .yaw = yaw_deg,
        .pitch = pitch_deg,
        .gimbal_mode = mode};
    incoming_status = MESSAGE_OK;
    ResetActions();

    GimbalTask();

    TEST_CHECK(action_count == 8u);
    for (uint8_t motor_index = 0u; motor_index < 2u; ++motor_index)
    {
        TEST_CHECK(FindAction(MOTOR_ACTION_ENABLE, motor_index, OPEN_LOOP) != NULL);
        TEST_CHECK(FindAction(MOTOR_ACTION_STOP, motor_index, OPEN_LOOP) == NULL);

        const MotorAction_s *angle_feed =
            FindAction(MOTOR_ACTION_CHANGE_FEED, motor_index, ANGLE_LOOP);
        const MotorAction_s *speed_feed =
            FindAction(MOTOR_ACTION_CHANGE_FEED, motor_index, SPEED_LOOP);
        TEST_CHECK(angle_feed != NULL && angle_feed->source == OTHER_FEED);
        TEST_CHECK(speed_feed != NULL && speed_feed->source == OTHER_FEED);
    }

    const MotorAction_s *yaw_ref = FindAction(MOTOR_ACTION_SET_REF, 0u, OPEN_LOOP);
    const MotorAction_s *pitch_ref = FindAction(MOTOR_ACTION_SET_REF, 1u, OPEN_LOOP);
    TEST_CHECK(yaw_ref != NULL && fabsf(yaw_ref->reference - yaw_deg * DEGREE_2_RAD) < 1e-6f);
    TEST_CHECK(pitch_ref != NULL && fabsf(pitch_ref->reference - pitch_deg * DEGREE_2_RAD) < 1e-6f);
}

static void CheckStoppedMode(void)
{
    ResetActions();
    GimbalTask();

    TEST_CHECK(action_count == 4u);
    for (uint8_t motor_index = 0u; motor_index < 2u; ++motor_index)
    {
        TEST_CHECK(FindAction(MOTOR_ACTION_STOP, motor_index, OPEN_LOOP) != NULL);
        const MotorAction_s *reference =
            FindAction(MOTOR_ACTION_SET_REF, motor_index, OPEN_LOOP);
        TEST_CHECK(reference != NULL && reference->reference == 0.0f);
        TEST_CHECK(FindAction(MOTOR_ACTION_ENABLE, motor_index, OPEN_LOOP) == NULL);
    }
}

static void CheckFeedbackSource(const Motor_Signal_Source_s *source,
                                float expected_value, Motor_Unit_e expected_unit)
{
    TEST_CHECK(source->read != NULL);
    TEST_CHECK(source->context != NULL);
    TEST_CHECK(source->unit == expected_unit);
    TEST_CHECK(source->reference == MOTOR_REFERENCE_EXTERNAL);
    TEST_CHECK(source->max_age_ms == 5u);

    Motor_Signal_Snapshot_s snapshot = {0};
    TEST_CHECK(source->read(source->context, &snapshot) == MOTOR_FEEDBACK_OK);
    TEST_CHECK(snapshot.valid == 1u && snapshot.sequence == ins_snapshot.sequence);
    TEST_CHECK(snapshot.source_time_ms == ins_snapshot.timestamp_ms);
    TEST_CHECK(snapshot.unit == expected_unit);
    TEST_CHECK(snapshot.reference == MOTOR_REFERENCE_EXTERNAL);
    TEST_CHECK(fabsf(snapshot.value - expected_value) < 1e-6f);
}

static void TestFeedbackConfiguration(void)
{
    for (uint8_t index = 0u; index < 2u; ++index)
    {
        TEST_CHECK(motor_configs[index].controller_setting_init_config.angle_feedback_source == OTHER_FEED);
        TEST_CHECK(motor_configs[index].controller_setting_init_config.speed_feedback_source == OTHER_FEED);
        CheckFeedbackSource(&motor_configs[index].controller_param_init_config.other_angle_feedback,
                            index == 0u ? 6.5f : -0.5f, MOTOR_UNIT_RAD);
        CheckFeedbackSource(&motor_configs[index].controller_param_init_config.other_speed_feedback,
                            index == 0u ? 3.75f : 1.25f, MOTOR_UNIT_RAD_S);
    }
}

int main(void)
{
    TEST_CHECK(GimbalInit() == 1u);
    TEST_CHECK(topics_init_count == 1u && motor_init_count == 2u);
    TEST_CHECK(motor_configs[0].controller_param_init_config.angle_PID.Kp == 3.25f &&
               motor_configs[0].controller_param_init_config.speed_PID.Kp == 4.5f);
    TEST_CHECK(motor_configs[1].controller_param_init_config.angle_PID.Kp == 6.25f &&
               motor_configs[1].controller_param_init_config.speed_PID.Kp == 7.5f);
    TestFeedbackConfiguration();

    current_time_ms = 100u;
    CheckActiveMode(GIMBAL_GYRO_MODE, 90.0f, -30.0f);
    CheckActiveMode(GIMBAL_FREE_MODE, -45.0f, 12.0f);

    incoming_command = (Gimbal_Ctrl_Cmd_s){
        .yaw = 120.0f,
        .pitch = -50.0f,
        .gimbal_mode = GIMBAL_ZERO_FORCE};
    incoming_status = MESSAGE_OK;
    CheckStoppedMode();

    incoming_command = (Gimbal_Ctrl_Cmd_s){
        .yaw = 120.0f,
        .pitch = -50.0f,
        .gimbal_mode = (gimbal_mode_e)99};
    incoming_status = MESSAGE_OK;
    CheckStoppedMode();

    /* 读取失败时不得沿用上次有效的 GYRO 目标。 */
    incoming_command.gimbal_mode = GIMBAL_GYRO_MODE;
    incoming_status = MESSAGE_ERR_EXPIRED;
    CheckStoppedMode();

    TEST_CHECK(publish_count == 5u);
    TEST_CHECK(published_feedback.gimbal_imu_data.Roll == imu_storage.Roll);
    TEST_CHECK(published_feedback.gimbal_imu_data.Pitch == imu_storage.Pitch);
    TEST_CHECK(published_feedback.yaw_motor_single_round_angle == 42u);

    ins_snapshot.sequence = 0u;
    Motor_Signal_Snapshot_s unchanged = {
        .value = 77.0f,
        .unit = MOTOR_UNIT_DEG,
        .reference = MOTOR_REFERENCE_ROTOR,
        .sequence = 88u};
    TEST_CHECK(motor_configs[0].controller_param_init_config.other_angle_feedback.read(
                   motor_configs[0].controller_param_init_config.other_angle_feedback.context,
                   &unchanged) == MOTOR_FEEDBACK_NOT_READY);
    TEST_CHECK(unchanged.value == 77.0f && unchanged.sequence == 88u);

    gimbal_configuration.gimbal_type = MINI_GIMBAL;
    TEST_CHECK(GimbalInit() == 0u);
    TEST_CHECK(motor_init_count == 2u);

    return 0;
}
