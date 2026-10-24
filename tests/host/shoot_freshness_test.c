#include <math.h>
#include <stdio.h>
#include <string.h>
#include "shoot.h"
#include "robot_def.h"
#include "config.h"
#include "message_center.h"
#include "topics/robot_topics.h"

#define CHECK(condition) do { if (!(condition)) { \
    fprintf(stderr, "FAIL:%d %s\n", __LINE__, #condition); return 1; \
} } while (0)

CAN_HandleTypeDef hcan2;
DJIMotorInstance shoot_fake_motors[3];
uint32_t shoot_fake_motor_init_count;
static uint32_t now_ms;
static float timeline_ms;
static Shoot_Config_s shoot_config = { .loader_slot_num = 10u, .loader_gearbox_ratio = 36.0f, .loader_disk_ratio = 1.0f };

uint32_t HAL_GetTick(void) { return now_ms; }
float DWT_GetTimeline_ms(void) { return timeline_ms; }
Shoot_Config_s *ShootConfigFeed(void) { return &shoot_config; }
DJIMotorInstance *DJIMotorInit(const Motor_Init_Config_s *config)
{
    if (config == NULL || shoot_fake_motor_init_count >= 3u)
        return NULL;
    DJIMotorInstance *motor = &shoot_fake_motors[shoot_fake_motor_init_count];
    memset(motor, 0, sizeof(*motor));
    motor->output_axis = config->output_axis;
    motor->close_loop_type = config->controller_setting_init_config.close_loop_type;
    motor->measure.total_angle = 100.0f + (float)shoot_fake_motor_init_count;
    shoot_fake_motor_init_count++;
    return motor;
}
void DJIMotorSetRef(DJIMotorInstance *motor, float ref) { if (motor != NULL) motor->ref = ref; }
void DJIMotorStop(DJIMotorInstance *motor) { if (motor != NULL) motor->enabled = 0u; }
void DJIMotorEnable(DJIMotorInstance *motor) { if (motor != NULL) motor->enabled = 1u; }
void DJIMotorOuterLoop(DJIMotorInstance *motor, Closeloop_Type_e loop)
{ if (motor != NULL) motor->outer_loop = loop; }
void DJIMotorGetSummary(DJIMotorInstance *motor, DJIMotorSummary_t *summary)
{ if (motor != NULL && summary != NULL) summary->enabled = motor->enabled; }
Motor_Feedback_Status_e DJIMotorGetFeedbackSnapshot(const DJIMotorInstance *motor,
                                                     uint32_t timestamp,
                                                     Motor_Feedback_Snapshot_s *snapshot)
{
    (void)timestamp;
    if (motor == NULL || snapshot == NULL)
        return MOTOR_FEEDBACK_BAD_ARGUMENT;
    const float scale = 0.01745329252f * (float)motor->output_axis.direction /
                        motor->output_axis.gear_ratio;
    snapshot->position = (Motor_Signal_Snapshot_s){
        motor->measure.total_angle * scale, MOTOR_UNIT_RAD, MOTOR_REFERENCE_OUTPUT_AXIS,
        now_ms, 1u, 1u};
    return MOTOR_FEEDBACK_OK;
}

static MessageStatus_t publish(MessagePublisher_t *publisher,
                               const Shoot_Ctrl_Cmd_s *command, uint32_t timestamp)
{
    return MessagePublishAt(publisher, command, timestamp, timestamp, timestamp,
                            MESSAGE_CLOCK_LOCAL);
}

int main(void)
{
    timeline_ms = 1000.0f;
    CHECK(ShootInit() == 1u);
    CHECK(shoot_fake_motor_init_count == 3u);
    CHECK((shoot_fake_motors[2].close_loop_type & ANGLE_LOOP) != 0);
    MessageSubscriber_t *observer = MessageSubscriberRegister(ROBOT_TOPIC_SHOOT_COMMAND, ROBOT_TOPIC_ENDPOINT_SHOOT);
    MessagePublisher_t *publisher = MessagePublisherRegister(ROBOT_TOPIC_SHOOT_COMMAND, ROBOT_TOPIC_ENDPOINT_COMMAND);
    CHECK(observer != NULL && publisher != NULL);

    Shoot_Ctrl_Cmd_s command = {
        .shoot_mode = SHOOT_ON,
        .load_mode = LOAD_STOP,
        .lid_mode = LID_CLOSE,
        .friction_mode = FRICTION_ON,
        .bullet_speed = (Bullet_Speed_e)99,
    };
    now_ms = 0u;
    CHECK(publish(publisher, &command, now_ms) == MESSAGE_OK);
    Shoot_Ctrl_Cmd_s published_command = {0};
    const MessageStatus_t read_status = MessageReadSnapshot(observer, &published_command,
        sizeof(published_command), now_ms, NULL);
    CHECK(read_status == MESSAGE_OK);
    CHECK(memcmp(&published_command, &command, sizeof(command)) == 0);
    ShootTask();
    CHECK(shoot_fake_motors[0].enabled && shoot_fake_motors[1].enabled && shoot_fake_motors[2].enabled);
    const float friction_speed = 30000.0f * 0.01745329252f / 19.0f;
    CHECK(fabsf(shoot_fake_motors[0].ref - friction_speed) < 1e-4f &&
          fabsf(shoot_fake_motors[1].ref - friction_speed) < 1e-4f);
    CHECK(shoot_fake_motors[0].output_axis.direction == MOTOR_AXIS_DIRECTION_NORMAL &&
          shoot_fake_motors[1].output_axis.direction == MOTOR_AXIS_DIRECTION_REVERSE);

    command.load_mode = LOAD_1_BULLET;
    now_ms = 1u;
    timeline_ms = 1001.0f;
    CHECK(publish(publisher, &command, now_ms) == MESSAGE_OK);
    ShootTask();
    const float expected_one_bullet_target =
        102.0f * 0.01745329252f / 36.0f + 0.62831853072f;
    if (fabsf(shoot_fake_motors[2].ref - expected_one_bullet_target) >= 1e-5f)
        fprintf(stderr, "loader target got %.9f, expected %.9f\n",
                shoot_fake_motors[2].ref, expected_one_bullet_target);
    CHECK(fabsf(shoot_fake_motors[2].ref - expected_one_bullet_target) < 1e-5f);

    /* No later publication: age 51ms must disable every motor and clear references. */
    now_ms = 52u;
    ShootTask();
    CHECK(!shoot_fake_motors[0].enabled && !shoot_fake_motors[1].enabled && !shoot_fake_motors[2].enabled);
    CHECK(shoot_fake_motors[0].ref == 0.0f && shoot_fake_motors[1].ref == 0.0f && shoot_fake_motors[2].ref == 0.0f);

    /* A shot cannot re-arm from an old trigger; a fresh LOAD_STOP is required first. */
    command.load_mode = LOAD_1_BULLET;
    now_ms = 53u;
    CHECK(publish(publisher, &command, now_ms) == MESSAGE_OK);
    ShootTask();
    CHECK(shoot_fake_motors[2].ref == 0.0f);

    command.load_mode = LOAD_STOP;
    now_ms = 54u;
    CHECK(publish(publisher, &command, now_ms) == MESSAGE_OK);
    ShootTask();
    CHECK(shoot_fake_motors[2].ref == 0.0f);

    command.load_mode = LOAD_3_BULLET;
    now_ms = 55u;
    timeline_ms = 1300.0f;
    CHECK(publish(publisher, &command, now_ms) == MESSAGE_OK);
    ShootTask();
    const float expected_three_bullet_target =
        102.0f * 0.01745329252f / 36.0f + 3.0f * 0.62831853072f;
    CHECK(fabsf(shoot_fake_motors[2].ref - expected_three_bullet_target) < 1e-5f);

    puts("PASS: production shooter initialization, expiry shutdown, and fresh LOAD_STOP re-arm");
    return 0;
}
