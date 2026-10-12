#include <math.h>
#include <stdio.h>
#include <string.h>
#include "shoot.h"
#include "robot_def.h"
#include "config.h"
#include "message_center.h"

#define CHECK(condition) do { if (!(condition)) { \
    fprintf(stderr, "FAIL:%d %s\n", __LINE__, #condition); return 1; \
} } while (0)

CAN_HandleTypeDef hcan2;
DJIMotorInstance shoot_fake_motors[3];
uint32_t shoot_fake_motor_init_count;
static uint32_t now_ms;
static float timeline_ms;
static Shoot_Config_s shoot_config = { .loader_slot_num = 10u, .loader_gearbox_ratio = 1.0f, .loader_disk_ratio = 1.0f };

uint32_t HAL_GetTick(void) { return now_ms; }
float DWT_GetTimeline_ms(void) { return timeline_ms; }
Shoot_Config_s *ShootConfigFeed(void) { return &shoot_config; }
DJIMotorInstance *DJIMotorInit(Motor_Init_Config_s *config)
{
    if (config == NULL || shoot_fake_motor_init_count >= 3u)
        return NULL;
    DJIMotorInstance *motor = &shoot_fake_motors[shoot_fake_motor_init_count];
    memset(motor, 0, sizeof(*motor));
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

static void publish(Publisher_t *publisher, Shoot_Ctrl_Cmd_s *command, uint32_t timestamp)
{
    (void)PubPushMessageAt(publisher, command, timestamp);
}

int main(void)
{
    timeline_ms = 1000.0f;
    CHECK(ShootInit() == 1u);
    CHECK(shoot_fake_motor_init_count == 3u);
    Subscriber_t *observer = SubRegister("shoot_cmd", sizeof(Shoot_Ctrl_Cmd_s));
    Publisher_t *publisher = PubRegister("shoot_cmd", sizeof(Shoot_Ctrl_Cmd_s));
    CHECK(observer != NULL && publisher != NULL);

    Shoot_Ctrl_Cmd_s command = {
        .shoot_mode = SHOOT_ON,
        .load_mode = LOAD_STOP,
        .lid_mode = LID_CLOSE,
        .friction_mode = FRICTION_ON,
        .bullet_speed = (Bullet_Speed_e)99,
    };
    now_ms = 0u;
    publish(publisher, &command, now_ms);
    ShootTask();
    CHECK(shoot_fake_motors[0].enabled && shoot_fake_motors[1].enabled && shoot_fake_motors[2].enabled);
    CHECK(shoot_fake_motors[0].ref == 30000.0f && shoot_fake_motors[1].ref == 30000.0f);

    command.load_mode = LOAD_1_BULLET;
    now_ms = 1u;
    timeline_ms = 1001.0f;
    publish(publisher, &command, now_ms);
    ShootTask();
    CHECK(fabsf(shoot_fake_motors[2].ref - 138.0f) < 0.001f);

    /* No later publication: age 51ms must disable every motor and clear references. */
    now_ms = 52u;
    ShootTask();
    CHECK(!shoot_fake_motors[0].enabled && !shoot_fake_motors[1].enabled && !shoot_fake_motors[2].enabled);
    CHECK(shoot_fake_motors[0].ref == 0.0f && shoot_fake_motors[1].ref == 0.0f && shoot_fake_motors[2].ref == 0.0f);

    /* A shot cannot re-arm from an old trigger; a fresh LOAD_STOP is required first. */
    command.load_mode = LOAD_1_BULLET;
    now_ms = 53u;
    publish(publisher, &command, now_ms);
    ShootTask();
    CHECK(shoot_fake_motors[2].ref == 0.0f);

    command.load_mode = LOAD_STOP;
    now_ms = 54u;
    publish(publisher, &command, now_ms);
    ShootTask();
    CHECK(shoot_fake_motors[2].ref == 0.0f);

    command.load_mode = LOAD_3_BULLET;
    now_ms = 55u;
    timeline_ms = 1300.0f;
    publish(publisher, &command, now_ms);
    ShootTask();
    CHECK(fabsf(shoot_fake_motors[2].ref - 210.0f) < 0.001f);

    puts("PASS: production shooter initialization, expiry shutdown, and fresh LOAD_STOP re-arm");
    return 0;
}
