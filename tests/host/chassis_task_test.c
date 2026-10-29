#include "chassis.h"
#include "message_center.h"
#include "topics/robot_topics.h"
#include "robot_def.h"
#include "referee_task.h"
#include "super_cap.h"
#include "general_def.h"

#include <stdio.h>
#include <string.h>

#define CHECK(condition) do { if (!(condition)) { \
    fprintf(stderr, "FAIL:%d %s\n", __LINE__, #condition); return 1; } } while (0)

static uint32_t test_tick;
uint32_t HAL_GetTick(void) { return test_tick; }
CAN_HandleTypeDef hcan1, hcan2;
UART_HandleTypeDef huart1;
static DJIMotorInstance motors[4];
static Motor_Init_Config_s motor_configs[4];
static unsigned motor_count;
static unsigned operation_order;
static SuperCapInstance cap;

DJIMotorInstance *DJIMotorInit(const Motor_Init_Config_s *config)
{
    if (motor_count >= 4)
        return NULL;
    motor_configs[motor_count] = *config;
    return &motors[motor_count++];
}
void DJIMotorSetRef(DJIMotorInstance *motor, float ref)
{ if (motor) { motor->ref = ref; motor->set_ref_order = ++operation_order; } }
void DJIMotorStop(DJIMotorInstance *motor)
{ if (motor) { motor->enabled = 0; ++motor->stop_count; } }
void DJIMotorEnable(DJIMotorInstance *motor)
{ if (motor) { motor->enabled = 1; ++motor->enable_count; motor->enable_order = ++operation_order; } }
void DJIMotorGetSummary(DJIMotorInstance *motor, DJIMotorSummary_t *summary)
{ (void)motor; memset(summary, 0, sizeof(*summary)); }
SuperCapInstance *SuperCapInit(const SuperCap_Init_Config_s *config) { (void)config; return &cap; }
RefereeStatus_t RefereeInit(UART_HandleTypeDef *handle, const RefereeConfig_t *config)
{ (void)handle; (void)config; return REFEREE_OK; }
void MC02ChassisTestLogError(void) {}

int main(void)
{
    ChassisInit();
    CHECK(motor_count == 4);
    for (unsigned i = 0; i < 4; ++i) {
        CHECK(motor_configs[i].output_axis.gear_ratio == REDUCTION_RATIO_WHEEL);
        CHECK(motor_configs[i].output_axis.direction == MOTOR_AXIS_DIRECTION_REVERSE);
        CHECK(motor_configs[i].controller_param_init_config.speed_PID.Kp > 10.0f);
    }
    MessagePublisher_t *commands = MessagePublisherRegister(ROBOT_TOPIC_CHASSIS_COMMAND, ROBOT_TOPIC_ENDPOINT_COMMAND);
    Chassis_Ctrl_Cmd_s command = {
        .vx = 100.0f, .vy = 0.0f, .wz = 0.0f, .offset_angle = 0.0f,
        .chassis_mode = CHASSIS_NO_FOLLOW,
    };
    MessagePublishNow(commands, &command);
    ChassisTask();
#if CHASSIS_TASK_EXPECT_CONFIGURED
    CHECK(motors[0].enabled && motors[1].enabled && motors[2].enabled && motors[3].enabled);
    unsigned last_reference_write = 0;
    unsigned first_enable = UINT32_MAX;
    for (unsigned i = 0; i < 4; ++i) {
        CHECK(motors[i].ref != 0.0f);
        if (motors[i].set_ref_order > last_reference_write) last_reference_write = motors[i].set_ref_order;
        if (motors[i].enable_order < first_enable) first_enable = motors[i].enable_order;
    }
    CHECK(last_reference_write < first_enable);
    CHECK(motors[0].ref > 0.0f && motors[2].ref < 0.0f);
    command = (Chassis_Ctrl_Cmd_s){.offset_angle = 0.5f,
                                   .chassis_mode = CHASSIS_FOLLOW_GIMBAL_YAW};
    MessagePublishNow(commands, &command);
    ChassisTask();
    for (unsigned i = 0; i < 4; ++i) CHECK(motors[i].ref < 0.0f);
    command.offset_angle = -0.5f;
    MessagePublishNow(commands, &command);
    ChassisTask();
    for (unsigned i = 0; i < 4; ++i) CHECK(motors[i].ref > 0.0f);
#else
    for (unsigned i = 0; i < 4; ++i) CHECK(!motors[i].enabled && motors[i].ref == 0.0f);
#endif

    /* A zero-force command must clear old references as well as stop output. */
    command.chassis_mode = CHASSIS_ZERO_FORCE;
    MessagePublishNow(commands, &command);
    ChassisTask();
    for (unsigned i = 0; i < 4; ++i)
        CHECK(!motors[i].enabled && motors[i].ref == 0.0f && motors[i].stop_count > 0);

#if CHASSIS_TASK_EXPECT_CONFIGURED
    /* Re-entry computes new bounded references before enabling the motors. */
    command = (Chassis_Ctrl_Cmd_s){.vx = 100000.0f, .chassis_mode = CHASSIS_NO_FOLLOW};
    MessagePublishNow(commands, &command);
    ChassisTask();
    for (unsigned i = 0; i < 4; ++i) {
        CHECK(motors[i].enabled);
        CHECK(motors[i].enable_count > 0);
        CHECK(motors[i].ref != 0.0f);
        const float max_output_rad_s = CHASSIS_MAX_MOTOR_SPEED_DEG_S * DEGREE_2_RAD /
                                       REDUCTION_RATIO_WHEEL;
        CHECK(motors[i].ref <= max_output_rad_s && motors[i].ref >= -max_output_rad_s);
    }
    command = (Chassis_Ctrl_Cmd_s){.vx = 20.0f, .chassis_mode = (chassis_mode_e)99};
    MessagePublishNow(commands, &command);
    ChassisTask();
    for (unsigned i = 0; i < 4; ++i)
        CHECK(!motors[i].enabled && motors[i].ref == 0.0f);
    command = (Chassis_Ctrl_Cmd_s){.vx = 20.0f, .offset_angle = 181.0f,
                                   .chassis_mode = CHASSIS_NO_FOLLOW};
    MessagePublishNow(commands, &command);
    ChassisTask();
    for (unsigned i = 0; i < 4; ++i)
        CHECK(!motors[i].enabled && motors[i].ref == 0.0f);
#endif

    /* The 50 ms edge remains valid and the following millisecond revokes output. */
    test_tick = 100u;
    command = (Chassis_Ctrl_Cmd_s){.vx = 100.0f, .chassis_mode = CHASSIS_NO_FOLLOW};
    MessagePublishNow(commands, &command);
    test_tick = 150u;
    ChassisTask();
#if CHASSIS_TASK_EXPECT_CONFIGURED
    CHECK(motors[0].enabled && motors[0].ref != 0.0f);
#endif
    test_tick = 151u;
    ChassisTask();
    for (unsigned i = 0; i < 4; ++i)
        CHECK(!motors[i].enabled && motors[i].ref == 0.0f);
    test_tick = 152u;
    MessagePublishNow(commands, &command);
    ChassisTask();
#if CHASSIS_TASK_EXPECT_CONFIGURED
    CHECK(motors[0].enabled && motors[0].ref != 0.0f);
#else
    for (unsigned i = 0; i < 4; ++i) CHECK(!motors[i].enabled && motors[i].ref == 0.0f);
#endif

    puts("chassis task checks passed");
    return 0;
}
