#ifndef MC02_SHOOT_FAKE_DJI_MOTOR_H
#define MC02_SHOOT_FAKE_DJI_MOTOR_H
#include <stdint.h>
#include "robot_def.h"
typedef struct { float total_angle; } Shoot_Motor_Measure_s;
typedef struct {
    Shoot_Motor_Measure_s measure;
    float ref;
    uint8_t enabled;
    Closeloop_Type_e outer_loop;
} DJIMotorInstance;
typedef struct {
    uint8_t online, enabled;
    float speed_aps, angle_single_round, total_angle;
    int16_t current;
    uint8_t temperature;
} DJIMotorSummary_t;
DJIMotorInstance *DJIMotorInit(Motor_Init_Config_s *config);
void DJIMotorSetRef(DJIMotorInstance *motor, float ref);
void DJIMotorStop(DJIMotorInstance *motor);
void DJIMotorEnable(DJIMotorInstance *motor);
void DJIMotorOuterLoop(DJIMotorInstance *motor, Closeloop_Type_e loop);
void DJIMotorGetSummary(DJIMotorInstance *motor, DJIMotorSummary_t *summary);
extern DJIMotorInstance shoot_fake_motors[3];
extern uint32_t shoot_fake_motor_init_count;
#endif
