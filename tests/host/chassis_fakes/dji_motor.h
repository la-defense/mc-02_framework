#ifndef MC02_HOST_CHASSIS_DJI_MOTOR_H
#define MC02_HOST_CHASSIS_DJI_MOTOR_H
#include <stddef.h>
#include <stdint.h>
#include "main.h"
typedef struct { float Kp, Ki, Kd, IntegralLimit, MaxOut; uint32_t Improve; } ChassisPidConfig_s;
typedef struct { ChassisPidConfig_s speed_PID, current_PID; } ChassisControllerParam_s;
typedef struct {
    int angle_feedback_source, speed_feedback_source, outer_loop_type, close_loop_type;
    int motor_reverse_flag;
} ChassisControllerSetting_s;
typedef struct { CAN_HandleTypeDef *can_handle; uint32_t tx_id; } ChassisCanConfig_s;
typedef struct {
    ChassisCanConfig_s can_init_config;
    ChassisControllerParam_s controller_param_init_config;
    ChassisControllerSetting_s controller_setting_init_config;
    int motor_type;
} Motor_Init_Config_s;
typedef struct {
    float ref;
    uint8_t enabled;
    unsigned stop_count, enable_count, set_ref_order, enable_order;
} DJIMotorInstance;
typedef struct { uint8_t unused; } DJIMotorSummary_t;
enum { PID_Trapezoid_Intergral = 1, PID_Integral_Limit = 2, PID_Derivative_On_Measurement = 4 };
enum { MOTOR_FEED = 0, SPEED_LOOP = 1, CURRENT_LOOP = 2, M3508 = 1, MOTOR_DIRECTION_REVERSE = 1 };
DJIMotorInstance *DJIMotorInit(Motor_Init_Config_s *config);
void DJIMotorSetRef(DJIMotorInstance *motor, float ref);
void DJIMotorStop(DJIMotorInstance *motor);
void DJIMotorEnable(DJIMotorInstance *motor);
void DJIMotorGetSummary(DJIMotorInstance *motor, DJIMotorSummary_t *summary);
#endif
