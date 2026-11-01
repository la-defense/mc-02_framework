#ifndef MC02_SHOOT_FAKE_ROBOT_DEF_H
#define MC02_SHOOT_FAKE_ROBOT_DEF_H
#include <stdint.h>
#include "main.h"
#include "../../../modules/motor/motor_feedback.h"
#include "../../../application/config/shoot_config_types.h"

// 复用生产 PID 参数，避免新增字段时替身仍接受旧接口。
#include "../../../modules/motor/motor_pid.h"

typedef enum { MOTOR_FEED = 0 } Feedback_Source_e;
typedef enum { SPEED_LOOP = 2, CURRENT_LOOP = 1, ANGLE_LOOP = 4 } Closeloop_Type_e;
typedef enum { MOTOR_DIRECTION_NORMAL = 0, MOTOR_DIRECTION_REVERSE = 1 } Motor_Reverse_Flag_e;
typedef struct {
    CAN_HandleTypeDef *can_handle;
    uint8_t tx_id;
} Shoot_CAN_Init_s;
typedef struct {
    PID_Init_Config_s speed_PID, current_PID, angle_PID;
} Shoot_Controller_Init_s;
typedef struct {
    Feedback_Source_e angle_feedback_source, speed_feedback_source;
    Closeloop_Type_e outer_loop_type, close_loop_type;
    Motor_Reverse_Flag_e motor_reverse_flag;
} Shoot_Controller_Setting_s;
typedef struct {
    Shoot_CAN_Init_s can_init_config;
    Shoot_Controller_Init_s controller_param_init_config;
    Shoot_Controller_Setting_s controller_setting_init_config;
    Motor_Output_Axis_Config_s output_axis;
    Motor_Type_e motor_type;
} Motor_Init_Config_s;
typedef struct {
    Shoot_Type_e shoot_type;
    Motor_Init_Config_s friction_motor_config, loader_motor_config;
    uint8_t friction_motor_id[4];
    Motor_Reverse_Flag_e friction_motor_reverse_flag[4];
    uint8_t loader_slot_num;
    float loader_gearbox_ratio, loader_disk_ratio;
    float friction_default_speed_rad_s, loader_reverse_speed_rad_s;
} Shoot_Config_s;

typedef enum { SHOOT_OFF = 0, SHOOT_ON } shoot_mode_e;
typedef enum { FRICTION_OFF = 0, FRICTION_ON } friction_mode_e;
typedef enum { LOAD_STOP = 0, LOAD_REVERSE, LOAD_1_BULLET, LOAD_3_BULLET, LOAD_BURSTFIRE } loader_mode_e;
typedef enum { LID_OPEN = 0, LID_CLOSE } lid_mode_e;
typedef enum { SMALL_AMU_15 = 0, SMALL_AMU_18, SMALL_AMU_30 } Bullet_Speed_e;
#pragma pack(push, 1)
typedef struct {
    shoot_mode_e shoot_mode;
    loader_mode_e load_mode;
    lid_mode_e lid_mode;
    friction_mode_e friction_mode;
    Bullet_Speed_e bullet_speed;
    uint8_t rest_heat;
    float shoot_rate;
} Shoot_Ctrl_Cmd_s;
#pragma pack(pop)
#endif
