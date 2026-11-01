#ifndef MC02_HOST_CHASSIS_DJI_MOTOR_H
#define MC02_HOST_CHASSIS_DJI_MOTOR_H
#include <stddef.h>
#include <stdint.h>
#include "main.h"
// PID 参数复用生产声明，替身只记录电机调用。
#include "../../../modules/motor/motor_pid.h"
#include "../../../bsp/board/platform_resources.h"
typedef PID_Init_Config_s ChassisPidConfig_s;
typedef struct { ChassisPidConfig_s speed_PID, current_PID; } ChassisControllerParam_s;
typedef struct {
    int angle_feedback_source, speed_feedback_source, outer_loop_type, close_loop_type;
    int motor_reverse_flag;
} ChassisControllerSetting_s;
typedef enum { MOTOR_AXIS_DIRECTION_REVERSE = -1, MOTOR_AXIS_DIRECTION_NORMAL = 1 } Motor_Axis_Direction_e;
typedef struct { float gear_ratio; Motor_Axis_Direction_e direction; } Motor_Output_Axis_Config_s;
typedef PlatformCanEndpoint_t ChassisCanConfig_s; // 与生产车型配置使用相同逻辑总线类型。
typedef struct {
    ChassisCanConfig_s can_init_config;
    ChassisControllerParam_s controller_param_init_config;
    ChassisControllerSetting_s controller_setting_init_config;
    Motor_Output_Axis_Config_s output_axis;
    int motor_type;
} Motor_Init_Config_s;
typedef struct {
    float ref;
    uint8_t enabled;
    uint8_t feedback_online;       // 守护器通信状态，由测试单独控制。
    uint8_t feedback_speed_valid;  // 速度字段是否有效。
    uint8_t feedback_current_valid; // 电流字段是否有效。
    uint32_t feedback_age_ms;      // 测试指定的反馈年龄，超过20ms视为过期。
    unsigned stop_count, enable_count, set_ref_order, enable_order;
} DJIMotorInstance;
typedef struct { uint8_t online; } DJIMotorSummary_t;
typedef struct {
    struct { uint8_t valid; } speed, current;
} Motor_Feedback_Snapshot_s;
typedef enum {
    MOTOR_FEEDBACK_OK = 0,
    MOTOR_FEEDBACK_BAD_ARGUMENT,
    MOTOR_FEEDBACK_NOT_READY,
    MOTOR_FEEDBACK_STALE
} Motor_Feedback_Status_e;
enum { MOTOR_FEED = 0, SPEED_LOOP = 1, CURRENT_LOOP = 2, M3508 = 1,
       MOTOR_DIRECTION_NORMAL = 0, MOTOR_DIRECTION_REVERSE = 1 };
DJIMotorInstance *DJIMotorInit(const Motor_Init_Config_s *config);
void DJIMotorSetRef(DJIMotorInstance *motor, float ref);
void DJIMotorStop(DJIMotorInstance *motor);
void DJIMotorEnable(DJIMotorInstance *motor);
void DJIMotorGetSummary(DJIMotorInstance *motor, DJIMotorSummary_t *summary);
Motor_Feedback_Status_e DJIMotorGetFeedbackSnapshot(DJIMotorInstance *motor, uint32_t now_ms,
                                                     Motor_Feedback_Snapshot_s *snapshot);
#endif
