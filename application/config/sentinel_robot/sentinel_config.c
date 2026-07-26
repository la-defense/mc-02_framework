#include "sentinel_config.h"

/*
 * 哨兵机器人配置. hcan1/hcan2 直接映射到本地 hfdcan1/hfdcan2
 * (本地 bsp_can.h 已有 #define hcan1 hfdcan1 / #define hcan2 hfdcan2).
 * 机械参数/电机ID/PID 沿用远程 v1.0.0-autoaim-stable, 后续按实车标定.
 */
Robot_Config_s *SentinelConfigInit(void)
{
    static Robot_Config_s sentinel_config = {
        .chassis_param = {
            .chassis_motor_id[MOTOR_LF] = 3,
            .chassis_motor_id[MOTOR_RF] = 2,
            .chassis_motor_id[MOTOR_RB] = 1,
            .chassis_motor_id[MOTOR_LB] = 4,

            .wheel_type = OMNI_WHEEL,
            .wheel_measure = {
                .center_gimbal_offset_x = 0.0,
                .center_gimbal_offset_y = 0.0,
                .wheel_base = 500,
                .track_width = 500,
                .radius_wheel = 150,
                .reduction_ratio_wheel = 19.0,
            },

            .chassis_motor_config = {
                .can_init_config.can_handle = &hcan1,
                .controller_param_init_config = {
                    .speed_PID = {
                        .Kp = 5.0,
                        .Ki = 1.0,
                        .Kd = 0.0,
                        .IntegralLimit = 3000,
                        .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                        .MaxOut = 12000,
                    },
                    .current_PID = {
                        .Kp = 0.5,
                        .Ki = 0.0,
                        .Kd = 0.0,
                        .IntegralLimit = 3000,
                        .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                        .MaxOut = 15000,
                    },
                },
                .controller_setting_init_config = {
                    .angle_feedback_source = MOTOR_FEED,
                    .speed_feedback_source = MOTOR_FEED,
                    .outer_loop_type = SPEED_LOOP,
                    .close_loop_type = CURRENT_LOOP | SPEED_LOOP,
                },
                .motor_type = M3508,
            }
        },

        .gimbal_param = {
            .gimbal_type = MINI_GIMBAL,
            .gimbal_offset = {
                .mini_yaw_offset = -36.0f,
                .yaw_offset = 118,
                .pitch_min_angle = -25.0f,
                .pitch_max_angle = 25.0f,
            },

            .yaw_motor_config = {
                .can_init_config = {
                    .can_handle = &hcan1,
                    .tx_id = 5,
                },
                .controller_param_init_config = {
                    .angle_PID = {
                        .Kp = 10.0,
                        .Ki = 0.0,
                        .Kd = 0.7,
                        .DeadBand = 0.1,
                        .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                        .IntegralLimit = 100,
                        .MaxOut = 500,
                    },
                    .speed_PID = {
                        .Kp = 100.0,
                        .Ki = 0.0,
                        .Kd = 0.0,
                        .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                        .IntegralLimit = 3000,
                        .MaxOut = 20000,
                    },
                },
                .controller_setting_init_config = {
                    .angle_feedback_source = OTHER_FEED,
                    .speed_feedback_source = OTHER_FEED,
                    .outer_loop_type = ANGLE_LOOP,
                    .close_loop_type = ANGLE_LOOP | SPEED_LOOP,
                    .motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
                    .feedback_reverse_flag = FEEDBACK_DIRECTION_NORMAL,
                    .feedforward_flag = SPEED_FEEDFORWARD | CURRENT_FEEDFORWARD,
                },
                .motor_type = GM6020,
            },

            .mini_yaw_motor_config = {
                .can_init_config = {
                    .can_handle = &hcan2,
                    .tx_id = 1,
                },
                .controller_param_init_config = {
                    .angle_PID = {
                        .Kp = 10.0,
                        .Ki = 0.0,
                        .Kd = 0.0,
                        .DeadBand = 0.1,
                        .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                        .IntegralLimit = 100,
                        .MaxOut = 300,
                    },
                    .speed_PID = {
                        .Kp = 40.0,
                        .Ki = 0.0,
                        .Kd = 0.0,
                        .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                        .IntegralLimit = 8000,
                        .MaxOut = 10000,
                    },
                },
                .controller_setting_init_config = {
                    .angle_feedback_source = MOTOR_FEED,
                    .speed_feedback_source = MOTOR_FEED,
                    .outer_loop_type = ANGLE_LOOP,
                    .close_loop_type = ANGLE_LOOP | SPEED_LOOP,
                    .motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
                    .feedback_reverse_flag = FEEDBACK_DIRECTION_NORMAL,
                },
                .motor_type = GM6020,
            },

            .pitch_motor_config = {
                .can_init_config = {
                    .can_handle = &hcan2,
                    .tx_id = 6,
                },
                .controller_param_init_config = {
                    .angle_PID = {
                        .Kp = 10.0,
                        .Ki = 0.0,
                        .Kd = 0.2,
                        .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                        .IntegralLimit = 100,
                        .MaxOut = 500,
                    },
                    .speed_PID = {
                        .Kp = 45.0,
                        .Ki = 0.0,
                        .Kd = 0.2,
                        .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                        .IntegralLimit = 2500,
                        .MaxOut = 20000,
                    },
                },
                .controller_setting_init_config = {
                    .angle_feedback_source = OTHER_FEED,
                    .speed_feedback_source = OTHER_FEED,
                    .outer_loop_type = ANGLE_LOOP,
                    .close_loop_type = ANGLE_LOOP | SPEED_LOOP,
                    .motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
                    .feedback_reverse_flag = FEEDBACK_DIRECTION_NORMAL,
                    .feedforward_flag = SPEED_FEEDFORWARD | CURRENT_FEEDFORWARD,
                },
                .motor_type = GM6020
            }
        },

        .shoot_param = {
            .friction_motor_id[MOTOR_LF] = 8,
            .friction_motor_id[MOTOR_RF] = 4,
            .friction_motor_reverse_flag[MOTOR_LF] = MOTOR_DIRECTION_REVERSE,
            .friction_motor_reverse_flag[MOTOR_RF] = MOTOR_DIRECTION_NORMAL,

            .shoot_type = DUAL_SHOOT,
            .friction_motor_config = {
                .can_init_config = {
                    .can_handle = &hcan2,
                },
                .controller_param_init_config = {
                    .speed_PID = {
                        .Kp = 15.0,
                        .Ki = 1.0,
                        .Kd = 0.0,
                        .Improve = PID_Integral_Limit,
                        .IntegralLimit = 10000,
                        .MaxOut = 15000,
                    },
                    .current_PID = {
                        .Kp = 0.4,
                        .Ki = 0.0,
                        .Kd = 0.0,
                        .Improve = PID_Integral_Limit,
                        .IntegralLimit = 10000,
                        .MaxOut = 15000,
                    },
                },
                .controller_setting_init_config = {
                    .angle_feedback_source = MOTOR_FEED,
                    .speed_feedback_source = MOTOR_FEED,
                    .outer_loop_type = SPEED_LOOP,
                    .close_loop_type = SPEED_LOOP | CURRENT_LOOP,
                    .motor_reverse_flag = MOTOR_DIRECTION_REVERSE,
                },
                .motor_type = M3508
            },

            .loader_motor_config = {
                .can_init_config = {
                    .can_handle = &hcan1,
                    .tx_id = 7,
                },
                .controller_param_init_config = {
                    .angle_PID = {
                        .Kp = 5.0,
                        .Ki = 1.0,
                        .Kd = 0.0,
                        .MaxOut = 200,
                    },
                    .speed_PID = {
                        .Kp = 5.0,
                        .Ki = 1.0,
                        .Kd = 0.0,
                        .Improve = PID_Integral_Limit,
                        .IntegralLimit = 8000,
                        .MaxOut = 10000,
                    },
                    .current_PID = {
                        .Kp = 0.4,
                        .Ki = 0.0,
                        .Kd = 0.0,
                        .Improve = PID_Integral_Limit,
                        .IntegralLimit = 5000,
                        .MaxOut = 5000,
                    },
                },
                .controller_setting_init_config = {
                    .angle_feedback_source = MOTOR_FEED,
                    .speed_feedback_source = MOTOR_FEED,
                    .outer_loop_type = SPEED_LOOP,
                    .close_loop_type = CURRENT_LOOP | SPEED_LOOP,
                    .motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
                },
                .motor_type = M2006
            }
        }
    };

    return &sentinel_config;
}
