/** @file sentinel_config.c
* @brief 保存移入的哨兵配置，供 RobotConfigInit() 选择；来源说明保留如下。
 */
#include "sentinel_config.h"
#include "general_def.h"
#include "robot_safety.h"
#include "topics/robot_topics.h"
#include "referee_protocol.h"

static const RefereeUIFieldConfig_t sentinel_referee_ui_fields[] = {
    {{'u', '0', '0'}, "stage:", 80u, 150u, 20u, 2u, UI_Color_Cyan,
     REFEREE_UI_SOURCE_GAME_STAGE_REMAIN},
    {{'u', '0', '1'}, "HP:", 80u, 195u, 20u, 2u, UI_Color_Green,
     REFEREE_UI_SOURCE_ROBOT_HP},
    {{'u', '0', '2'}, "limit:", 80u, 240u, 20u, 2u, UI_Color_Yellow,
     REFEREE_UI_SOURCE_CHASSIS_POWER_LIMIT},
    {{'u', '0', '3'}, "buffer:", 80u, 285u, 20u, 2u, UI_Color_Cyan,
     REFEREE_UI_SOURCE_BUFFER_ENERGY},
    {{'u', '0', '4'}, "heat:", 80u, 330u, 20u, 2u, UI_Color_Orange,
     REFEREE_UI_SOURCE_HEAT_17MM},
};

static const RefereeUIConfig_t sentinel_referee_ui_config = {
    .fields = sentinel_referee_ui_fields,
    .field_count = (uint8_t)(sizeof(sentinel_referee_ui_fields) /
                             sizeof(sentinel_referee_ui_fields[0])),
    .frame_interval_ms = 115u,
};

/* 这些是哨兵软件默认提示色和节奏，不代表灯珠实测亮度、蜂鸣器声压或故障安全等级标定。 */
static const RobotAlarmSoundSegment_t sentinel_beep_short[] = {
    {100u, OCTAVE_5, 220u}, {180u, BUZZER_SILENT, 0u}};
static const RobotAlarmSoundSegment_t sentinel_beep_double[] = {
    {90u, OCTAVE_6, 220u}, {100u, BUZZER_SILENT, 0u},
    {90u, OCTAVE_6, 220u}, {720u, BUZZER_SILENT, 0u}};
static const RobotAlarmSoundSegment_t sentinel_beep_urgent[] = {
    {90u, OCTAVE_7, 250u}, {90u, BUZZER_SILENT, 0u},
    {90u, OCTAVE_7, 250u}, {90u, BUZZER_SILENT, 0u},
    {90u, OCTAVE_7, 250u}, {550u, BUZZER_SILENT, 0u}};
static const RobotAlarmLedSegment_t sentinel_estop_led[] = {
    {160u, 255u, 0u, 0u}, {160u, 0u, 0u, 0u},
    {160u, 0u, 0u, 255u}, {520u, 0u, 0u, 0u}};

#define SENTINEL_LED(_mode, _period, _r, _g, _b) \
    {.led = {.mode = (_mode), .period_ms = (_period), .r = (_r), .g = (_g), .b = (_b)}}
#define SENTINEL_LED_SOUND(_mode, _period, _r, _g, _b, _sound) \
    {.led = {.mode = (_mode), .period_ms = (_period), .r = (_r), .g = (_g), .b = (_b)}, \
     .sound = {.segment_count = (uint8_t)(sizeof(_sound) / sizeof((_sound)[0])), \
               .segments = (_sound)}}
#define SENTINEL_SEGMENTED(_segments, _sound) \
    {.led = {.mode = ROBOT_ALARM_LED_SEGMENTED, \
             .repeat = ROBOT_ALARM_REPEAT_LOOP, \
             .segment_count = (uint8_t)(sizeof(_segments) / sizeof((_segments)[0])), \
             .segments = (_segments)}, \
     .sound = {.segment_count = (uint8_t)(sizeof(_sound) / sizeof((_sound)[0])), \
               .segments = (_sound)}}

static const RobotAlarmFaultRule_t sentinel_alarm_faults[] = {
    {.fault_bit = ROBOT_FAULT_INIT_FAILURE, .priority = 0u, .mute_level = 0u,
     .style = SENTINEL_LED_SOUND(ROBOT_ALARM_LED_BLINK, 240u, 255u, 0u, 0u,
                                 sentinel_beep_urgent)},
    {.fault_bit = ROBOT_FAULT_RESET_LOCKOUT, .priority = 1u, .mute_level = 1u,
     .style = SENTINEL_LED_SOUND(ROBOT_ALARM_LED_BLINK, 300u, 255u, 0u, 180u,
                                 sentinel_beep_urgent)},
    {.fault_bit = ROBOT_FAULT_TASK_TIMEOUT, .priority = 2u, .mute_level = 2u,
     .style = SENTINEL_LED_SOUND(ROBOT_ALARM_LED_BLINK, 360u, 255u, 0u, 0u,
                                 sentinel_beep_double)},
    {.fault_bit = ROBOT_FAULT_RC_OFFLINE, .priority = 3u, .mute_level = 3u,
     .style = SENTINEL_LED_SOUND(ROBOT_ALARM_LED_BLINK, 500u, 255u, 100u, 0u,
                                 sentinel_beep_double)},
    {.fault_bit = ROBOT_FAULT_MOTOR_OFFLINE, .priority = 4u, .mute_level = 4u,
     .style = SENTINEL_LED_SOUND(ROBOT_ALARM_LED_BREATHE, 900u, 255u, 30u, 0u,
                                 sentinel_beep_short)},
    {.fault_bit = ROBOT_FAULT_CAN_BUSOFF, .priority = 5u, .mute_level = 5u,
     .style = SENTINEL_LED_SOUND(ROBOT_ALARM_LED_BLINK, 700u, 255u, 120u, 0u,
                                 sentinel_beep_short)},
    {.fault_bit = ROBOT_FAULT_IMU_INVALID, .priority = 6u, .mute_level = 6u,
     .style = SENTINEL_LED_SOUND(ROBOT_ALARM_LED_BLINK, 900u, 120u, 0u, 255u,
                                 sentinel_beep_short)},
    {.fault_bit = ROBOT_FAULT_TILT, .priority = 7u, .mute_level = 7u,
     .style = SENTINEL_SEGMENTED(sentinel_estop_led, sentinel_beep_double)},
    {.fault_bit = ROBOT_FAULT_CALIB_INVALID, .priority = 8u, .mute_level = 7u,
     .style = SENTINEL_LED_SOUND(ROBOT_ALARM_LED_BLINK, 1200u, 0u, 180u, 255u,
                                 sentinel_beep_short)},
};

static const RobotAlarmStateRule_t sentinel_alarm_states[] = {
    {.state = ROBOT_ALARM_STATE_STOP,
     .style = SENTINEL_LED(ROBOT_ALARM_LED_OFF, 0u, 0u, 0u, 0u)},
    {.state = ROBOT_ALARM_STATE_INIT,
     .style = SENTINEL_LED(ROBOT_ALARM_LED_BLINK, 1000u, 0u, 60u, 255u)},
    {.state = ROBOT_ALARM_STATE_SAFE,
     .style = SENTINEL_LED(ROBOT_ALARM_LED_SOLID, 0u, 0u, 40u, 120u)},
    {.state = ROBOT_ALARM_STATE_CALIB,
     .style = SENTINEL_LED(ROBOT_ALARM_LED_BREATHE, 1400u, 255u, 150u, 0u)},
    {.state = ROBOT_ALARM_STATE_READY,
     .style = SENTINEL_LED(ROBOT_ALARM_LED_SOLID, 0u, 0u, 180u, 0u)},
    {.state = ROBOT_ALARM_STATE_FAULT,
     .style = SENTINEL_LED(ROBOT_ALARM_LED_BLINK, 600u, 255u, 0u, 0u)},
    {.state = ROBOT_ALARM_STATE_ESTOP,
     .style = SENTINEL_SEGMENTED(sentinel_estop_led, sentinel_beep_urgent)},
};

static const RobotAlarmConfig_t sentinel_alarm_config = {
    .fault_rule_count = (uint8_t)(sizeof(sentinel_alarm_faults) /
                                  sizeof(sentinel_alarm_faults[0])),
    .fault_rules = sentinel_alarm_faults,
    .state_rule_count = (uint8_t)(sizeof(sentinel_alarm_states) /
                                  sizeof(sentinel_alarm_states[0])),
    .state_rules = sentinel_alarm_states,
    .unknown_fault_style = SENTINEL_LED_SOUND(ROBOT_ALARM_LED_BLINK, 450u,
                                              255u, 0u, 180u, sentinel_beep_urgent),
    .unknown_state_style = SENTINEL_LED(ROBOT_ALARM_LED_BLINK, 1600u,
                                        255u, 180u, 0u),
    .mute_level_mask = 0xF0u,
    .max_mute_ms = 30000u,
    .max_pattern_ms = ROBOT_ALARM_MAX_PATTERN_MS,
};

#undef SENTINEL_LED
#undef SENTINEL_LED_SOUND
#undef SENTINEL_SEGMENTED

/*
 * 哨兵机器人配置使用板级声明的逻辑 CAN 编号；MC-02 当前将编号 1／2 绑定 FDCAN1／2。
 * 机械参数/电机ID/PID 沿用远程 v1.0.0-autoaim-stable, 后续按实车标定.
 */
const Robot_Config_s *SentinelConfigInit(void)
{
    static const Robot_Config_s sentinel_config = {
        .alarm_config = &sentinel_alarm_config,
        .referee_ui_config = &sentinel_referee_ui_config,
        .referee_uart = PLATFORM_UART_1, // 当前裁判系统接在板级声明的串口 1。
        .remote_uart = PLATFORM_UART_5, // 当前遥控器接在板级声明的串口 5。
#if defined(CHASSIS_BOARD)
        .can_bridge = {
            .can_bus = PLATFORM_CAN_BUS_2, // 底盘板使用板级声明的第二条 CAN 总线。
            .tx_id = 0x311u, // 底盘板发送反馈时使用的标准 ID。
            .rx_id = 0x312u, // 底盘板接收云台侧消息使用的标准 ID。
            .command_topic = ROBOT_TOPIC_CHASSIS_COMMAND, // 从对端接收的底盘命令话题。
            .feedback_topic = ROBOT_TOPIC_CHASSIS_FEEDBACK, // 发往对端的底盘反馈话题。
        },
#elif defined(GIMBAL_BOARD)
        .can_bridge = {
            .can_bus = PLATFORM_CAN_BUS_1, // 云台板使用板级声明的第一条 CAN 总线。
            .tx_id = 0x312u, // 云台板发送命令时使用的标准 ID。
            .rx_id = 0x311u, // 云台板接收底盘侧消息使用的标准 ID。
            .command_topic = ROBOT_TOPIC_CHASSIS_COMMAND, // 发往对端的底盘命令话题。
            .feedback_topic = ROBOT_TOPIC_CHASSIS_FEEDBACK, // 从对端接收的底盘反馈话题。
        },
#endif
        .chassis_param = {
            .chassis_motor_id[MOTOR_LF] = 3,
            .chassis_motor_id[MOTOR_RF] = 2,
            .chassis_motor_id[MOTOR_RB] = 1,
            .chassis_motor_id[MOTOR_LB] = 4,

            .wheel_type = OMNI_WHEEL,
            .motion_limits = {.configured = 0u},
            // 未测量轮位、滚动方向和限速；生产构建由车型确认状态拒绝。
            .omni_geometry = {.wheels = {{0}}},
            // 长度使用 mm，减速比无单位；参数沿用移入版本，启用时仍须按实车核对。
            .wheel_measure = {
                .center_gimbal_offset_x = 0.0,
                .center_gimbal_offset_y = 0.0,
                .wheel_base = 500,
                .track_width = 500,
                .radius_wheel = 150,
                .reduction_ratio_wheel = 19.0,
            },

            .chassis_motor_config = {
                .can_init_config.can_bus = PLATFORM_CAN_BUS_1,
                .controller_param_init_config = {
                    .speed_PID = {
                        .DtMin_s = MOTOR_PID_DT_MIN_S,
                        .DtMax_s = MOTOR_PID_DT_MAX_S,
                        .Kp = 5.0f * RAD_2_DEGREE * 19.0f,
                        .Ki = 1.0f * RAD_2_DEGREE * 19.0f,
                        .Kd = 0.0,
                        .IntegralLimit = 3000.0f * DEGREE_2_RAD / 19.0f,
                        .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                        .MaxOut = 12000,
                    },
                    .current_PID = {
                        .DtMin_s = MOTOR_PID_DT_MIN_S,
                        .DtMax_s = MOTOR_PID_DT_MAX_S,
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
                    .motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
                },
                .output_axis = {19.0f, MOTOR_AXIS_DIRECTION_REVERSE},
                .motor_type = M3508,
            }
        },

        // 角度字段用度；PID 及反馈来源必须与实际电机和安装方向配套。
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
                    .can_bus = PLATFORM_CAN_BUS_1,
                    .tx_id = 5,
                },
                .controller_param_init_config = {
                    .angle_PID = {
                        .DtMin_s = MOTOR_PID_DT_MIN_S,
                        .DtMax_s = MOTOR_PID_DT_MAX_S,
                        .Kp = 10.0,
                        .Ki = 0.0,
                        .Kd = 0.7,
                        .DeadBand = 0.1f * DEGREE_2_RAD,
                        .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                        .IntegralLimit = 100.0f * DEGREE_2_RAD,
                        .MaxOut = 500.0f * DEGREE_2_RAD,
                    },
                    .speed_PID = {
                        .DtMin_s = MOTOR_PID_DT_MIN_S,
                        .DtMax_s = MOTOR_PID_DT_MAX_S,
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
                },
                .output_axis = {1.0f, MOTOR_AXIS_DIRECTION_NORMAL},
                .motor_type = GM6020,
            },

            .mini_yaw_motor_config = {
                .can_init_config = {
                    .can_bus = PLATFORM_CAN_BUS_2,
                    .tx_id = 1,
                },
                .controller_param_init_config = {
                    .angle_PID = {
                        .DtMin_s = MOTOR_PID_DT_MIN_S,
                        .DtMax_s = MOTOR_PID_DT_MAX_S,
                        .Kp = 10.0,
                        .Ki = 0.0,
                        .Kd = 0.0,
                        .DeadBand = 0.1f * DEGREE_2_RAD,
                        .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                        .IntegralLimit = 100.0f * DEGREE_2_RAD,
                        .MaxOut = 300.0f * DEGREE_2_RAD,
                    },
                    .speed_PID = {
                        .DtMin_s = MOTOR_PID_DT_MIN_S,
                        .DtMax_s = MOTOR_PID_DT_MAX_S,
                        /* 内部转子速度由 deg/s 改为 rad/s，按输入单位缩放。 */
                        .Kp = 40.0f * RAD_2_DEGREE,
                        .Ki = 0.0,
                        .Kd = 0.0,
                        .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                        .IntegralLimit = 8000.0f * DEGREE_2_RAD,
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
                .output_axis = {1.0f, MOTOR_AXIS_DIRECTION_NORMAL},
                .motor_type = GM6020,
            },

            .pitch_motor_config = {
                .can_init_config = {
                    .can_bus = PLATFORM_CAN_BUS_2,
                    .tx_id = 6,
                },
                .controller_param_init_config = {
                    .angle_PID = {
                        .DtMin_s = MOTOR_PID_DT_MIN_S,
                        .DtMax_s = MOTOR_PID_DT_MAX_S,
                        .Kp = 10.0,
                        .Ki = 0.0,
                        .Kd = 0.2,
                        .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                        .IntegralLimit = 100.0f * DEGREE_2_RAD,
                        .MaxOut = 500.0f * DEGREE_2_RAD,
                    },
                    .speed_PID = {
                        .DtMin_s = MOTOR_PID_DT_MIN_S,
                        .DtMax_s = MOTOR_PID_DT_MAX_S,
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
                },
                .output_axis = {1.0f, MOTOR_AXIS_DIRECTION_NORMAL},
                .motor_type = GM6020
            }
        },

        // 发射参数含电机编号与传动比例；ShootTask() 按拨弹盘输出轴计算每发角度，单位为rad。
        .shoot_param = {
            .friction_motor_id[MOTOR_LF] = 8,
            .friction_motor_id[MOTOR_RF] = 4,
            .friction_motor_reverse_flag[MOTOR_LF] = MOTOR_DIRECTION_REVERSE,
            .friction_motor_reverse_flag[MOTOR_RF] = MOTOR_DIRECTION_NORMAL,

            .shoot_type = DUAL_SHOOT,
            .friction_motor_config = {
                .can_init_config = {
                    .can_bus = PLATFORM_CAN_BUS_2,
                },
                .controller_param_init_config = {
                    .speed_PID = {
                        .DtMin_s = MOTOR_PID_DT_MIN_S,
                        .DtMax_s = MOTOR_PID_DT_MAX_S,
                        .Kp = 15.0f * RAD_2_DEGREE * 19.0f,
                        .Ki = 1.0f * RAD_2_DEGREE * 19.0f,
                        .Kd = 0.0,
                        .Improve = PID_Integral_Limit,
                        .IntegralLimit = 10000.0f * DEGREE_2_RAD / 19.0f,
                        .MaxOut = 15000,
                    },
                    .current_PID = {
                        .DtMin_s = MOTOR_PID_DT_MIN_S,
                        .DtMax_s = MOTOR_PID_DT_MAX_S,
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
                    .motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
                },
                .output_axis = {19.0f, MOTOR_AXIS_DIRECTION_NORMAL},
                .motor_type = M3508
            },

            .loader_motor_config = {
                .can_init_config = {
                    .can_bus = PLATFORM_CAN_BUS_1,
                    .tx_id = 7,
                },
                .controller_param_init_config = {
                    .angle_PID = {
                        .DtMin_s = MOTOR_PID_DT_MIN_S,
                        .DtMax_s = MOTOR_PID_DT_MAX_S,
                        /* 角度输入与角速度输出同时按同一输出轴比例缩放，增益保持不变。 */
                        .Kp = 5.0f,
                        .Ki = 1.0f,
                        .Kd = 0.0,
                        .MaxOut = 200.0f * DEGREE_2_RAD / 36.0f,
                    },
                    .speed_PID = {
                        .DtMin_s = MOTOR_PID_DT_MIN_S,
                        .DtMax_s = MOTOR_PID_DT_MAX_S,
                        .Kp = 5.0f * RAD_2_DEGREE * 36.0f,
                        .Ki = 1.0f * RAD_2_DEGREE * 36.0f,
                        .Kd = 0.0,
                        .Improve = PID_Integral_Limit,
                        .IntegralLimit = 8000.0f * DEGREE_2_RAD / 36.0f,
                        .MaxOut = 10000,
                    },
                    .current_PID = {
                        .DtMin_s = MOTOR_PID_DT_MIN_S,
                        .DtMax_s = MOTOR_PID_DT_MAX_S,
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
                .output_axis = {36.0f, MOTOR_AXIS_DIRECTION_NORMAL},
                .motor_type = M2006
            },
            .loader_slot_num = 7,           // 拨弹盘槽位数(每转一圈发射弹丸数), 待实测确认
            .loader_gearbox_ratio = 36.0f,  // M2006 自带减速箱(转子->输出轴), 用户确认 36
            .loader_disk_ratio = 1.0f,      // 输出轴 -> 拨弹盘减速比(哨兵=1)
            .friction_default_speed_rad_s = 30000.0f * DEGREE_2_RAD / 19.0f,
            .loader_reverse_speed_rad_s = 500.0f * DEGREE_2_RAD / 36.0f,
        }
    };

    return &sentinel_config;
}
