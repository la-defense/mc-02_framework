/**
 * @file chassis.c
 * @author NeoZeng neozng1@hnu.edu.cn
 * @brief 底盘应用,负责接收robot_cmd的控制命令并根据命令进行运动学解算,得到输出
 *        注意底盘采取右手系,对于平面视图,底盘纵向运动的正前方为x正方向;横向运动的右侧为y正方向
 *
 * @version 0.1
 * @date 2022-12-04
 *
 * @copyright Copyright (c) 2022
 *
 */

#include "chassis.h"
#include "chassis_motion.h"
#include "robot_def.h"
#include "dji_motor.h"
#include "super_cap.h"
#include "message_center.h"
#include "topics/robot_topics.h"
#include "referee_task.h"
#include "bsp_log.h"
#include "general_def.h"
#ifdef CHASSIS_BOARD
#include "robot_safety.h"
#endif
#include <string.h>

/* 底盘应用包含的模块和信息存储,底盘是单例模式,因此不需要为底盘建立单独的结构体 */
#ifdef CHASSIS_BOARD // 如果是底盘板,使用板载IMU获取底盘转动角速度
#include "ins_task.h"
attitude_t *Chassis_IMU_data;
#endif // CHASSIS_BOARD
#if defined(ONE_BOARD) || defined(CHASSIS_BOARD)
static MessagePublisher_t *chassis_pub;              // 用于发布底盘的数据
static MessageSubscriber_t *chassis_sub;             // 用于订阅底盘的控制命令
#endif
static Chassis_Ctrl_Cmd_s chassis_cmd_recv;         // 底盘接收到的控制命令
#if defined(ONE_BOARD) || defined(CHASSIS_BOARD)
static Chassis_Upload_Data_s chassis_feedback_data; // 底盘回传的反馈数据
#endif

static Referee_Interactive_info_t ui_data; // UI数据，将底盘中的数据传入此结构体的对应变量中，UI会自动检测是否变化，对应显示UI

static SuperCapInstance *cap;                                       // 超级电容
static DJIMotorInstance *motor_lf, *motor_rf, *motor_lb, *motor_rb; // left right forward back
#ifdef CHASSIS_BOARD
static uint8_t chassis_command_rearm_required = 1u;
static uint8_t chassis_rearm_sequence_valid = 0u;
static uint32_t chassis_rearm_sequence = 0u;
#endif

uint8_t ChassisInit(void)
{
    if (!RobotTopicsInit())
        return 0u;
    // 四个轮子的参数一样,改tx_id和反转标志位即可
    Motor_Init_Config_s chassis_motor_config = {
        .can_init_config.can_handle = &hcan1,
        .controller_param_init_config = {
            .speed_PID = {
                .DtMin_s = MOTOR_PID_DT_MIN_S,
                .DtMax_s = MOTOR_PID_DT_MAX_S,
                /* 转子 deg/s 改为输出轴 rad/s，按同一物理速度缩放 PID 输入单位。 */
                .Kp = 10.0f * RAD_2_DEGREE * REDUCTION_RATIO_WHEEL,
                .Ki = 0,  // 0
                .Kd = 0,  // 0
                .IntegralLimit = 3000.0f * DEGREE_2_RAD / REDUCTION_RATIO_WHEEL,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .MaxOut = 12000,
            },
            .current_PID = {
                .DtMin_s = MOTOR_PID_DT_MIN_S,
                .DtMax_s = MOTOR_PID_DT_MAX_S,
                .Kp = 0.5, // 0.4
                .Ki = 0,   // 0
                .Kd = 0,
                .IntegralLimit = 3000,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
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
        .output_axis = {REDUCTION_RATIO_WHEEL, MOTOR_AXIS_DIRECTION_REVERSE},
        .motor_type = M3508,
    };
    // 每个电机使用相同的输出轴单位和方向映射，编号仍按CAN反馈ID登记。
    chassis_motor_config.can_init_config.tx_id = 1;
    motor_lf = DJIMotorInit(&chassis_motor_config);

    chassis_motor_config.can_init_config.tx_id = 2;
    motor_rf = DJIMotorInit(&chassis_motor_config);

    chassis_motor_config.can_init_config.tx_id = 4;
    motor_lb = DJIMotorInit(&chassis_motor_config);

    chassis_motor_config.can_init_config.tx_id = 3;
    motor_rb = DJIMotorInit(&chassis_motor_config);

    RefereeStatus_t referee_status = RefereeInit(&huart1, NULL);
#if MC02_ENABLE_REFEREE_UI
    // UI绑定只服务显示，失败时不撤销已经独立注册的裁判接收。
    UIStatus_t ui_status = UITaskInit(&ui_data);
    if (ui_status != UI_OK) BSP_LOG_EVENT_1(EV_APPLICATION_CHASSIS_CHASSIS_C_01, BSP_LOG_FIELD_I32((int)ui_status));
#endif

    SuperCap_Init_Config_s cap_conf = {
        .can_config = {
            .can_handle = &hcan2,
            .tx_id = 0x302, // 超级电容默认接收id
            .rx_id = 0x301, // 超级电容默认发送id,注意tx和rx在其他人看来是反的
        }};
    cap = SuperCapInit(&cap_conf); // 超级电容初始化

    // 发布订阅初始化,如果为双板,则需要can comm来传递消息
#ifdef CHASSIS_BOARD
    Chassis_IMU_data = INS_GetStorage(); // INS 由调度器启动后的专属任务初始化

#endif                                          // CHASSIS_BOARD

#if defined(ONE_BOARD) || defined(CHASSIS_BOARD)
    chassis_sub = MessageSubscriberRegister(ROBOT_TOPIC_CHASSIS_COMMAND,
                                            ROBOT_TOPIC_ENDPOINT_CHASSIS);
    chassis_pub = MessagePublisherRegister(ROBOT_TOPIC_CHASSIS_FEEDBACK,
                                           ROBOT_TOPIC_ENDPOINT_CHASSIS);
#endif
    if (motor_lf == NULL || motor_rf == NULL || motor_lb == NULL || motor_rb == NULL ||
        referee_status != REFEREE_OK || cap == NULL)
        return 0u;
#if defined(ONE_BOARD) || defined(CHASSIS_BOARD)
    if (chassis_sub == NULL || chassis_pub == NULL)
        return 0u;
#endif
#ifdef CHASSIS_BOARD
    if (Chassis_IMU_data == NULL)
        return 0u;
#endif
    return 1u;
}

/**
 * @brief 停止四个底盘电机并清除可能遗留的速度参考值
 */
static void ChassisStopAndClear(void)
{
    DJIMotorStop(motor_lf);
    DJIMotorStop(motor_rf);
    DJIMotorStop(motor_lb);
    DJIMotorStop(motor_rb);
    DJIMotorSetRef(motor_lf, 0.0f);
    DJIMotorSetRef(motor_rf, 0.0f);
    DJIMotorSetRef(motor_lb, 0.0f);
    DJIMotorSetRef(motor_rb, 0.0f);
}

#ifdef CHASSIS_BOARD
static uint8_t ChassisMotorFeedbackIsHealthy(void)
{
    DJIMotorSummary_t summaries[4] = {{0}};
    uint8_t healthy = 1u;
    DJIMotorGetSummary(motor_lf, &summaries[0]);
    DJIMotorGetSummary(motor_rf, &summaries[1]);
    DJIMotorGetSummary(motor_lb, &summaries[2]);
    DJIMotorGetSummary(motor_rb, &summaries[3]);

    for (size_t i = 0u; i < sizeof(summaries) / sizeof(summaries[0]); ++i)
    {
        if (summaries[i].online == 0u)
            healthy = 0u;
    }
    if (healthy != 0u)
        RobotFaultClear(ROBOT_FAULT_MOTOR_OFFLINE);
    else
        RobotFaultReport(ROBOT_FAULT_MOTOR_OFFLINE);
    return healthy;
}
#endif

// 实际速度估计尚未实现，保留入口，不把空反馈当作已经测得的车速。
static void EstimateSpeed()
{
    // 根据电机速度和陀螺仪的角速度进行解算,还可以利用加速度计判断是否打滑(如果有)
    // chassis_feedback_data.vx vy wz =
    //  ...
}

/* 机器人底盘控制核心任务 */
void ChassisTask()
{
    // 单板消息和双板通信都按 50 ms 检查命令，失败后清零，不沿用旧速度。
    uint8_t command_fresh = 0u;
    MessageFreshness_t command_freshness = {0};
#if defined(ONE_BOARD) || defined(CHASSIS_BOARD)
    command_fresh = (uint8_t)(MessageReadSnapshot(chassis_sub, &chassis_cmd_recv,
        sizeof(chassis_cmd_recv), HAL_GetTick(), &command_freshness) == MESSAGE_OK);
#endif
    if (!command_fresh)
        memset(&chassis_cmd_recv, 0, sizeof(chassis_cmd_recv));

#ifdef CHASSIS_BOARD
    const uint8_t command_requests_motion = (uint8_t)(command_fresh != 0u &&
        chassis_cmd_recv.chassis_mode != CHASSIS_ZERO_FORCE);
    const uint8_t motor_feedback_healthy = ChassisMotorFeedbackIsHealthy();
    RobotSafetySetRemoteChassisPermit((uint8_t)(command_requests_motion && motor_feedback_healthy));
    RobotSafetyUpdate();
    uint8_t local_permit = (uint8_t)(RobotSafetyIsReady() && motor_feedback_healthy);
    if (local_permit == 0u)
    {
        chassis_command_rearm_required = 1u;
        chassis_rearm_sequence_valid = 0u;
    }
    else if (chassis_command_rearm_required != 0u)
    {
        if (command_fresh == 0u || command_freshness.valid == 0u)
        {
            local_permit = 0u;
            chassis_rearm_sequence_valid = 0u;
        }
        else if (chassis_rearm_sequence_valid == 0u)
        {
            // 首次观察到本地条件恢复时记下当前命令；必须再等一条新命令。
            chassis_rearm_sequence = command_freshness.sequence;
            chassis_rearm_sequence_valid = 1u;
            local_permit = 0u;
        }
        else if (command_freshness.sequence == chassis_rearm_sequence)
        {
            local_permit = 0u;
        }
        else
        {
            chassis_command_rearm_required = 0u;
        }
    }
#endif

    ChassisMotionMode_e motion_mode = CHASSIS_MOTION_MODE_INVALID;
    // 只将支持的模式交给运动计算；停机或未知模式都保持无效，随后停止四轮。
    switch (chassis_cmd_recv.chassis_mode)
    {
    case CHASSIS_NO_FOLLOW:
        motion_mode = CHASSIS_MOTION_MODE_NO_FOLLOW;
        break;
    case CHASSIS_FOLLOW_GIMBAL_YAW:
        motion_mode = CHASSIS_MOTION_MODE_FOLLOW;
        break;
    case CHASSIS_ROTATE:
        motion_mode = CHASSIS_MOTION_MODE_ROTATE;
        break;
    default:
        break;
    }

    const ChassisMotionInput_s motion_input = {
        .vx_mm_s = chassis_cmd_recv.vx,
        .vy_mm_s = chassis_cmd_recv.vy,
        .requested_wz_deg_s = chassis_cmd_recv.wz,
        .offset_angle_deg = chassis_cmd_recv.offset_angle,
        .mode = motion_mode,
    };
    const ChassisMotionGeometry_s geometry = {
        .wheel_base_mm = WHEEL_BASE,
        .track_width_mm = TRACK_WIDTH,
        .center_gimbal_offset_x_mm = CENTER_GIMBAL_OFFSET_X,
        .center_gimbal_offset_y_mm = CENTER_GIMBAL_OFFSET_Y,
        .wheel_radius_mm = RADIUS_WHEEL,
        .reduction_ratio = REDUCTION_RATIO_WHEEL,
    };
    const ChassisMotionLimits_s limits = {
        .configured = CHASSIS_MOTION_LIMITS_CONFIGURED,
        .max_wz_deg_s = CHASSIS_MAX_WZ_DEG_S,
        .rotate_wz_deg_s = CHASSIS_ROTATE_WZ_DEG_S,
        .max_motor_speed_deg_s = CHASSIS_MAX_MOTOR_SPEED_DEG_S,
    };
    ChassisMotionOutput_s motion_output;
    const uint8_t command_allowed = command_fresh != 0u &&
#ifdef CHASSIS_BOARD
                                    local_permit != 0u &&
#endif
                                    chassis_cmd_recv.chassis_mode != CHASSIS_ZERO_FORCE &&
                                    ChassisMotionCalculate(&motion_input, &geometry, &limits,
                                                           &motion_output);
#ifdef CHASSIS_BOARD
    if (command_allowed == 0u && local_permit != 0u)
    {
        chassis_command_rearm_required = 1u;
        chassis_rearm_sequence_valid = 0u;
    }
#endif
    if (!command_allowed)
    {
        // 配置、模式或数值不允许计算时，同时停机和清掉参考值，避免重新使能后使用旧速度。
        ChassisStopAndClear();
    }
    else
    {
        const float rotor_to_output = -DEGREE_2_RAD / REDUCTION_RATIO_WHEEL;
        DJIMotorSetRef(motor_lf, motion_output.motor_speed_deg_s[0] * rotor_to_output);
        DJIMotorSetRef(motor_rf, motion_output.motor_speed_deg_s[1] * rotor_to_output);
        DJIMotorSetRef(motor_lb, motion_output.motor_speed_deg_s[2] * rotor_to_output);
        DJIMotorSetRef(motor_rb, motion_output.motor_speed_deg_s[3] * rotor_to_output);
        DJIMotorEnable(motor_lf);
        DJIMotorEnable(motor_rf);
        DJIMotorEnable(motor_lb);
        DJIMotorEnable(motor_rb);
        chassis_cmd_recv.wz = motion_output.wz_deg_s;
    }

    // 根据电机的反馈速度和IMU(如果有)计算真实速度
    EstimateSpeed();

    // // 获取裁判系统数据   建议将裁判系统与底盘分离，所以此处数据应使用消息中心发送
    // // 我方颜色id小于7是红色,大于7是蓝色,注意这里发送的是对方的颜色, 0:blue , 1:red
    // // 当前只做了17mm热量的数据获取,后续根据robot_def中的宏切换双枪管和英雄42mm的情况

    // 推送反馈消息
#if defined(ONE_BOARD) || defined(CHASSIS_BOARD)
    (void)MessagePublishNow(chassis_pub, &chassis_feedback_data);
#endif
}

void Chassis_GetMotorSummary(Chassis_Motor_Summary_t *summary)
{
    if (summary == NULL)
        return;

    *summary = (Chassis_Motor_Summary_t){0};

    DJIMotorGetSummary(motor_lf, &summary->lf);
    DJIMotorGetSummary(motor_rf, &summary->rf);
    DJIMotorGetSummary(motor_lb, &summary->lb);
    DJIMotorGetSummary(motor_rb, &summary->rb);
}
