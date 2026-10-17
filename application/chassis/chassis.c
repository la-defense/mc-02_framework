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
#include "referee_task.h"
#include <string.h>

/* 底盘应用包含的模块和信息存储,底盘是单例模式,因此不需要为底盘建立单独的结构体 */
#ifdef CHASSIS_BOARD // 如果是底盘板,使用板载IMU获取底盘转动角速度
#include "can_comm.h"
#include "ins_task.h"
static CANCommInstance *chasiss_can_comm; // 双板通信CAN comm
attitude_t *Chassis_IMU_data;
#endif // CHASSIS_BOARD
#ifdef ONE_BOARD
static Publisher_t *chassis_pub;                    // 用于发布底盘的数据
static Subscriber_t *chassis_sub;                   // 用于订阅底盘的控制命令
#endif                                              // !ONE_BOARD
static Chassis_Ctrl_Cmd_s chassis_cmd_recv;         // 底盘接收到的控制命令
#if defined(ONE_BOARD) || defined(CHASSIS_BOARD)
static Chassis_Upload_Data_s chassis_feedback_data; // 底盘回传的反馈数据
#endif

static referee_info_t* referee_data; // 用于获取裁判系统的数据
static Referee_Interactive_info_t ui_data; // UI数据，将底盘中的数据传入此结构体的对应变量中，UI会自动检测是否变化，对应显示UI

static SuperCapInstance *cap;                                       // 超级电容
static DJIMotorInstance *motor_lf, *motor_rf, *motor_lb, *motor_rb; // left right forward back

uint8_t ChassisInit(void)
{
    // 四个轮子的参数一样,改tx_id和反转标志位即可
    Motor_Init_Config_s chassis_motor_config = {
        .can_init_config.can_handle = &hcan1,
        .controller_param_init_config = {
            .speed_PID = {
                .DtMin_s = MOTOR_PID_DT_MIN_S,
                .DtMax_s = MOTOR_PID_DT_MAX_S,
                .Kp = 10, // 4.5
                .Ki = 0,  // 0
                .Kd = 0,  // 0
                .IntegralLimit = 3000,
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
        },
        .motor_type = M3508,
    };
    // 每个电机使用相同配置，注册前分别填写编号和转向标记。
    chassis_motor_config.can_init_config.tx_id = 1;
    chassis_motor_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_REVERSE;
    motor_lf = DJIMotorInit(&chassis_motor_config);

    chassis_motor_config.can_init_config.tx_id = 2;
    chassis_motor_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_REVERSE;
    motor_rf = DJIMotorInit(&chassis_motor_config);

    chassis_motor_config.can_init_config.tx_id = 4;
    chassis_motor_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_REVERSE;
    motor_lb = DJIMotorInit(&chassis_motor_config);

    chassis_motor_config.can_init_config.tx_id = 3;
    chassis_motor_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_REVERSE;
    motor_rb = DJIMotorInit(&chassis_motor_config);

    referee_data = UITaskInit(&huart1,&ui_data); // 裁判系统初始化,会同时初始化UI

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

    CANComm_Init_Config_s comm_conf = {
        .can_config = {
            .can_handle = &hcan2,
            .tx_id = 0x311,
            .rx_id = 0x312,
        },
        .recv_data_len = sizeof(Chassis_Ctrl_Cmd_s),
        .send_data_len = sizeof(Chassis_Upload_Data_s),
    };
    chasiss_can_comm = CANCommInit(&comm_conf); // can comm初始化
#endif                                          // CHASSIS_BOARD

#ifdef ONE_BOARD // 单板控制整车,则通过pubsub来传递消息
    chassis_sub = SubRegister("chassis_cmd", sizeof(Chassis_Ctrl_Cmd_s));
    chassis_pub = PubRegister("chassis_feed", sizeof(Chassis_Upload_Data_s));
#endif // ONE_BOARD
    if (motor_lf == NULL || motor_rf == NULL || motor_lb == NULL || motor_rb == NULL ||
        referee_data == NULL || cap == NULL)
        return 0u;
#ifdef ONE_BOARD
    if (chassis_sub == NULL || chassis_pub == NULL)
        return 0u;
#endif
#ifdef CHASSIS_BOARD
    if (Chassis_IMU_data == NULL || chasiss_can_comm == NULL)
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
#ifdef ONE_BOARD
    command_fresh = SubGetSnapshot(chassis_sub, &chassis_cmd_recv, sizeof(chassis_cmd_recv),
                                   HAL_GetTick(), MESSAGE_DEFAULT_MAX_AGE_MS, NULL);
#endif
#ifdef CHASSIS_BOARD
    command_fresh = CANCommGetSnapshot(chasiss_can_comm, &chassis_cmd_recv,
                                       sizeof(chassis_cmd_recv), HAL_GetTick(),
                                       MESSAGE_DEFAULT_MAX_AGE_MS, NULL);
#endif
    if (!command_fresh)
        memset(&chassis_cmd_recv, 0, sizeof(chassis_cmd_recv));

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
    const uint8_t command_allowed = chassis_cmd_recv.chassis_mode != CHASSIS_ZERO_FORCE &&
                                    ChassisMotionCalculate(&motion_input, &geometry, &limits,
                                                           &motion_output);
    if (!command_allowed)
    {
        // 配置、模式或数值不允许计算时，同时停机和清掉参考值，避免重新使能后使用旧速度。
        ChassisStopAndClear();
    }
    else
    {
        DJIMotorSetRef(motor_lf, motion_output.motor_speed_deg_s[0]);
        DJIMotorSetRef(motor_rf, motion_output.motor_speed_deg_s[1]);
        DJIMotorSetRef(motor_lb, motion_output.motor_speed_deg_s[2]);
        DJIMotorSetRef(motor_rb, motion_output.motor_speed_deg_s[3]);
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
    // chassis_feedback_data.enemy_color = referee_data->GameRobotState.robot_id > 7 ? 1 : 0;
    // // 当前只做了17mm热量的数据获取,后续根据robot_def中的宏切换双枪管和英雄42mm的情况
    // chassis_feedback_data.bullet_speed = referee_data->GameRobotState.shooter_id1_17mm_speed_limit;
    // chassis_feedback_data.rest_heat = referee_data->PowerHeatData.shooter_heat0;

    // 推送反馈消息
#ifdef ONE_BOARD
    PubPushMessage(chassis_pub, (void *)&chassis_feedback_data);
#endif
#ifdef CHASSIS_BOARD
    CANCommSend(chasiss_can_comm, (void *)&chassis_feedback_data);
#endif // CHASSIS_BOARD
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
