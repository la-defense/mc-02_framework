/** @file shoot.c
* @brief 接收发射命令，切换摩擦轮和拨弹盘控制；单发、三发只在进入对应模式时增加目标角度。
*/
#include "shoot.h"
#include "robot_def.h"
#include "config.h"

#include "dji_motor.h"
#include "message_center.h"
#include "topics/robot_topics.h"
#include "bsp_dwt.h"
#include "general_def.h"
#include "main.h"
#include <stddef.h>
#include <string.h>

#define LOADER_REVERSE_SPEED_DEGS 500 // reverse unload speed (deg/s), TBD on bench

/* 对于双发射机构的机器人,将下面的数据封装成结构体即可,生成两份shoot应用实例 */
static DJIMotorInstance *friction_l, *friction_r, *loader; // 拨盘电机
// static servo_instance *lid; 需要增加弹舱盖

static Shoot_Config_s *shoot_config; // 拨弹槽数及转子到拨弹盘的传动比例。
static float loader_output_ratio = 1.0f; // 拨弹盘输出轴与电机转子的总减速比。
static loader_mode_e last_load_mode = LOAD_STOP; // 上次处理的模式，用于判断是否刚进入单发或三发。
static uint8_t require_load_stop = 1u; // 数据中断后先等一次停止供弹，再允许新的单发或三发。

static Shoot_Ctrl_Cmd_s shoot_cmd_recv; // 来自cmd的发射控制信息
static MessageSubscriber_t *shoot_sub;

// dwt定时,计算冷却用
static float hibernate_time = 0, dead_time = 0;

uint8_t ShootInit(void)
{
    if (!RobotTopicsInit())
        return 0u;
    shoot_config = ShootConfigFeed(); // 电机输出轴减速比也使用同一份已准备配置。
    if (shoot_config == NULL)
        return 0u;
    const float loader_ratio =
        (shoot_config->loader_gearbox_ratio > 0.0f ? shoot_config->loader_gearbox_ratio : 1.0f) *
        (shoot_config->loader_disk_ratio > 0.0f ? shoot_config->loader_disk_ratio : 1.0f);
    loader_output_ratio = loader_ratio;

    // 左摩擦轮
    Motor_Init_Config_s friction_config = {
        .can_init_config = {
            .can_handle = &hcan2,
        },
        .controller_param_init_config = {
            .speed_PID = {
                .DtMin_s = MOTOR_PID_DT_MIN_S,
                .DtMax_s = MOTOR_PID_DT_MAX_S,
                .Kp = 0.0f * RAD_2_DEGREE * 19.0f,
                .Ki = 0.0f * RAD_2_DEGREE * 19.0f,
                .Kd = 0,
                .Improve = PID_Integral_Limit,
                .IntegralLimit = 10000.0f * DEGREE_2_RAD / 19.0f,
                .MaxOut = 15000,
            },
            .current_PID = {
                .DtMin_s = MOTOR_PID_DT_MIN_S,
                .DtMax_s = MOTOR_PID_DT_MAX_S,
                .Kp = 0, // 0.7
                .Ki = 0, // 0.1
                .Kd = 0,
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
        .motor_type = M3508};
    friction_config.can_init_config.tx_id = 1,
    friction_l = DJIMotorInit(&friction_config);

    friction_config.can_init_config.tx_id = 2; // 右摩擦轮,改txid和方向就行
    friction_config.output_axis.direction = MOTOR_AXIS_DIRECTION_REVERSE;
    friction_r = DJIMotorInit(&friction_config);

    // 拨盘电机
    Motor_Init_Config_s loader_config = {
        .can_init_config = {
            .can_handle = &hcan2,
            .tx_id = 3,
        },
        .controller_param_init_config = {
            .angle_PID = {
                // 如果启用位置环来控制发弹,需要较大的I值保证输出力矩的线性度否则出现接近拨出的力矩大幅下降
                .DtMin_s = MOTOR_PID_DT_MIN_S,
                .DtMax_s = MOTOR_PID_DT_MAX_S,
                .Kp = 0, // 10
                .Ki = 0,
                .Kd = 0,
                .MaxOut = 200.0f * DEGREE_2_RAD / loader_ratio,
            },
            .speed_PID = {
                .DtMin_s = MOTOR_PID_DT_MIN_S,
                .DtMax_s = MOTOR_PID_DT_MAX_S,
                .Kp = 0.0f * RAD_2_DEGREE * loader_ratio,
                .Ki = 0.0f * RAD_2_DEGREE * loader_ratio,
                .Kd = 0,
                .Improve = PID_Integral_Limit,
                .IntegralLimit = 5000.0f * DEGREE_2_RAD / loader_ratio,
                .MaxOut = 5000,
            },
            .current_PID = {
                .DtMin_s = MOTOR_PID_DT_MIN_S,
                .DtMax_s = MOTOR_PID_DT_MAX_S,
                .Kp = 0, // 0.7
                .Ki = 0, // 0.1
                .Kd = 0,
                .Improve = PID_Integral_Limit,
                .IntegralLimit = 5000,
                .MaxOut = 5000,
            },
        },
        .controller_setting_init_config = {
            .angle_feedback_source = MOTOR_FEED, .speed_feedback_source = MOTOR_FEED,
            .outer_loop_type = SPEED_LOOP, // 初始化成SPEED_LOOP,让拨盘停在原地,防止拨盘上电时乱转
            .close_loop_type = CURRENT_LOOP | SPEED_LOOP | ANGLE_LOOP,
            .motor_reverse_flag = MOTOR_DIRECTION_NORMAL, // 输出轴正方向对应拨弹方向。
        },
        .output_axis = {loader_ratio, MOTOR_AXIS_DIRECTION_NORMAL},
        .motor_type = M2006 // 英雄使用m3508
    };
    loader = DJIMotorInit(&loader_config);

    shoot_sub = MessageSubscriberRegister(ROBOT_TOPIC_SHOOT_COMMAND,
                                          ROBOT_TOPIC_ENDPOINT_SHOOT);
    return (uint8_t)(friction_l != NULL && friction_r != NULL && loader != NULL &&
                     shoot_config != NULL && shoot_sub != NULL);
}

/* 机器人发射机构控制核心任务 */
void ShootTask()
{
    // 从cmd获取控制数据
    const uint8_t command_fresh = (uint8_t)(MessageReadSnapshot(shoot_sub, &shoot_cmd_recv,
        sizeof(shoot_cmd_recv), HAL_GetTick(), NULL) == MESSAGE_OK);
    if (!command_fresh)
    {
        memset(&shoot_cmd_recv, 0, sizeof(shoot_cmd_recv));
        shoot_cmd_recv.shoot_mode = SHOOT_OFF;
        shoot_cmd_recv.friction_mode = FRICTION_OFF;
        shoot_cmd_recv.load_mode = LOAD_STOP;
        // 数据中断后不立即重放单发或三发，须先收到停止供弹命令。
        require_load_stop = 1u;
    }
    // 新消息持续请求单发也先等待 LOAD_STOP；连续供弹仍按当前模式处理。
    else if (require_load_stop != 0u)
    {
        if (shoot_cmd_recv.load_mode == LOAD_STOP)
            require_load_stop = 0u;
        else if (shoot_cmd_recv.load_mode == LOAD_1_BULLET ||
                 shoot_cmd_recv.load_mode == LOAD_3_BULLET)
            shoot_cmd_recv.load_mode = LOAD_STOP;
    }

    // 发射总开关关闭时先停止全部电机，后面的模式计算仍会继续，但不解除停机标记。
    if (shoot_cmd_recv.shoot_mode == SHOOT_OFF)
    {
        DJIMotorStop(friction_l);
        DJIMotorStop(friction_r);
        DJIMotorStop(loader);
    }
    else // 恢复运行
    {
        DJIMotorEnable(friction_l);
        DJIMotorEnable(friction_r);
        DJIMotorEnable(loader);
    }

    // 如果上一次触发单发或3发指令的时间加上不应期仍然大于当前时间(尚未休眠完毕),直接返回即可
    // 单发模式主要提供给能量机关激活使用(以及英雄的射击大部分处于单发)
    // if (hibernate_time + dead_time > DWT_GetTimeline_ms())
    //     return;

    // 若不在休眠状态,根据robotCMD传来的控制模式进行拨盘电机参考值设定和模式切换
    // 发射槽数定义输出轴角度；每发转过整圈的1/slots，单位 rad。
    const uint8_t slots = shoot_config->loader_slot_num ? shoot_config->loader_slot_num : 10u;
    const float one_bullet_delta = 2.0f * PI / slots;

    switch (shoot_cmd_recv.load_mode)
    {
    // 停止供弹时改为速度环并把目标速度置 0。
    case LOAD_STOP:
        DJIMotorOuterLoop(loader, SPEED_LOOP);
        DJIMotorSetRef(loader, 0);
        break;
    // 仅从其他模式进入单发且等待时间已到时，增加一次目标角度，避免每个周期重复增加。
    case LOAD_1_BULLET:
        if (last_load_mode != LOAD_1_BULLET && hibernate_time + dead_time <= DWT_GetTimeline_ms())
        {
            Motor_Feedback_Snapshot_s feedback = {0};
            if (DJIMotorGetFeedbackSnapshot(loader, HAL_GetTick(), &feedback) == MOTOR_FEEDBACK_OK &&
                feedback.position.valid != 0u)
            {
                DJIMotorOuterLoop(loader, ANGLE_LOOP);
                DJIMotorSetRef(loader, feedback.position.value + one_bullet_delta);
                hibernate_time = DWT_GetTimeline_ms();
                dead_time = 150;
            }
            else
            {
                /* 位置基准无效时不拿旧角度发弹；下一次单发命令可重新尝试。 */
                DJIMotorStop(loader);
                DJIMotorSetRef(loader, 0.0f);
            }
        }
        break;
    // 三发同样只在新进入时触发，随后至少等 300 ms。
    case LOAD_3_BULLET:
        if (last_load_mode != LOAD_3_BULLET && hibernate_time + dead_time <= DWT_GetTimeline_ms())
        {
            Motor_Feedback_Snapshot_s feedback = {0};
            if (DJIMotorGetFeedbackSnapshot(loader, HAL_GetTick(), &feedback) == MOTOR_FEEDBACK_OK &&
                feedback.position.valid != 0u)
            {
                DJIMotorOuterLoop(loader, ANGLE_LOOP);
                DJIMotorSetRef(loader, feedback.position.value + 3.0f * one_bullet_delta);
                hibernate_time = DWT_GetTimeline_ms();
                dead_time = 300;
            }
            else
            {
                /* 三发同样要求可信位置，否则不提交新的角度目标。 */
                DJIMotorStop(loader);
                DJIMotorSetRef(loader, 0.0f);
            }
        }
        break;
    // 连续供弹用输出轴速度环：发/秒乘每发输出轴角度，得到 rad/s。
    case LOAD_BURSTFIRE:
        DJIMotorOuterLoop(loader, SPEED_LOOP);
        DJIMotorSetRef(loader, shoot_cmd_recv.shoot_rate * one_bullet_delta);
        break;
    // 反转目标使用输出轴 rad/s，转子减速比由驱动内部处理。
    case LOAD_REVERSE:
        DJIMotorOuterLoop(loader, SPEED_LOOP);
        DJIMotorSetRef(loader, -LOADER_REVERSE_SPEED_DEGS * DEGREE_2_RAD / loader_output_ratio);
        break;
    default:
        // 未知供弹模式保持等待；此路径不会返回到任务中的下一次运行记录更新。
        while (1)
            ;
    }
    last_load_mode = shoot_cmd_recv.load_mode;

    if (shoot_cmd_recv.friction_mode == FRICTION_ON)
    {
        // 根据收到的弹速设置设定摩擦轮电机参考值,需实测后填入
        switch (shoot_cmd_recv.bullet_speed)
        {
        case SMALL_AMU_15:
            DJIMotorSetRef(friction_l, 0);
            DJIMotorSetRef(friction_r, 0);
            break;
        case SMALL_AMU_18:
            DJIMotorSetRef(friction_l, 0);
            DJIMotorSetRef(friction_r, 0);
            break;
        case SMALL_AMU_30:
            DJIMotorSetRef(friction_l, 0);
            DJIMotorSetRef(friction_r, 0);
            break;
        default: // 未匹配弹速时沿用约定的转子角速度默认目标，需按实际机构确认。
        {
            const float friction_speed_rad_s = 30000.0f * DEGREE_2_RAD / 19.0f;
            DJIMotorSetRef(friction_l, friction_speed_rad_s);
            DJIMotorSetRef(friction_r, friction_speed_rad_s);
            break;
        }
        }
    }
    else // 关闭摩擦轮
    {
        DJIMotorSetRef(friction_l, 0);
        DJIMotorSetRef(friction_r, 0);
    }

    // 开关弹舱盖
    if (shoot_cmd_recv.lid_mode == LID_CLOSE)
    {
        //...
    }
    else if (shoot_cmd_recv.lid_mode == LID_OPEN)
    {
        //...
    }

    // 反馈数据,目前暂时没有要设定的反馈数据,后续可能增加应用离线监测以及卡弹反馈
}

void Shoot_GetMotorSummary(Shoot_Motor_Summary_t *summary)
{
    if (summary == NULL)
        return;

    *summary = (Shoot_Motor_Summary_t){0};

    DJIMotorGetSummary(friction_l, &summary->friction_l);
    DJIMotorGetSummary(friction_r, &summary->friction_r);
    DJIMotorGetSummary(loader, &summary->loader);
}
