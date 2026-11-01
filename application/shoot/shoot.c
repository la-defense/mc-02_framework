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
#include <math.h>


/* 对于双发射机构的机器人,将下面的数据封装成结构体即可,生成两份shoot应用实例 */
static DJIMotorInstance *friction_motors[4], *loader;
static uint8_t friction_count;
// static servo_instance *lid; 需要增加弹舱盖

static const Shoot_Config_s *shoot_config;
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

    shoot_config = ShootConfigFeed();
    if (shoot_config == NULL)
        return 0u;

    friction_count = ShootFrictionWheelCount(shoot_config->shoot_type);
    if (friction_count == 0u) return 0u;

    const float loader_ratio = shoot_config->loader_gearbox_ratio *
                               shoot_config->loader_disk_ratio;
    if (shoot_config->loader_slot_num == 0u ||
        !isfinite(shoot_config->loader_gearbox_ratio) || shoot_config->loader_gearbox_ratio <= 0.0f ||
        !isfinite(shoot_config->loader_disk_ratio) || shoot_config->loader_disk_ratio <= 0.0f ||
        !isfinite(loader_ratio) || loader_ratio <= 0.0f ||
        !isfinite(shoot_config->friction_default_speed_rad_s) ||
        shoot_config->friction_default_speed_rad_s <= 0.0f ||
        !isfinite(shoot_config->loader_reverse_speed_rad_s) ||
        shoot_config->loader_reverse_speed_rad_s <= 0.0f ||
        shoot_config->friction_motor_config.can_init_config.can_bus < PLATFORM_CAN_BUS_1 ||
        shoot_config->friction_motor_config.can_init_config.can_bus > PLATFORM_CAN_BUS_3 ||
        shoot_config->friction_motor_config.motor_type == MOTOR_TYPE_NONE ||
        shoot_config->loader_motor_config.can_init_config.can_bus < PLATFORM_CAN_BUS_1 ||
        shoot_config->loader_motor_config.can_init_config.can_bus > PLATFORM_CAN_BUS_3 ||
        shoot_config->loader_motor_config.can_init_config.tx_id == 0u ||
        shoot_config->loader_motor_config.can_init_config.tx_id > 8u ||
        shoot_config->loader_motor_config.motor_type == MOTOR_TYPE_NONE)
        return 0u;

    for (uint8_t index = 0u; index < friction_count; ++index)
    {
        if (shoot_config->friction_motor_id[index] == 0u ||
            shoot_config->friction_motor_id[index] > 8u ||
            (shoot_config->friction_motor_reverse_flag[index] != MOTOR_DIRECTION_NORMAL &&
             shoot_config->friction_motor_reverse_flag[index] != MOTOR_DIRECTION_REVERSE))
            return 0u;
        if (shoot_config->friction_motor_config.can_init_config.can_bus ==
                shoot_config->loader_motor_config.can_init_config.can_bus &&
            shoot_config->friction_motor_id[index] ==
                shoot_config->loader_motor_config.can_init_config.tx_id)
            return 0u;
        for (uint8_t previous = 0u; previous < index; ++previous)
            if (shoot_config->friction_motor_id[previous] == shoot_config->friction_motor_id[index])
                return 0u;
    }

    for (uint8_t index = 0u; index < friction_count; ++index)
    {
        Motor_Init_Config_s friction_config = shoot_config->friction_motor_config;
        friction_config.can_init_config.tx_id = shoot_config->friction_motor_id[index];
        // 每个摩擦轮的安装方向来自当前车型配置。
        friction_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_NORMAL;
        friction_config.output_axis.direction =
            shoot_config->friction_motor_reverse_flag[index] == MOTOR_DIRECTION_REVERSE ?
                MOTOR_AXIS_DIRECTION_REVERSE : MOTOR_AXIS_DIRECTION_NORMAL;
        friction_motors[index] = DJIMotorInit(&friction_config);
        if (friction_motors[index] == NULL)
            return 0u;
    }

    Motor_Init_Config_s loader_config = shoot_config->loader_motor_config;
    loader_config.output_axis.gear_ratio = loader_ratio;
    loader = DJIMotorInit(&loader_config);

    shoot_sub = MessageSubscriberRegister(ROBOT_TOPIC_SHOOT_COMMAND,
                                          ROBOT_TOPIC_ENDPOINT_SHOOT);
    return (uint8_t)(loader != NULL && shoot_sub != NULL);
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
        for (uint8_t index = 0u; index < friction_count; ++index)
            DJIMotorStop(friction_motors[index]);
        DJIMotorStop(loader);
    }
    else // 恢复运行
    {
        for (uint8_t index = 0u; index < friction_count; ++index)
            DJIMotorEnable(friction_motors[index]);
        DJIMotorEnable(loader);
    }

    // 如果上一次触发单发或3发指令的时间加上不应期仍然大于当前时间(尚未休眠完毕),直接返回即可
    // 单发模式主要提供给能量机关激活使用(以及英雄的射击大部分处于单发)
    // if (hibernate_time + dead_time > DWT_GetTimeline_ms())
    //     return;

    // 若不在休眠状态,根据robotCMD传来的控制模式进行拨盘电机参考值设定和模式切换
    // 发射槽数定义输出轴角度；每发转过整圈的1/slots，单位 rad。
    const uint8_t slots = shoot_config->loader_slot_num;
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
        DJIMotorSetRef(loader, -shoot_config->loader_reverse_speed_rad_s);
        break;
    default:
        // 未知供弹模式保持等待；此路径不会返回到任务中的下一次运行记录更新。
        while (1)
            ;
    }
    last_load_mode = shoot_cmd_recv.load_mode;

    if (shoot_cmd_recv.friction_mode == FRICTION_ON)
    {
        float friction_speed_rad_s = shoot_config->friction_default_speed_rad_s;
        switch (shoot_cmd_recv.bullet_speed)
        {
        case SMALL_AMU_15:
        case SMALL_AMU_18:
        case SMALL_AMU_30:
            friction_speed_rad_s = 0.0f;
            break;
        default:
            break;
        }
        for (uint8_t index = 0u; index < friction_count; ++index)
            DJIMotorSetRef(friction_motors[index], friction_speed_rad_s);
    }
    else // 关闭摩擦轮
    {
        for (uint8_t index = 0u; index < friction_count; ++index)
            DJIMotorSetRef(friction_motors[index], 0.0f);
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

    summary->friction_count = friction_count;
    if (friction_count > 0u)
        DJIMotorGetSummary(friction_motors[0], &summary->friction_l);
    if (friction_count > 1u)
        DJIMotorGetSummary(friction_motors[1], &summary->friction_r);
    if (friction_count > 2u)
        DJIMotorGetSummary(friction_motors[2], &summary->friction_extra[0]);
    if (friction_count > 3u)
        DJIMotorGetSummary(friction_motors[3], &summary->friction_extra[1]);
    DJIMotorGetSummary(loader, &summary->loader);
}
