/**
 * @file referee_task.h
 * @brief 声明裁判UI状态绑定、初始化和刷新入口。
 * 裁判UART由RefereeInit单独注册，只有开启MC02_ENABLE_REFEREE_UI才创建UI任务。
 * @author kidneygood (you@domain.com)
 * @version 0.1
 * @date 2022-12-02
 * @copyright Copyright (c) HNU YueLu EC 2022 all rights reserved
 */
#ifndef REFEREE_TASK_H
#define REFEREE_TASK_H

#include "rm_referee.h"
#include "referee_UI.h"
#include "robot_def.h"

/** @brief UI待刷新位；每一位对应一种演示或功率显示内容。 */
typedef struct {
    uint32_t chassis_flag : 1;
    uint32_t gimbal_flag : 1;
    uint32_t shoot_flag : 1;
    uint32_t lid_flag : 1;
    uint32_t friction_flag : 1;
    uint32_t Power_flag : 1;
    uint8_t refresh_flag : 2;
} Referee_Interactive_Flag_t;

/** @brief UI读取的应用状态；调用者需在整个UI服务期间保留该对象。 */
typedef struct {
    Referee_Interactive_Flag_t Referee_Interactive_Flag;
    chassis_mode_e chassis_mode;
    gimbal_mode_e gimbal_mode;
    shoot_mode_e shoot_mode;
    friction_mode_e friction_mode;
    lid_mode_e lid_mode;
    Chassis_Power_Data_s Chassis_Power_Data;
    chassis_mode_e chassis_last_mode;
    gimbal_mode_e gimbal_last_mode;
    shoot_mode_e shoot_last_mode;
    friction_mode_e friction_last_mode;
    lid_mode_e lid_last_mode;
    Chassis_Power_Data_s Chassis_last_Power_Data;
    Bullet_Speed_e bullet_speed;
    float chassis_offset_angle;
    float pitch_angle;
} Referee_Interactive_info_t;

/**
 * @brief 绑定长期保存的应用状态，不注册或修改裁判接收服务。
 * @param[in] UI_data UI要显示的应用状态；必须非NULL并保持到UI停止。
 * @return UI_OK或UI_ERROR_ARGUMENT；失败保留原绑定。
 * @note 先成功RefereeInit再运行UI任务；本入口不证明已收到裁判数据。
 */
UIStatus_t UITaskInit(Referee_Interactive_info_t *UI_data);

/**
 * @brief 根据最新应用状态逐项提交显示更新。
 * @return 本轮完成返回UI_OK，身份／来源失效、未初始化或提交失败返回具体状态。
 * @note 由一个任务串行调用；实际发送前要求0x0201新鲜且机器人身份与初始化时相同。
 */
UIStatus_t UITask(void);

/** @brief 提交应用状态变化，不执行RobotModeTest演示状态改写。 */
UIStatus_t UIRefresh(void);

/**
 * @brief 根据当前新鲜机器人编号重新初始化整套UI图形。
 * @return 所有初始化帧均被本机接受返回UI_SUBMITTED，否则返回具体失败状态。
 * @note 接收服务必须已注册；身份变化或过期后，只有再次显式调用本入口才重新启用UI。
 */
UIStatus_t MyUIInit(void);

#endif
