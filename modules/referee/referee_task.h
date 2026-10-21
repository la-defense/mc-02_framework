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
typedef struct
{
    uint32_t chassis_flag : 1;
    uint32_t gimbal_flag : 1;
    uint32_t shoot_flag : 1;
    uint32_t lid_flag : 1;
    uint32_t friction_flag : 1;
    uint32_t Power_flag : 1;
    uint8_t refresh_flag : 2;
} Referee_Interactive_Flag_t;

/** @brief UI读取的应用状态；调用者需在整个UI服务期间保留该对象。 */
typedef struct
{
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

/** @brief UI流程阶段；运行只表示本机初始化帧已完成，不表示客户端已显示。 */
typedef enum
{
    UI_PHASE_UNBOUND,
    UI_PHASE_WAIT_ID,
    UI_PHASE_INITIALIZING,
    UI_PHASE_RUNNING,
    UI_PHASE_NEED_INIT,
    UI_PHASE_ERROR
} UIPhase_t;
typedef struct
{
    UIPhase_t phase;
    UIStatus_t last_error;
    uint32_t generation, pending_id;
    uint8_t init_step;
} UIState_t;
/** @brief 绑定UI长期数据，不注册接收；由UI任务串行初始化。
 * @param[in] UI_data 非NULL、长期保存；当前演示由UI任务独占写入。
 * @return UI_OK、参数错误或存在未完成报文时BUSY；失败保留原绑定。
 * @note 将来外部生产者写入必须另外同步，本接口不使任意共享指针自动一致。
 */
UIStatus_t UITaskInit(Referee_Interactive_info_t *UI_data);
/** @brief 推进一次发送及按115ms步长推进原演示；仅UI任务调用。
 * @return UI_OK表示本轮没有提交，UI_SUBMITTED表示一条已入队，其他值说明失败。
 */
UIStatus_t UITask(void);
/** @brief 推进一次初始化／增量发送，不修改演示数据，不等待传输。
 * @return 与UITask相同；身份过期后停止，恢复需要MyUIInit。
 */
UIStatus_t UIRefresh(void);
/** @brief 登记显式初始化请求，不发送、不等待；仅UI任务串行调用。
 * @return UI_SUBMITTED表示请求已接受，尚未绑定数据则NOT_INITIALIZED。
 * @note 取消旧代未完成报文，等待新鲜身份后按原顺序分15帧初始化。
 */
UIStatus_t MyUIInit(void);
/** @brief 复制只读阶段；查询不推进、不发帧。
 * @param[out] output 非NULL；空指针返回参数错误。
 * @return UI_OK或UI_ERROR_ARGUMENT。
 */
UIStatus_t UIGetState(UIState_t *output);
#endif
