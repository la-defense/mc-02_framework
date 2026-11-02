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
#include "referee_ui_config.h"

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
/** @brief 绑定长期只读的车型布局，不注册或改变裁判接收。
* @param[in] config 非NULL、长期保存的静态布局，须先通过RefereeUIConfigIsValid。
* @return UI_OK、参数错误／配置错误，或存在未完成报文时BUSY；失败保留原绑定。
 * @note 字段来自一次复制的裁判数据；对应命令过期时显示“--”。
 */
UIStatus_t UITaskInit(const RefereeUIConfig_t *config);
/** @brief 推进一次有界 UI 状态和发送；仅 UI 任务调用，不等待传输。
 * @return UI_OK表示本轮没有提交，UI_SUBMITTED表示一条已入队，其他值说明失败。
 */
UIStatus_t UITask(void);
/** @brief 推进一次初始化／增量发送，使用当前裁判数据，不等待传输。
 * @return 与UITask相同；身份过期后停止，恢复有效时自动重绘。
 */
UIStatus_t UIRefresh(void);
/** @brief 登记显式初始化请求，不发送、不等待；仅UI任务串行调用。
 * @return UI_SUBMITTED表示请求已接受，尚未绑定数据则NOT_INITIALIZED。
 * @note 取消旧代未完成报文，等待有效身份后按配置字段逐帧重绘。
 */
UIStatus_t MyUIInit(void);
/** @brief 复制只读阶段；查询不推进、不发帧。
 * @param[out] output 非NULL；空指针返回参数错误。
 * @return UI_OK或UI_ERROR_ARGUMENT。
 */
UIStatus_t UIGetState(UIState_t *output);
#endif
