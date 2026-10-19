/**
* @file referee_task.h
* @brief 声明裁判系统 UI 初始化、刷新和数据读取入口。
* 先成功注册裁判串口，再准备 UI 状态并运行任务。
 */
#ifndef REFEREE_H
#define REFEREE_H

#include "rm_referee.h"
#include "referee_UI.h"
#include "robot_def.h"

/**
* @brief 注册裁判串口并保存 UI 状态地址。
* @param referee_usart_handle 已初始化的 UART 句柄，不能为 NULL。
* @param UI_data UI 要读取的状态对象，调用方须一直保留该对象。
* @return 成功返回裁判接收数据地址；空状态或注册失败返回NULL，不改原有效绑定。
* @note UI_data须长期保存；注册成功只表示接收可用，UI仍需MyUIInit显式初始化。
 */
referee_info_t *UITaskInit(UART_HandleTypeDef *referee_usart_handle, Referee_Interactive_info_t *UI_data);

/** @brief 取得裁判系统当前接收数据；尚未初始化时返回 NULL。 */
referee_info_t *RefereeGetInfo(void);

/**
* @brief 初始化 UI 图形和文字。
* @return 全部帧被本机接受返回UI_SUBMITTED；否则返回具体错误并停用后续UI刷新。
* @note 注册成功且已收到机器人编号后，由裁判任务串行调用；没有自动重试。
*       部分帧可能已经发送，失败不撤销接收服务，再次调用会重新尝试整轮初始化。
 */
UIStatus_t MyUIInit(void);

/** @brief 根据当前演示状态提交变化项；保留原有RobotModeTest演示行为。
 * @return 本轮无待刷新或全部接受返回UI_OK；失败返回原因并保留未接受项的刷新标记。
 * @note 先成功MyUIInit；同一任务串行调用，未就绪返回UI_NOT_INITIALIZED且不发送。
 */
UIStatus_t UITask(void);

/** @brief 只根据已绑定的应用状态提交变化项，不执行RobotModeTest演示状态改写。
 * @return 全部待刷新项接受或无变化返回UI_OK；失败返回原因并保留未接受项标记。
 * @note 先成功MyUIInit；由同一裁判任务串行调用，应用状态生命周期覆盖全部调用。
 * 不创建额外任务，不刷新来源时间；UITask沿用演示行为后调用本入口。
 */
UIStatus_t UIRefresh(void);

#endif // REFEREE_H
