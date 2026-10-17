/**
* @file referee_task.h
* @brief 声明裁判系统 UI 初始化、刷新和数据读取入口。
* 先成功注册裁判串口，再准备 UI 状态并运行任务。
 */
#ifndef REFEREE_H
#define REFEREE_H

#include "rm_referee.h"
#include "robot_def.h"

/**
* @brief 注册裁判串口并保存 UI 状态地址。
* @param referee_usart_handle 已初始化的 UART 句柄，不能为 NULL。
* @param UI_data UI 要读取的状态对象，调用方须一直保留该对象。
* @return 返回裁判接收数据地址；实际使用前须确保 RefereeInit 注册成功。
* @note 当前实现没有检查 RefereeInit 的失败返回值。
 */
referee_info_t *UITaskInit(UART_HandleTypeDef *referee_usart_handle, Referee_Interactive_info_t *UI_data);

/** @brief 取得裁判系统当前接收数据；尚未初始化时返回 NULL。 */
referee_info_t *RefereeGetInfo(void);

/**
* @brief 初始化 UI 图形和文字。
* @note 在裁判串口注册后、UITask 开始刷新前，从 FreeRTOS 任务上下文调用。
*       尚未初始化时会删除当前任务；机器人编号为0时每100ms等待一次。
 */
void MyUIInit();

/** @brief 根据当前状态刷新裁判系统客户端上的文字和图形。 */
void UITask();

#endif // REFEREE_H
