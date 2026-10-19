/**
 * @file lcd_referee_rows.h
 * @brief 将裁判只读数据副本整理为LCD页面使用的8行文字。
 * 纯格式化模块不访问屏幕、时钟或裁判接收对象，调用者提供完整输出区。
 * @author kidneygood (you@domain.com)
 * @copyright Copyright (c) HNU YueLu EC 2022 all rights reserved
 */
#ifndef LCD_REFEREE_ROWS_H
#define LCD_REFEREE_ROWS_H

#include "referee_snapshot.h"

#define LCD_REFEREE_VALUE_COUNT 8u
#define LCD_REFEREE_VALUE_SIZE 40u

/**
 * @brief 按各自命令状态生成比赛、血量、功率上限、热量和弹量文字。
 * @param[in] snapshot 一次读取所得的数据副本；可为NULL。
 * @param[out] rows 8行、每行40字节的可写区域；不得为NULL。
 * @return 无返回值；缺少任一必要命令或snapshot为空时对应文字为"--"。
 * @note 组合行只在相关命令都新鲜时生成；射速作为最近射击记录并附带年龄。
 */
void LCDRefereeFormatRows(const RefereeSnapshot_t *snapshot,
                          char rows[LCD_REFEREE_VALUE_COUNT][LCD_REFEREE_VALUE_SIZE]);

#endif
