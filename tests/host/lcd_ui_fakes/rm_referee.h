/** @file rm_referee.h @brief LCD UI 主机绘制测试的裁判快照读取声明替身。 */
#ifndef MC02_LCD_UI_FAKE_RM_REFEREE_H
#define MC02_LCD_UI_FAKE_RM_REFEREE_H
#include "lcd_ui_test_types.h"
#include "referee_snapshot.h"
typedef enum { REFEREE_OK, REFEREE_ERROR_ARGUMENT, REFEREE_ERROR_NOT_INITIALIZED } RefereeStatus_t;
RefereeStatus_t RefereeGetSnapshot(RefereeSnapshot_t *output);
#endif
