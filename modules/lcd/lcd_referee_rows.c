/**
 * @file lcd_referee_rows.c
 * @brief 依据各条裁判命令自己的期限，生成LCD只读页面数值。
 * 只使用RefereeSnapshot_t中已解码的数据，不把保留字节当作测量值。
 * @author kidneygood (you@domain.com)
 * @copyright Copyright (c) HNU YueLu EC 2022 all rights reserved
 */
#include "lcd_referee_rows.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

static void unavailable(char rows[LCD_REFEREE_VALUE_COUNT][LCD_REFEREE_VALUE_SIZE])
{
    for (size_t i=0u;i<LCD_REFEREE_VALUE_COUNT;++i) memcpy(rows[i],"--",3u);
}

void LCDRefereeFormatRows(const RefereeSnapshot_t *snapshot,
                          char rows[LCD_REFEREE_VALUE_COUNT][LCD_REFEREE_VALUE_SIZE])
{
    if (rows == NULL) return;
    unavailable(rows);
    if (snapshot == NULL) return;
    const RefereeCommandSnapshot_t *game=&snapshot->command[REF_CMD_GAME_STATE];
    const RefereeCommandSnapshot_t *robot=&snapshot->command[REF_CMD_ROBOT_STATE];
    const RefereeCommandSnapshot_t *power=&snapshot->command[REF_CMD_POWER_HEAT];
    const RefereeCommandSnapshot_t *ammo=&snapshot->command[REF_CMD_PROJECTILE_ALLOWANCE];
    const RefereeCommandSnapshot_t *shot=&snapshot->command[REF_CMD_SHOOT_DATA];

    if(game->state==REFEREE_DATA_FRESH)
        snprintf(rows[0],LCD_REFEREE_VALUE_SIZE,"G:%u T:%us",(unsigned)game->data.game_state.game_progress,
                 (unsigned)game->data.game_state.stage_remain_s);
    if(robot->state==REFEREE_DATA_FRESH) {
        snprintf(rows[1],LCD_REFEREE_VALUE_SIZE,"%u/%u",(unsigned)robot->data.robot_state.current_hp,
                 (unsigned)robot->data.robot_state.maximum_hp);
        // 裁判接收格式没有可信的实际功率字段，只显示机器人状态命令中的上限。
        snprintf(rows[2],LCD_REFEREE_VALUE_SIZE,"limit %uW",(unsigned)robot->data.robot_state.chassis_power_limit_w);
    }
    if(power->state==REFEREE_DATA_FRESH)
        snprintf(rows[3],LCD_REFEREE_VALUE_SIZE,"%u",(unsigned)power->data.power_heat.buffer_energy_j);
    if(power->state==REFEREE_DATA_FRESH && robot->state==REFEREE_DATA_FRESH)
        snprintf(rows[4],LCD_REFEREE_VALUE_SIZE,"%u/%u",(unsigned)power->data.power_heat.heat_17mm,
                 (unsigned)robot->data.robot_state.heat_limit);
    if(ammo->state==REFEREE_DATA_FRESH) {
        snprintf(rows[5],LCD_REFEREE_VALUE_SIZE,"%u/%u",(unsigned)ammo->data.projectile_allowance.allowance_17mm,
                 (unsigned)ammo->data.projectile_allowance.remaining_gold);
        snprintf(rows[7],LCD_REFEREE_VALUE_SIZE,"%u",(unsigned)ammo->data.projectile_allowance.remaining_gold);
    }
    if(shot->state==REFEREE_DATA_EVENT_RECORDED && isfinite(shot->data.shoot_data.initial_speed_mps))
        snprintf(rows[6],LCD_REFEREE_VALUE_SIZE,"last %.1f %lums",
                 (double)shot->data.shoot_data.initial_speed_mps,(unsigned long)shot->age_ms);
}
