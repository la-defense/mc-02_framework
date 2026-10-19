/** @file lcd_referee_rows_test.c
 * @brief 验证LCD按裁判命令期限显示旧数值、缺失行及最近射击记录。
 * @author kidneygood (you@domain.com)
 * @copyright Copyright (c) HNU YueLu EC 2022 all rights reserved
 */
#include <math.h>
#include <string.h>
#include "lcd_referee_rows.h"
#include "test_check.h"

int main(void)
{
    RefereeSnapshot_t snapshot={0};
    char rows[LCD_REFEREE_VALUE_COUNT][LCD_REFEREE_VALUE_SIZE];
    LCDRefereeFormatRows(NULL,rows);
    for(size_t i=0u;i<LCD_REFEREE_VALUE_COUNT;++i) TEST_CHECK(strcmp(rows[i],"--")==0);
    snapshot.command[REF_CMD_GAME_STATE].state=REFEREE_DATA_FRESH;
    snapshot.command[REF_CMD_GAME_STATE].data.game_state.game_progress=3u;
    snapshot.command[REF_CMD_GAME_STATE].data.game_state.stage_remain_s=42u;
    snapshot.command[REF_CMD_ROBOT_STATE].state=REFEREE_DATA_FRESH;
    snapshot.command[REF_CMD_ROBOT_STATE].data.robot_state.current_hp=100u;
    snapshot.command[REF_CMD_ROBOT_STATE].data.robot_state.maximum_hp=200u;
    snapshot.command[REF_CMD_ROBOT_STATE].data.robot_state.chassis_power_limit_w=60u;
    snapshot.command[REF_CMD_ROBOT_STATE].data.robot_state.heat_limit=100u;
    snapshot.command[REF_CMD_POWER_HEAT].state=REFEREE_DATA_FRESH;
    snapshot.command[REF_CMD_POWER_HEAT].data.power_heat.reserved[0]=0xffu;
    snapshot.command[REF_CMD_POWER_HEAT].data.power_heat.buffer_energy_j=50u;
    snapshot.command[REF_CMD_POWER_HEAT].data.power_heat.heat_17mm=25u;
    snapshot.command[REF_CMD_PROJECTILE_ALLOWANCE].state=REFEREE_DATA_FRESH;
    snapshot.command[REF_CMD_PROJECTILE_ALLOWANCE].data.projectile_allowance.allowance_17mm=30u;
    snapshot.command[REF_CMD_PROJECTILE_ALLOWANCE].data.projectile_allowance.remaining_gold=40u;
    snapshot.command[REF_CMD_SHOOT_DATA].state=REFEREE_DATA_EVENT_RECORDED;
    snapshot.command[REF_CMD_SHOOT_DATA].data.shoot_data.initial_speed_mps=18.5f;
    snapshot.command[REF_CMD_SHOOT_DATA].age_ms=1234u;
    LCDRefereeFormatRows(&snapshot,rows);
    TEST_CHECK(strcmp(rows[0],"G:3 T:42s")==0 && strcmp(rows[1],"100/200")==0);
    TEST_CHECK(strcmp(rows[2],"limit 60W")==0 && strcmp(rows[3],"50")==0);
    TEST_CHECK(strcmp(rows[4],"25/100")==0 && strcmp(rows[5],"30/40")==0);
    TEST_CHECK(strcmp(rows[6],"last 18.5 1234ms")==0 && strcmp(rows[7],"40")==0);
    snapshot.command[REF_CMD_POWER_HEAT].state=REFEREE_DATA_EXPIRED;
    LCDRefereeFormatRows(&snapshot,rows);
    TEST_CHECK(strcmp(rows[3],"--")==0 && strcmp(rows[4],"--")==0);
    TEST_CHECK(strcmp(rows[2],"limit 60W")==0 && strcmp(rows[5],"30/40")==0);
    snapshot.command[REF_CMD_POWER_HEAT].state=REFEREE_DATA_FRESH;
    snapshot.command[REF_CMD_ROBOT_STATE].state=REFEREE_DATA_EXPIRED;
    LCDRefereeFormatRows(&snapshot,rows);
    TEST_CHECK(strcmp(rows[1],"--")==0 && strcmp(rows[2],"--")==0 && strcmp(rows[4],"--")==0);
    TEST_CHECK(strcmp(rows[3],"50")==0 && strcmp(rows[5],"30/40")==0);
    snapshot.command[REF_CMD_ROBOT_STATE].state=REFEREE_DATA_FRESH;
    snapshot.command[REF_CMD_SHOOT_DATA].data.shoot_data.initial_speed_mps=NAN;
    LCDRefereeFormatRows(&snapshot,rows);
    TEST_CHECK(strcmp(rows[6],"--")==0);
    return 0;
}
