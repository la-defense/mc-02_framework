/** @file config.h
 * @brief 已生成配置的命令任务主机替身；所有数据由测试提供。
 */
#ifndef MC02_TEST_GENERATED_CONFIG_H
#define MC02_TEST_GENERATED_CONFIG_H
#include "config/build_selection.h"
#include "std_cmd_remote.h"
typedef struct {
    struct { struct { float yaw_offset, pitch_min_angle, pitch_max_angle; } gimbal_offset; } gimbal_param;
    struct { const StdCmdRemoteMap_t *map; uint32_t max_dt_ms; } manual_input;
} Robot_Config_s;
const Robot_Config_s *RobotConfigFeed(void);
#endif
