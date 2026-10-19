/**
 * @file referee_snapshot.h
 * @brief 保存裁判协议解码后的只读数据及每条命令的接收状态。
 * 该头文件只依赖标准整数类型，不引用 HAL、RTOS 或 UI 类型。
 * @author kidneygood (you@domain.com)
 * @version 0.1
 * @date 2022-12-02
 * @copyright Copyright (c) HNU YueLu EC 2022 all rights reserved
 */
#ifndef REFEREE_SNAPSHOT_H
#define REFEREE_SNAPSHOT_H

#include <stdint.h>

/** @brief 当前接收模块支持的裁判命令索引；数组顺序同时用于快照查询。 */
typedef enum {
    REF_CMD_GAME_STATE = 0, REF_CMD_GAME_RESULT, REF_CMD_ROBOT_HP,
    REF_CMD_EVENT_DATA, REF_CMD_WARNING, REF_CMD_DART_INFO,
    REF_CMD_ROBOT_STATE, REF_CMD_POWER_HEAT, REF_CMD_ROBOT_POSITION,
    REF_CMD_BUFF, REF_CMD_ROBOT_HURT, REF_CMD_SHOOT_DATA,
    REF_CMD_PROJECTILE_ALLOWANCE, REF_CMD_RFID, REF_CMD_DART_CLIENT,
    REF_CMD_GROUND_POSITION, REF_CMD_RADAR_MARK, REF_CMD_SENTRY_INFO,
    REF_CMD_RADAR_INFO, REF_CMD_MAP_COMMAND, REF_CMD_INTERACTIVE,
    REFEREE_COMMAND_COUNT
} RefereeCommandIndex_t;

/** @brief 一条命令的数据状态；事件状态只表示最近记录，不代表操作仍有效。 */
typedef enum {
    REFEREE_DATA_UNSEEN = 0,
    REFEREE_DATA_FRESH,
    REFEREE_DATA_EXPIRED,
    REFEREE_DATA_EVENT_RECORDED
} RefereeDataState_t;

/** @brief 机器人状态输出许可位；0x0201第16字节按这些掩码解释。 */
#define REFEREE_OUTPUT_GIMBAL_MASK 0x01u
#define REFEREE_OUTPUT_CHASSIS_MASK 0x02u
#define REFEREE_OUTPUT_SHOOTER_MASK 0x04u

/** @brief 比赛类型占用bit0～3，阶段占用bit4～7。 */
#define REFEREE_GAME_TYPE_MASK 0x0fu
#define REFEREE_GAME_PROGRESS_SHIFT 4u
/** @brief 飞镖状态按低位起依次存目标、命中计数和选定目标。 */
#define REFEREE_DART_LAST_TARGET_MASK 0x07u
#define REFEREE_DART_HIT_COUNT_SHIFT 3u
#define REFEREE_DART_HIT_COUNT_MASK 0x07u
#define REFEREE_DART_SELECTED_TARGET_SHIFT 6u
#define REFEREE_DART_SELECTED_TARGET_MASK 0x07u
/** @brief 0x0204能量反馈从bit0到bit6依次代表125%、100%、50%、30%、15%、5%、1%。 */
#define REFEREE_ENERGY_125_PERCENT_MASK 0x01u
#define REFEREE_ENERGY_100_PERCENT_MASK 0x02u
#define REFEREE_ENERGY_50_PERCENT_MASK 0x04u
#define REFEREE_ENERGY_30_PERCENT_MASK 0x08u
#define REFEREE_ENERGY_15_PERCENT_MASK 0x10u
#define REFEREE_ENERGY_5_PERCENT_MASK 0x20u
#define REFEREE_ENERGY_1_PERCENT_MASK 0x40u

/** @brief 比赛阶段剩余秒数；同步时间是协议中的UNIX秒字段，不能与本机毫秒比较。 */
typedef struct { uint8_t game_type, game_progress; uint16_t stage_remain_s; uint64_t sync_timestamp; } RefereeGameState_t;
typedef struct { uint8_t winner; } RefereeGameResult_t;
/** @brief hp数组顺序为己方1、2、3、4号、己方7号、己方前哨站/基地、对方前哨站/基地。 */
typedef struct { uint16_t hp[9]; int16_t damage_difference; } RefereeRobotHp_t;
/** @brief 0x0101按bit0～31保存赛事事件状态，字段值按小端解码。 */
typedef struct { uint32_t event_bits; } RefereeEventData_t;
typedef struct { uint8_t level, offending_robot_id, count; } RefereeWarning_t;
/** @brief 目标和命中计数按头文件掩码读取；remaining_s单位为秒。 */
typedef struct { uint8_t remaining_s; uint16_t info_bits; } RefereeDartInfo_t;
/** @brief 机器人编号、血量和能力上限；弹速单位m/s，功率上限单位W。 */
typedef struct {
    uint8_t robot_id, robot_level;
    uint16_t current_hp, maximum_hp, cooling_per_s, heat_limit, chassis_power_limit_w;
    float bullet_speed_limit_mps;
    uint8_t power_output_bits;
} RefereeRobotState_t;
/** @brief 0x0202前8字节原样保留；缓冲能量单位J，热量是枪管读数。 */
typedef struct { uint8_t reserved[8]; uint16_t buffer_energy_j, heat_17mm, heat_42mm; } RefereePowerHeat_t;
/** @brief 坐标单位m，方向单位度，正北为0度。 */
typedef struct { float x_m, y_m, angle_deg; } RefereeRobotPosition_t;
/** @brief 能量反馈保持原位图；各阈值对应REFEREE_ENERGY_*掩码。 */
typedef struct {
    uint8_t recovery_buff; uint16_t cooling_buff; uint8_t defence_buff, vulnerability_buff;
    uint16_t attack_buff; uint8_t remaining_energy_bits;
} RefereeBuff_t;
/** @brief 装甲／测速模块编号位0～3，扣血原因位4～7。 */
typedef struct { uint8_t armor_id, hurt_type; } RefereeRobotHurt_t;
/** @brief 最近一次发射记录；射速单位Hz，初速度单位m/s。 */
typedef struct { uint8_t bullet_type, shooter_number, frequency_hz; float initial_speed_mps; } RefereeShootData_t;
typedef struct { uint16_t allowance_17mm, allowance_42mm, remaining_gold, allowance_fortress; } RefereeProjectileAllowance_t;
typedef struct { uint32_t status_bits; uint8_t extra_status; } RefereeRfid_t;
typedef struct { uint8_t opening_status, reserved; uint16_t target_change_time, latest_launch_time; } RefereeDartClient_t;
/** @brief 8个有效位置依次为英雄、工程、3号步兵、4号步兵的x/y米坐标。 */
typedef struct { float position_m[8]; uint8_t reserved[8]; } RefereeGroundPosition_t;
/** @brief 0x020C按协议bit0～15记录双方易伤及空中机器人状态。 */
typedef struct { uint16_t mark_bits; } RefereeRadarMark_t;
/** @brief 0x020D三段信息均为小端位图，长度分别为32、16、64位。 */
typedef struct { uint32_t info_bits_1; uint16_t info_bits_2; uint64_t info_bits_3; } RefereeSentryInfo_t;
/** @brief 0x020E低两位是机会数，其余位含双倍易伤和加密等级等标志。 */
typedef struct { uint8_t info_bits; } RefereeRadarInfo_t;
typedef struct { float target_x_m, target_y_m; uint8_t keyboard_bits, target_robot_id; uint16_t source_id; } RefereeMapCommand_t;
typedef struct { uint16_t content_id, sender_id, receiver_id; uint8_t data[5]; } RefereeInteractive_t;

/** @brief 不同命令解码后的字段；保留字段以原始字节保存，不作物理量解释。 */
typedef union {
    RefereeGameState_t game_state;
    RefereeGameResult_t game_result;
    RefereeRobotHp_t robot_hp;
    RefereeEventData_t event_data;
    RefereeWarning_t warning;
    RefereeDartInfo_t dart_info;
    RefereeRobotState_t robot_state;
    RefereePowerHeat_t power_heat;
    RefereeRobotPosition_t robot_position;
    RefereeBuff_t buff;
    RefereeRobotHurt_t robot_hurt;
    RefereeShootData_t shoot_data;
    RefereeProjectileAllowance_t projectile_allowance;
    RefereeRfid_t rfid;
    RefereeDartClient_t dart_client;
    RefereeGroundPosition_t ground_position;
    RefereeRadarMark_t radar_mark;
    RefereeSentryInfo_t sentry_info;
    RefereeRadarInfo_t radar_info;
    RefereeMapCommand_t map_command;
    RefereeInteractive_t interactive;
} RefereeCommandData_t;

/** @brief 单条命令的数据及其来源；time为首个组帧字节的本机毫秒时间，0合法。 */
typedef struct {
    RefereeDataState_t state;
    uint32_t source_rx_ms;
    uint32_t age_ms;
    uint32_t receive_count;
    uint8_t wire_sequence;
    RefereeCommandData_t data;
} RefereeCommandSnapshot_t;

/** @brief 一次读取所得的全部命令、各自期限状态和整体链路诊断。 */
typedef struct {
    uint8_t link_online;
    uint8_t has_valid_frame;
    uint32_t link_source_rx_ms;
    uint32_t link_age_ms;
    RefereeCommandSnapshot_t command[REFEREE_COMMAND_COUNT];
} RefereeSnapshot_t;

#endif
