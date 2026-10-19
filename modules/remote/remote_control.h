/**
 * @file remote_control.h
 * @author DJI 2016
 * @author modified by neozng
 * @brief  遥控器模块定义头文件
 * @version beta
 * @date 2022-11-01
 *
 * @copyright Copyright (c) 2016 DJI corp
 * @copyright Copyright (c) 2022 HNU YueLu EC all rights reserved
 *
 */
#ifndef REMOTE_CONTROL_H
#define REMOTE_CONTROL_H

#include <stdint.h>
#include "main.h"
#include "usart.h"

// 用于遥控器数据读取,遥控器数据是一个大小为2的数组
#define LAST 1
#define TEMP 0

// 获取按键操作
#define KEY_PRESS 0
#define KEY_STATE 1
#define KEY_PRESS_WITH_CTRL 1
#define KEY_PRESS_WITH_SHIFT 2

// 检查接收值是否出错
#define RC_CH_VALUE_MIN ((uint16_t)364)
#define RC_CH_VALUE_OFFSET ((uint16_t)1024)
#define RC_CH_VALUE_MAX ((uint16_t)1684)

/* ----------------------- RC Switch Definition----------------------------- */
#define RC_SW_UP ((uint16_t)1)   // 开关向上时的值
#define RC_SW_MID ((uint16_t)3)  // 开关中间时的值
#define RC_SW_DOWN ((uint16_t)2) // 开关向下时的值
// 三个判断开关状态的宏
#define switch_is_down(s) (s == RC_SW_DOWN)
#define switch_is_mid(s) (s == RC_SW_MID)
#define switch_is_up(s) (s == RC_SW_UP)

/* ----------------------- PC Key Definition-------------------------------- */
// 键盘位号，同时作为 key_count 的第二个下标；key[].w 等字段读取当前按键。
#define Key_W 0
#define Key_S 1
// Official RoboMaster Client 5.0.2.37; verified through DT7/DR16 (MC02-0049).
#define Key_A 2
#define Key_D 3
#define Key_Shift 4
#define Key_Ctrl 5
#define Key_Q 6
#define Key_E 7
#define Key_R 8
#define Key_F 9
#define Key_G 10
#define Key_Z 11
#define Key_X 12
#define Key_C 13
#define Key_V 14
#define Key_B 15

/* ----------------------- Data Struct ------------------------------------- */
// 同一组按键可以按名称读取，也可以通过 keys 一次读取全部 16 位。
typedef union
{
    struct // 用于访问键盘状态
    {
        uint16_t w : 1;
        uint16_t s : 1;
        uint16_t a : 1;
        uint16_t d : 1;
        uint16_t shift : 1;
        uint16_t ctrl : 1;
        uint16_t q : 1;
        uint16_t e : 1;
        uint16_t r : 1;
        uint16_t f : 1;
        uint16_t g : 1;
        uint16_t z : 1;
        uint16_t x : 1;
        uint16_t c : 1;
        uint16_t v : 1;
        uint16_t b : 1;
    };
    uint16_t keys; // 用于memcpy而不需要进行强制类型转换
} Key_t;

// @todo 当前结构体嵌套过深,需要进行优化
typedef struct
{
    struct
    {
        int16_t rocker_l_; // 左水平
        int16_t rocker_l1; // 左竖直
        int16_t rocker_r_; // 右水平
        int16_t rocker_r1; // 右竖直
        int16_t dial;      // 侧边拨轮

        uint8_t switch_left;  // 左侧开关
        uint8_t switch_right; // 右侧开关
    } rc;
    struct
    {
        int16_t x;
        int16_t y;
        uint8_t press_l;
        uint8_t press_r;
    } mouse;

    Key_t key[3]; // 普通、配合 Ctrl、配合 Shift 的三组按键状态。

    uint8_t key_count[3][16];
} RC_ctrl_t;

/* ------------------------- Internal Data ----------------------------------- */

/** @brief 注册遥控 UART 和离线计时，清空旧输入。
* @param[in] rc_usart_handle 已配置好的遥控 UART 句柄，不能为 NULL。
* @return 成功返回模块内部两个 RC_ctrl_t 对象的首地址，长期有效且不可释放；失败返回 NULL。
* @note 只初始化一次；读取控制输入优先使用 RemoteControlGetSnapshot()，不直接改写内部对象。
 */
RC_ctrl_t *RemoteControlInit(UART_HandleTypeDef *rc_usart_handle);

/**
 * @brief 检查合法18字节块的来源年龄及看门狗，未初始化为离线。
 *
 * @return uint8_t 1:在线 0:离线
 * @note 原RxEvent时间起100ms边界允许，超过后清输入、撤销许可并请求急停；查询不续期。
 * 仅任务调用；本函数可能执行离线重启接收及日志。
 */
uint8_t RemoteControlIsOnline();

#define REMOTE_PC_LEASE_MS 150u
typedef enum
{
    REMOTE_PC_REVOKE_NONE = 0,
    REMOTE_PC_REVOKE_INACTIVE = 1,
    REMOTE_PC_REVOKE_TIMEOUT = 2,
    REMOTE_PC_REVOKE_RADIO_LOST = 3,
} RemotePcRevokeReason_e;
typedef struct
{
    RC_ctrl_t input;
    uint8_t remote_online;
    uint8_t pc_valid;
} RemoteControlSnapshot_s;
/** @brief 一次复制当前输入及可用标记，并检查键鼠是否已超过 150 ms 未更新。
* @param[out] snapshot 调用方的输出对象，NULL 时直接返回；读取期间暂时关闭普通中断并恢复原状态。
* @note 每个命令周期调用，即使没有新串口帧；摇杆在线与键鼠可用分别由 remote_online、pc_valid 表示。
*       键鼠失效时清零；恢复要求连续两帧所有键鼠松开，且心跳编号向前变化。等于 150 ms 仍允许。
 */
void RemoteControlGetSnapshot(RemoteControlSnapshot_s *snapshot);

// 调试记录；时间单位为 HAL 毫秒，次数为累计次数，不参与调用方直接控制。
typedef struct
{
    uint32_t accepted_frames;
    uint32_t rejected_length;
    uint32_t rejected_fields;
    uint32_t last_valid_tick; // 最近合法块的原RxEvent时间，0也合法。
    uint32_t offline_tick;
    uint32_t offline_last_valid_tick;
    uint32_t offline_events;
    uint32_t has_valid_frame;
    uint32_t online;
    // 校验整帧后记录的原始键鼠字段；10、11 字节合成心跳编号，保存时扩成 32 位。
    uint32_t raw_mouse_z;
    uint32_t raw_key_bitmap;
    uint32_t raw_mouse_x;
    uint32_t raw_mouse_y;
    uint32_t raw_mouse_left;
    uint32_t raw_mouse_right;
    uint32_t pc_authorized;
    uint32_t pc_heartbeat;
    uint32_t pc_recovery_count;
    uint32_t pc_last_advance_tick; // 最近向前心跳的原RxEvent时间，独立于派发和撤销时刻。
    uint32_t pc_expire_tick;
    uint32_t pc_expirations;
    uint32_t pc_revoke_tick;
    uint32_t pc_revoke_reason;
} RemoteControlDiagnostics_s;
extern volatile RemoteControlDiagnostics_s remote_control_diag;

#endif
