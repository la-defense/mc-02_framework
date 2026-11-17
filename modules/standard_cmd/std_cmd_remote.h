/** @file std_cmd_remote.h
 * @brief 将已解析的遥控或键鼠状态交给人工意图换算，不选择模式或使能机器人。
 */
#ifndef MC02_STD_CMD_REMOTE_H
#define MC02_STD_CMD_REMOTE_H
#include "std_cmd.h"
#include "remote_control.h"

/** @brief 用于某个运动通道的遥控器原始输入。 */
typedef enum {
    STD_CMD_RC_ZERO = 0, /**< 不使用输入，输出原始值零。 */
    STD_CMD_RC_LEFT_X, /**< 左摇杆水平。 */
    STD_CMD_RC_LEFT_Y, /**< 左摇杆竖直。 */
    STD_CMD_RC_RIGHT_X, /**< 右摇杆水平。 */
    STD_CMD_RC_RIGHT_Y, /**< 右摇杆竖直。 */
    STD_CMD_RC_DIAL, /**< 遥控器侧拨轮。 */
    STD_CMD_RC_AXIS_COUNT
} StdCmdRemoteAxis_t;

/** @brief 各车型人工输入映射；数值系数与机械限制必须单独确认。 */
typedef struct {
    StdCmdManualMap_t remote_map; /**< 遥控原始通道到各 SI 单位的配置。 */
    StdCmdManualMap_t keyboard_map; /**< 键盘方向值 -1、0、1 到各 SI 单位的配置。 */
    StdCmdRemoteAxis_t remote_axis[STD_CMD_AXIS_COUNT]; /**< 每个运动轴使用哪一路遥控输入。 */
    uint16_t keyboard_positive[STD_CMD_AXIS_COUNT]; /**< 每个运动轴正向的按键位集合。 */
    uint16_t keyboard_negative[STD_CMD_AXIS_COUNT]; /**< 每个运动轴负向的按键位集合。 */
    StdCmdFrame_t remote_frame; /**< 遥控平移采用的方向参照。 */
    StdCmdFrame_t keyboard_frame; /**< 键盘平移采用的方向参照。 */
} StdCmdRemoteMap_t;

/** @brief 使用当前已选择档位换算人工连续输入。
 * @param[in] map 已确认并按车型提供的映射，不能为空。
 * @param[in] remote 遥控模块一次读取的输入和原始来源时间，不能为空。
 * @param[in] now_ms 当前本机毫秒，允许 32 位回绕。
 * @param[out] output 只在全部检查成功后改写；须与输入和配置分开存放。
 * @return 换算状态；离线、错误档位、无有效来源或非法映射均不交付结果。
 * @note 向下使用摇杆，向上使用已授权键盘；中档不提供人工运动。键鼠还要求无线来源新鲜。
 * 调用者必须先确认当前模式允许人工来源；有些协议将向上档用于自主模式。
 * 鼠标位移是按次事件，继续通过独立队列消费，不在这里作为持续角速度。
 */
StdCmdStatus_t StdCmdAdaptRemote(const StdCmdRemoteMap_t *map, const RemoteControlSnapshot_s *remote,
                                 uint32_t now_ms, StdCmdManualIntent_t *output);
#endif
