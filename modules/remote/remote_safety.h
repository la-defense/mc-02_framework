/** @file remote_safety.h
* @brief 将遥控开关持续保持一秒的组合转为使能、失能或清急停请求。
*/
#ifndef REMOTE_SAFETY_H
#define REMOTE_SAFETY_H
#include "remote_control.h"
// start 为本次开始保持的毫秒时间；active 为是否正在累计。
typedef struct { uint32_t start; uint8_t active; } RemoteHold_s;
typedef struct { RemoteHold_s enable, disable, reset; } RemoteSafetyGestures_s;
/** @brief 清零手势计时；首次使用前调用。
* @param[out] state 调用方长期保存的状态，不能为 NULL。
*/
void RemoteSafetyGesturesInit(RemoteSafetyGestures_s *state);
/** @brief 更新一轮手势；离线或 input 为 NULL 时取消全部保持计时。
* @param[in,out] state 已初始化的状态，不能为 NULL。
* @param[in] input 当前遥控输入；两个开关下、上、中分别对应使能、失能、清急停。
* @param[in] online 非 0 表示在线，为 0 时不处理输入。
* @param[in] now 当前毫秒时间，同一手势连续调用须使用同一时间来源。
* @param[in] allow_enable 非 0 才允许使能；为 0 时每次都请求失能。
* @note 组合持续一秒触发一次，继续保持会重新计时；拨轮大于 300 时立即请求人工急停。
*/
void RemoteSafetyGesturesUpdate(RemoteSafetyGestures_s *state, const RC_ctrl_t *input,
                               uint8_t online, uint32_t now, uint8_t allow_enable);
#endif
