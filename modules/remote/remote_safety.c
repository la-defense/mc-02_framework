/** @file remote_safety.c
* @brief 保存三种开关组合的持续时间，将确认后的操作交给 RobotSafety。
*/
#include "remote_safety.h"
#include "robot_safety.h"
#include <string.h>

#define REMOTE_SAFETY_HOLD_MS 1000u // 有效的两个开关组合必须持续一秒。

void RemoteSafetyGesturesInit(RemoteSafetyGestures_s *state)
{
    memset(state, 0, sizeof(*state));
}

// 选中状态满 1000 ms 才报告一次；未选中则取消，报告后仍保持会再开始一轮计时。
static uint8_t Held(RemoteHold_s *hold, uint8_t selected, uint32_t now)
{
    if (!selected) { hold->active = 0u; return 0u; }
    if (!hold->active) { hold->start = now; hold->active = 1u; }
    if ((uint32_t)(now - hold->start) < REMOTE_SAFETY_HOLD_MS) return 0u;
    hold->active = 0u;
    return 1u;
}

void RemoteSafetyGesturesUpdate(RemoteSafetyGestures_s *state, const RC_ctrl_t *input,
                               uint8_t online, uint32_t now, uint8_t allow_enable, uint32_t switch_generation)
{
    // 裸板诊断等场合每轮都撤销使能，不让旧请求随手势恢复。
    if (!allow_enable) RobotSafetyRequestDisable();
    if (!online || input == NULL)
    {
        RemoteSafetyGesturesInit(state);
        return;
    }
    if (input->rc.dial > REMOTE_ESTOP_DIAL_THRESHOLD) // 拨轮触发急停时，开关组合不能继续累计或解除急停。
    {
        RemoteSafetyGesturesInit(state); // 释放拨轮后再从第一次有效读取开始计时。
        EstopRequest(ESTOP_REASON_MANUAL); // 立即禁止输出，重复请求保持原锁存。
        return; // 本轮不处理任何双开关手势。
    }
    if (state->switch_generation != switch_generation) // 两次读取之间开关曾变化，或接收端经历了断线重连。
    {
        RemoteSafetyGesturesInit(state); // 结束旧的保持区间，当前输入可重新开始计时。
        state->switch_generation = switch_generation; // 记住本段计时对应的开关变化计数。
    }
    // 三种组合分别累计，开关离开对应位置后取消该组合的保持时间。
    uint8_t left = input->rc.switch_left, right = input->rc.switch_right;
    if (Held(&state->enable, (uint8_t)(allow_enable && left == RC_SW_DOWN && right == RC_SW_DOWN), now))
        RobotSafetyRequestEnable();
    if (Held(&state->disable, (uint8_t)(left == RC_SW_UP && right == RC_SW_UP), now))
        RobotSafetyRequestDisable();
    if (Held(&state->reset, (uint8_t)(left == RC_SW_MID && right == RC_SW_MID), now) && EstopIsLatched())
        (void)EstopClearAfterDialRelease(REMOTE_SAFETY_HOLD_MS); // 旧手势即使已满一秒，也不能清除刚收到的拨轮急停。
}
