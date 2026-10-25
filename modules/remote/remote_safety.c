/** @file remote_safety.c
* @brief 保存三种开关组合的持续时间，将确认后的操作交给 RobotSafety。
*/
#include "remote_safety.h"
#include "robot_safety.h"
#include "bsp_reset_recovery.h"
#include <string.h>

void RemoteSafetyGesturesInit(RemoteSafetyGestures_s *state)
{
    memset(state, 0, sizeof(*state));
}

// 选中状态满 1000 ms 才报告一次；未选中则取消，报告后仍保持会再开始一轮计时。
static uint8_t Held(RemoteHold_s *hold, uint8_t selected, uint32_t now)
{
    if (!selected) { hold->active = 0u; return 0u; }
    if (!hold->active) { hold->start = now; hold->active = 1u; }
    if ((uint32_t)(now - hold->start) < 1000u) return 0u;
    hold->active = 0u;
    return 1u;
}

void RemoteSafetyGesturesUpdate(RemoteSafetyGestures_s *state, const RC_ctrl_t *input,
                               uint8_t online, uint32_t now, uint8_t allow_enable)
{
    // 裸板诊断等场合每轮都撤销使能，不让旧请求随手势恢复。
    if (!allow_enable) RobotSafetyRequestDisable();
    if (!online || input == NULL)
    {
        RemoteSafetyGesturesInit(state);
        return;
    }
    // 三种组合分别累计，开关离开对应位置后取消该组合的保持时间。
    uint8_t left = input->rc.switch_left, right = input->rc.switch_right;
    if (Held(&state->enable, (uint8_t)(allow_enable && left == RC_SW_DOWN && right == RC_SW_DOWN), now))
        RobotSafetyRequestEnable();
    if (Held(&state->disable, (uint8_t)(left == RC_SW_UP && right == RC_SW_UP), now))
        RobotSafetyRequestDisable();
    if (Held(&state->reset, (uint8_t)(left == RC_SW_MID && right == RC_SW_MID), now) &&
        EstopIsLatched() && EstopClear())
        BSP_ResetRecoverySetManualEstop(0u);
    if (input->rc.dial > 300)
    {
        const uint8_t was_manual_estop =
            (EstopIsLatched() != 0u && EstopGetReason() == ESTOP_REASON_MANUAL) ? 1u : 0u;
        EstopRequest(ESTOP_REASON_MANUAL);
        if (was_manual_estop == 0u)
            BSP_ResetRecoverySetManualEstop(1u);
    }
}
