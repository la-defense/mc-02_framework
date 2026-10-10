#include "remote_safety.h"
#include "robot_safety.h"
#include <string.h>

void RemoteSafetyGesturesInit(RemoteSafetyGestures_s *state)
{
    memset(state, 0, sizeof(*state));
}

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
    if (!allow_enable) RobotSafetyRequestDisable();
    if (!online || input == NULL)
    {
        RemoteSafetyGesturesInit(state);
        return;
    }
    uint8_t left = input->rc.switch_left, right = input->rc.switch_right;
    if (Held(&state->enable, (uint8_t)(allow_enable && left == RC_SW_DOWN && right == RC_SW_DOWN), now))
        RobotSafetyRequestEnable();
    if (Held(&state->disable, (uint8_t)(left == RC_SW_UP && right == RC_SW_UP), now))
        RobotSafetyRequestDisable();
    if (Held(&state->reset, (uint8_t)(left == RC_SW_MID && right == RC_SW_MID), now) && EstopIsLatched())
        (void)EstopClear();
    if (input->rc.dial > 300) EstopRequest(ESTOP_REASON_MANUAL);
}
