#include "remote_bench.h"
#include "remote_safety.h"
#include "robot_safety.h"
#if MC02_BENCH_RC_DIAGNOSTICS && !defined(MC02_PROFILE_BENCH_SAFE)
#error "Remote diagnostics require bench_safe"
#endif
#if MC02_BENCH_RC_DIAGNOSTICS
static RC_ctrl_t *remote;
static RemoteSafetyGestures_s gestures;
void RemoteBenchInit(UART_HandleTypeDef *uart)
{
    RemoteSafetyGesturesInit(&gestures);
    remote = RemoteControlInit(uart);
}
void RemoteBenchTask(void)
{
    const uint8_t online = RemoteControlIsOnline();
    RobotSafetySetRcOnline(online);
    RemoteSafetyGesturesUpdate(&gestures, remote != NULL ? &remote[TEMP] : NULL,
                               online, HAL_GetTick(), 0u);
    RobotSafetyRequestDisable();
    RobotSafetyUpdate();
}
#endif
