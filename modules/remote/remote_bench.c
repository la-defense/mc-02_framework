#include "remote_bench.h"
#include "remote_safety.h"
#include "robot_safety.h"
#if MC02_BENCH_RC_DIAGNOSTICS && !defined(MC02_PROFILE_BENCH_SAFE)
#error "Remote diagnostics require bench_safe"
#endif
#if MC02_BENCH_RC_DIAGNOSTICS
static RemoteControlSnapshot_s snapshot;
static RemoteSafetyGestures_s gestures;
uint8_t RemoteBenchInit(UART_HandleTypeDef *uart)
{
    RemoteSafetyGesturesInit(&gestures);
    return (uint8_t)(RemoteControlInit(uart) != NULL);
}
void RemoteBenchTask(void)
{
    RemoteControlGetSnapshot(&snapshot);
    const uint8_t online = snapshot.remote_online;
    RobotSafetySetRcOnline(online);
    RemoteSafetyGesturesUpdate(&gestures, &snapshot.input,
                               online, HAL_GetTick(), 0u);
    RobotSafetyRequestDisable();
    RobotSafetyUpdate();
}
#endif
