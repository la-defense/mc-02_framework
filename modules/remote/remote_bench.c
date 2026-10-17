/** @file remote_bench.c
* @brief 裸板观察遥控及手势，持续请求失能；不用于驱动实车执行器。
*/
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
    // allow_enable 固定为 0，保留手势诊断过程，同时撤销使能请求。
    RemoteSafetyGesturesUpdate(&gestures, &snapshot.input,
                               online, HAL_GetTick(), 0u);
    // 更新状态前再次明确失能，避免其他来源留下的请求影响诊断。
    RobotSafetyRequestDisable();
    RobotSafetyUpdate();
}
#endif
