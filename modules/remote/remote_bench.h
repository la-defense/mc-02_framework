/** @file remote_bench.h
* @brief bench_safe 遥控诊断入口，仅在 MC02_BENCH_RC_DIAGNOSTICS 启用时提供实现。
*/
#ifndef REMOTE_BENCH_H
#define REMOTE_BENCH_H
#include "remote_control.h"
/** @brief 初始化手势计时并注册遥控串口。
* @param[in] port 遥控使用的逻辑 UART，未绑定时注册失败。
* @return 1 为注册成功，0 为失败；成功后才周期调用 RemoteBenchTask()。
*/
uint8_t RemoteBenchInit(PlatformUartPort_t port);
/** @brief 读取遥控、更新手势和诊断状态；每轮请求失能，始终不允许手势使能。 */
void RemoteBenchTask(void);
#endif
