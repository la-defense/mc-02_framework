/** @file platform_can_diagnostic.h
 * @brief 显式选择并触发板级 CAN 回环诊断。
 */
#ifndef MC02_PLATFORM_CAN_DIAGNOSTIC_H
#define MC02_PLATFORM_CAN_DIAGNOSTIC_H
#include <stdint.h>

// 默认不执行回环；启用时必须确保总线上没有连接执行器。
#ifndef CAN_SELFTEST_LOOPBACK
#define CAN_SELFTEST_LOOPBACK 0
#endif

#if CAN_SELFTEST_LOOPBACK
/** @brief 在所有设备注册后执行一次板级 CAN 回环诊断。
 * @return 1 表示原有诊断通过，0 表示失败。
 * @note 仅供授权的裸板诊断；后端可能切换总线模式并发送测试帧。
 */
uint8_t PlatformCanRunLoopbackDiagnostic(void);
#endif
#endif
