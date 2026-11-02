/** @file dji_motor_diagnostic_internal.h
 * @brief 供板级回环诊断取得第一个已注册的 DJI 电机。
 */
#ifndef MC02_DJI_MOTOR_DIAGNOSTIC_INTERNAL_H
#define MC02_DJI_MOTOR_DIAGNOSTIC_INTERNAL_H
#include "dji_motor.h"

#if CAN_DJI_INJECT_TEST >= 1
/** @brief 查询回环诊断使用的电机。
 * @return 第一个长期有效的电机实例；尚未注册时返回 NULL。
 * @note 仅在启动注册完成后供诊断调用，不能并发修改电机注册表。
 */
DJIMotorInstance *DJIMotorDiagnosticFirst(void);
#endif
#endif
