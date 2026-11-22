/** @file chassis_motion_diag.h
 * @brief 在受限底盘固件中登记按需的底盘运动诊断。
 */
#ifndef MC02_CHASSIS_MOTION_DIAG_H
#define MC02_CHASSIS_MOTION_DIAG_H

#include "bsp_log.h"

/** @brief 将轮速估计和可选里程计接入现有 USART10 查询。
 * @return 日志诊断登记结果。
 * @note 日志资源初始化后、调度器启动前调用。查询只读取反馈。
 */
BSPLogStatus_t ChassisMotionDiagnosticInit(void);

#endif
