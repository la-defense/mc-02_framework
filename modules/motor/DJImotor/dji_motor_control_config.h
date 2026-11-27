/** @file dji_motor_control_config.h
 * @brief 检查 DJI 控制环使用的反馈和前馈声明，可在登记设备前调用。
 */
#ifndef MC02_DJI_MOTOR_CONTROL_CONFIG_H
#define MC02_DJI_MOTOR_CONTROL_CONFIG_H

#include "../motor_def.h"

/** @brief 检查旧方向标志、反馈选择及所需信号的回调、单位、参考系和期限。
 * @param[in] config 电机的只读配置。
 * @return 1 为控制来源配置有效，0 为参数为空或声明无效。
 * @note 只检查声明，不调用回调或访问设备。环的组合、PID 参数、单轮安装方向和 CAN 身份另行校验。
 */
uint8_t DJIMotorControlConfigIsValid(const Motor_Init_Config_s *config);

#endif
