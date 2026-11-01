/** @file motor_config_public_header.c
 * @brief 只用框架头编译电机配置，验证公开类型不要求芯片或 RTOS 头。
 */
#include <stddef.h>
#include "motor_def.h"

/** @brief 读取电机配置的逻辑 CAN 编号。
 * @param[in] config 可为空的电机配置。
 * @return 非空时返回逻辑总线，否则返回无效编号。
 */
PlatformCanBus_t MotorConfigPublicHeaderProbe(const Motor_Init_Config_s *config)
{
    return config == NULL ? PLATFORM_CAN_INVALID : config->can_init_config.can_bus;
}
