/** @file robot_can_requirements.c
 * @brief 按已编译的业务设备汇总经典 CAN 接收端点。
 */
#include "config/build_selection.h"
#include "robot_can_requirements.h"

/** @brief 汇总当前设备的 CAN 端点需求，供容量预检及诊断复用。 */
uint8_t RobotCollectCanDemands(const Robot_Config_s *config, RobotCanFilterDemand_t demands[8])
{
    uint8_t count = 0u; // 固定容量足以覆盖当前设备组合。
    (void)config; // 无 CAN 设备的车型不读取这些字段。
    (void)demands; // 无 CAN 设备时不会写入数组。
#if defined(MC02_PROFILE_ROBOT) && (defined(CHASSIS_BOARD) || defined(GIMBAL_BOARD))
    demands[count++] = (RobotCanFilterDemand_t){ROBOT_MODULE_COMMUNICATION, config->can_bridge.can_bus, 1u, 0u}; // 双板桥接。
#endif
#if MC02_FEATURE_CHASSIS
    demands[count++] = (RobotCanFilterDemand_t){ROBOT_MODULE_CHASSIS, config->chassis_param.chassis_motor_config.can_init_config.can_bus, 4u, 0u}; // 四个底盘电机。
#if MC02_FEATURE_SUPERCAP
    demands[count++] = (RobotCanFilterDemand_t){ROBOT_MODULE_CHASSIS, PLATFORM_CAN_BUS_2, 1u, 0u}; // 旧电容由底盘注册。
#endif
#endif
#if MC02_FEATURE_GIMBAL
    demands[count++] = (RobotCanFilterDemand_t){ROBOT_MODULE_GIMBAL, config->gimbal_param.yaw_motor_config.can_init_config.can_bus, 1u, 0u}; // 大 yaw 电机。
    demands[count++] = (RobotCanFilterDemand_t){ROBOT_MODULE_GIMBAL, config->gimbal_param.pitch_motor_config.can_init_config.can_bus, 1u, 0u}; // pitch 电机。
#endif
#if MC02_FEATURE_SHOOTER
    demands[count++] = (RobotCanFilterDemand_t){ROBOT_MODULE_SHOOTER, config->shoot_param.friction_motor_config.can_init_config.can_bus, ShootFrictionWheelCount(config->shoot_param.shoot_type), 0u}; // 每个摩擦轮占一个端点。
    demands[count++] = (RobotCanFilterDemand_t){ROBOT_MODULE_SHOOTER, config->shoot_param.loader_motor_config.can_init_config.can_bus, 1u, 0u}; // 拨弹电机。
#endif
    return count; // 返回实际需求项数。
}
