/**
 * @file robot_def.c
 * @brief 机器人配置工厂入口, 根据 id 返回对应机器人配置, 并提供三个 Feed 函数
 *        供 chassis/gimbal/shoot 在 Init 时取配置指针.
 */
#include "robot_def.h"
#include "config.h"
#include "sentinel_config.h"

static Robot_Config_s *robot_instance = NULL;

Robot_Config_s *RobotConfigInit(uint8_t id)
{
    Robot_Config_s *instance = NULL;

    switch (id)
    {
    case SENTINEL_ROBOT:
        instance = SentinelConfigInit();
        break;
    /* 其它机器人配置(hero/infantry)暂未移植, 需要时在此添加 case */
    case HERO_ROBOT:
    case ENGINEER_ROBOT:
    case INFANTRY_ROBOT_3:
    case INFANTRY_ROBOT_4:
    case INFANTRY_ROBOT_5:
    case AERIAL_ROBOT:
    case DART_ROBOT:
    default:
        instance = SentinelConfigInit(); // 默认哨兵
        break;
    }

    robot_instance = instance;
    return instance;
}

Chassis_Config_s *ChassisConfigFeed(void)
{
    return &robot_instance->chassis_param;
}

Gimbal_Config_s *GimbalConfigFeed(void)
{
    return &robot_instance->gimbal_param;
}

Shoot_Config_s *ShootConfigFeed(void)
{
    return &robot_instance->shoot_param;
}
