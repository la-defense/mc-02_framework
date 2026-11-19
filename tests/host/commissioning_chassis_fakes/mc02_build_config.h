/** @file mc02_build_config.h
 * @brief 给受限底盘主机联测提供与单板固件相同的机构选择。
 */
#pragma once

#define MC02_FEATURE_CHASSIS 1 // 车型预检读取轮组参数。
#define MC02_FEATURE_GIMBAL 0 // 受限底盘不读取云台参数。
#define MC02_FEATURE_SHOOTER 0 // 受限底盘不读取发射参数。
#define MC02_CHASSIS_IS_OMNI 1 // 哨兵使用四轮全向轮模型。
