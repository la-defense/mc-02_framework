/** @file mc02_build_config.h
 * @brief 模拟已生成的双板配置，仅供电机任务启动状态测试。
 */
#ifndef MC02_BUILD_CONFIG_H
#define MC02_BUILD_CONFIG_H
#define MC02_FEATURE_MOTOR_CONTROL 1 /* 本用例允许任务推进 CAN 发送。 */
#define MC02_SELECTED_DRIVER_DJI 1 /* 用替身记录大疆电机计算。 */
#define MC02_SELECTED_DRIVER_LK 1 /* 用替身记录 LK 电机计算。 */
#endif
