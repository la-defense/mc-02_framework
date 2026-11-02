/** @file robot_state.h
 * @brief 整车运行状态定义；数值沿用现有消息和诊断接口。
 */
#ifndef MC02_ROBOT_STATE_H
#define MC02_ROBOT_STATE_H

/** @brief 整车状态；旧停止和正常运行值保留给兼容接口。 */
typedef enum
{
    ROBOT_STOP = 0, // 旧接口的停止输出状态。
    ROBOT_READY,    // 旧接口的正常运行状态。
    ROBOT_INIT,     // 正在初始化。
    ROBOT_SAFE,     // 已初始化但未使能。
    ROBOT_CALIB,    // 正在标定。
    ROBOT_FAULT,    // 发生故障，需要限制功能。
    ROBOT_ESTOP,    // 急停已锁存。
} Robot_Status_e;
#endif
