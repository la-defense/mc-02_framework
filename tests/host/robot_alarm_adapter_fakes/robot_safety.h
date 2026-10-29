/** @file robot_safety.h
 * @brief 告警适配层测试用 safety 快照接口。
 * @note 仅提供读取接口，测试可证明告警模块没有任何许可写入入口。
 */
#ifndef MC02_ROBOT_ALARM_TEST_SAFETY_H
#define MC02_ROBOT_ALARM_TEST_SAFETY_H

#include <stdint.h>

typedef enum
{
    ROBOT_STOP = 0,
    ROBOT_INIT,
    ROBOT_SAFE,
    ROBOT_CALIB,
    ROBOT_READY,
    ROBOT_FAULT,
    ROBOT_ESTOP,
} Robot_Status_e;

typedef struct
{
    Robot_Status_e state;
    uint32_t fault_flags;
    uint8_t estop_latched;
} RobotSafetySnapshot_t;

uint8_t RobotSafetyGetSnapshot(RobotSafetySnapshot_t *snapshot);

#endif
