#ifndef ROBOT_SAFETY_H
#define ROBOT_SAFETY_H

#include <stdint.h>
#include "robot_def.h"

/* 机器人故障位, 可同时置位多个 */
typedef enum
{
    ROBOT_FAULT_NONE = 0,
    ROBOT_FAULT_RC_OFFLINE = 1u << 0,
    ROBOT_FAULT_IMU_INVALID = 1u << 1,
    ROBOT_FAULT_MOTOR_OFFLINE = 1u << 2,
    ROBOT_FAULT_CAN_BUSOFF = 1u << 3,
    ROBOT_FAULT_TILT = 1u << 4,
    ROBOT_FAULT_CALIB_INVALID = 1u << 5,
    ROBOT_FAULT_TASK_TIMEOUT = 1u << 6,
} Robot_Fault_e;

/* 急停原因 */
typedef enum
{
    ESTOP_REASON_NONE = 0,
    ESTOP_REASON_RC_OFFLINE,
    ESTOP_REASON_MANUAL,
} Estop_Reason_e;

void RobotSafetyInit(void);
void RobotSafetyUpdate(void);

void RobotSafetySetRcOnline(uint8_t online);
void RobotSafetySetImuValid(uint8_t valid);
void RobotSafetySetCalibValid(uint8_t valid);
void RobotSafetySetTaskHealthy(uint8_t healthy);

void RobotSafetyRequestEnable(void);
void RobotSafetyRequestDisable(void);
void RobotSafetyRequestCalib(uint8_t enable);

void RobotFaultReport(Robot_Fault_e fault);
void RobotFaultClear(Robot_Fault_e fault);

void EstopRequest(Estop_Reason_e reason);
uint8_t EstopClear(void);
uint8_t EstopIsLatched(void);
Estop_Reason_e EstopGetReason(void);

Robot_Status_e RobotSafetyGetState(void);
uint8_t RobotSafetyIsReady(void);
uint32_t RobotSafetyGetFaults(void);
const char *RobotSafetyStateName(Robot_Status_e state);

#endif // !ROBOT_SAFETY_H
