#include "robot_safety.h"
#include <stdio.h>

#define CHECK(condition) do { if (!(condition)) { \
    fprintf(stderr, "FAIL:%d %s\n", __LINE__, #condition); return 1; } } while (0)

int main(void)
{
    RobotSafetyInit();
    RobotSafetySetImuValid(1u);
    RobotSafetySetCalibValid(1u);
    RobotSafetySetTaskHealthy(1u);
    RobotSafetySetRemoteChassisPermit(1u);
    RobotSafetyUpdate();
    CHECK(RobotSafetyGetState() == ROBOT_READY);
    CHECK(RobotSafetyIsReady());

    RobotSafetySetRemoteChassisPermit(0u);
    CHECK(!RobotSafetyIsReady());
    RobotSafetyUpdate();
    CHECK(RobotSafetyGetState() == ROBOT_SAFE);

    RobotSafetySetRemoteChassisPermit(1u);
    RobotSafetyUpdate();
    CHECK(RobotSafetyIsReady());
    RobotFaultReport(ROBOT_FAULT_CAN_BUSOFF);
    CHECK(!RobotSafetyIsReady());
    RobotSafetyUpdate();
    CHECK(RobotSafetyGetState() == ROBOT_FAULT);

    RobotFaultClear(ROBOT_FAULT_CAN_BUSOFF);
    RobotSafetyUpdate();
    CHECK(RobotSafetyIsReady());
    EstopRequest(ESTOP_REASON_MANUAL);
    CHECK(!RobotSafetyIsReady());
    RobotSafetyUpdate();
    CHECK(RobotSafetyGetState() == ROBOT_ESTOP);
    CHECK(EstopClear());
    CHECK(!RobotSafetyIsReady());

    RobotSafetySetRemoteChassisPermit(0u);
    RobotSafetyUpdate();
    RobotSafetySetImuValid(0u);
    RobotSafetySetRemoteChassisPermit(1u);
    RobotSafetyUpdate();
    CHECK(!RobotSafetyIsReady());
    return 0;
}
