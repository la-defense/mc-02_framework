#include <stdio.h>
#include <string.h>
#include "robot_safety.h"

#define CHECK(c) do { if (!(c)) { fprintf(stderr, "FAIL:%d %s\n", __LINE__, #c); return 1; } } while (0)

static void HealthyInputs(void)
{
    RobotSafetyInit();
    RobotSafetySetRcOnline(1);
    RobotSafetySetImuValid(1);
    RobotSafetySetCalibValid(1);
    RobotSafetySetTaskHealthy(1);
}

static int RecoveryAndPriority(void)
{
    const Robot_Fault_e faults[] = {ROBOT_FAULT_MOTOR_OFFLINE, ROBOT_FAULT_CAN_BUSOFF, ROBOT_FAULT_TILT};
    const Robot_Fault_e combined = ROBOT_FAULT_MOTOR_OFFLINE | ROBOT_FAULT_CAN_BUSOFF | ROBOT_FAULT_TILT;
    HealthyInputs(); RobotSafetyRequestEnable(); RobotFaultReport(combined); RobotSafetyUpdate();
    CHECK(RobotSafetyGetState() == ROBOT_FAULT);
    RobotFaultClear(ROBOT_FAULT_IMU_INVALID); RobotSafetyUpdate();
    CHECK(RobotSafetyGetFaults() == (uint32_t)combined && !RobotSafetyIsReady());
    for (unsigned i = 0; i < 3; ++i) {
        RobotFaultClear(faults[i]); RobotSafetyUpdate();
        CHECK(RobotSafetyIsReady() == (i == 2));
    }
    /* Clearing a bit cannot override a still-invalid input. */
    HealthyInputs(); RobotSafetyRequestEnable(); RobotSafetySetImuValid(0);
    RobotFaultClear(ROBOT_FAULT_IMU_INVALID); RobotSafetyUpdate(); CHECK(!RobotSafetyIsReady());
    RobotSafetySetImuValid(1); RobotSafetyUpdate(); CHECK(RobotSafetyIsReady());
    for (unsigned i = 0; i < 3; ++i) {
        for (unsigned interruption = 0; interruption < 5; ++interruption) {
            HealthyInputs(); RobotSafetyRequestEnable(); RobotFaultReport(faults[i]); RobotSafetyUpdate();
            CHECK(RobotSafetyGetState() == ROBOT_FAULT);
            if (interruption == 0) RobotSafetyRequestDisable();
            if (interruption == 1) RobotSafetyRequestCalib(1);
            if (interruption == 2) RobotSafetySetTaskHealthy(0);
            if (interruption == 3) EstopRequest(ESTOP_REASON_MANUAL);
            if (interruption == 4) RobotSafetySetRcOnline(0);
            RobotSafetyUpdate();
            if (interruption == 1) {
                CHECK(RobotSafetyGetState() == ROBOT_CALIB);
                CHECK(RobotSafetyGetFaults() == (uint32_t)faults[i]);
                RobotFaultReport(ROBOT_FAULT_TASK_TIMEOUT); RobotSafetyUpdate();
                CHECK(RobotSafetyGetState() == ROBOT_FAULT);
                EstopRequest(ESTOP_REASON_MANUAL); RobotSafetyUpdate();
                CHECK(RobotSafetyGetState() == ROBOT_ESTOP);
                RobotFaultClear(ROBOT_FAULT_TASK_TIMEOUT); CHECK(EstopClear());
                RobotSafetyUpdate(); CHECK(RobotSafetyGetState() == ROBOT_CALIB);
            }
            RobotFaultClear(faults[i]); RobotSafetySetTaskHealthy(1); RobotSafetySetRcOnline(1);
            RobotSafetyRequestCalib(0); CHECK(EstopClear()); RobotSafetyUpdate();
            CHECK(RobotSafetyGetState() == ROBOT_SAFE);
            RobotSafetyRequestEnable(); RobotSafetyUpdate(); CHECK(RobotSafetyIsReady());
        }
    }
    return 0;
}

static int InitializationFailureIsSticky(void)
{
    HealthyInputs();
    RobotFaultReport(ROBOT_FAULT_INIT_FAILURE);
    RobotSafetyRequestEnable();
    RobotSafetyRequestCalib(1u);
    RobotSafetyUpdate();
    CHECK(RobotSafetyGetState() == ROBOT_FAULT);
    CHECK(!RobotSafetyIsReady());
    RobotFaultClear(ROBOT_FAULT_INIT_FAILURE);
    CHECK(!EstopClear());
    RobotSafetyRequestEnable();
    RobotSafetyRequestCalib(1u);
    RobotSafetyUpdate();
    CHECK(RobotSafetyGetState() == ROBOT_FAULT);
    CHECK((RobotSafetyGetFaults() & ROBOT_FAULT_INIT_FAILURE) != 0u);
    return 0;
}

int main(int argc, char **argv)
{
    if (argc > 1 && !strcmp(argv[1], "recovery")) return RecoveryAndPriority();
    if (argc > 1 && !strcmp(argv[1], "init-failure")) return InitializationFailureIsSticky();
    const struct {
        Robot_Fault_e fault;
        Robot_Status_e state;
        uint8_t automatic;
    } cases[] = {
        {ROBOT_FAULT_MOTOR_OFFLINE, ROBOT_FAULT, 1},
        {ROBOT_FAULT_CAN_BUSOFF, ROBOT_FAULT, 1},
        {ROBOT_FAULT_TILT, ROBOT_FAULT, 1},
        {ROBOT_FAULT_RC_OFFLINE, ROBOT_ESTOP, 0},
        {ROBOT_FAULT_IMU_INVALID, ROBOT_SAFE, 1},
        {ROBOT_FAULT_CALIB_INVALID, ROBOT_SAFE, 1},
        {ROBOT_FAULT_TASK_TIMEOUT, ROBOT_FAULT, 0},
    };
    for (unsigned i = 0; i < sizeof(cases) / sizeof(cases[0]); ++i) {
        for (unsigned already_ready = 0; already_ready < 2; ++already_ready) {
            HealthyInputs();
            if (already_ready) {
                RobotSafetyRequestEnable(); RobotSafetyUpdate();
                CHECK(RobotSafetyIsReady());
            }
            RobotFaultReport(cases[i].fault);
            if (!already_ready) RobotSafetyRequestEnable();
            RobotSafetyUpdate();
            if (RobotSafetyIsReady())
                fprintf(stderr, "reported fault=%u, already_ready=%u\n", (unsigned)cases[i].fault, already_ready);
            CHECK(!RobotSafetyIsReady());
            CHECK(RobotSafetyGetState() == cases[i].state);
            CHECK(RobotSafetyGetFaults() == (uint32_t)cases[i].fault);
            RobotFaultClear(cases[i].fault);
            if (cases[i].fault == ROBOT_FAULT_RC_OFFLINE) {
                CHECK(EstopIsLatched()); CHECK(EstopClear());
            }
            RobotSafetyUpdate();
            CHECK(RobotSafetyIsReady() == cases[i].automatic);
            if (!cases[i].automatic) {
                CHECK(RobotSafetyGetState() == ROBOT_SAFE);
                RobotSafetyRequestEnable(); RobotSafetyUpdate();
                CHECK(RobotSafetyIsReady());
            }
        }
    }
    return 0;
}
