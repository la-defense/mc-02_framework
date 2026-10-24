#include <stdio.h>
#include <string.h>
#include "robot_safety.h"
#ifdef MC02_THREAD_TEST
#include <pthread.h>
#include <stdatomic.h>
#include <stdint.h>
#endif

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

#ifdef MC02_THREAD_TEST
static atomic_int safety_threads_done;
static atomic_int safety_threads_start;
static atomic_int safety_threads_failed;

static void WaitForThreadStart(void)
{
    while (atomic_load(&safety_threads_start) == 0)
        ;
}

static void *ToggleImuState(void *unused)
{
    (void)unused;
    WaitForThreadStart();
    for (uint32_t i = 0u; i < 100000u; ++i)
        RobotSafetySetImuValid((uint8_t)(i & 1u));
    atomic_fetch_add(&safety_threads_done, 1);
    return NULL;
}

static void *ToggleTaskHealth(void *unused)
{
    (void)unused;
    WaitForThreadStart();
    for (uint32_t i = 0u; i < 100000u; ++i)
        RobotSafetySetTaskHealthy((uint8_t)(i & 1u));
    atomic_fetch_add(&safety_threads_done, 1);
    return NULL;
}

static void *ToggleEstop(void *unused)
{
    (void)unused;
    WaitForThreadStart();
    for (uint32_t i = 0u; i < 100000u; ++i)
    {
        EstopRequest(ESTOP_REASON_MANUAL);
        (void)EstopClear();
    }
    atomic_fetch_add(&safety_threads_done, 1);
    return NULL;
}

static void *ReadSafetySnapshots(void *unused)
{
    (void)unused;
    atomic_store(&safety_threads_start, 1);
    do
    {
        RobotSafetySnapshot_t snapshot = {0};
        if (!RobotSafetyGetSnapshot(&snapshot) ||
            (snapshot.estop_latched == 0u && snapshot.estop_reason != ESTOP_REASON_NONE) ||
            (snapshot.estop_latched != 0u &&
             snapshot.estop_reason != ESTOP_REASON_MANUAL &&
             snapshot.estop_reason != ESTOP_REASON_RC_OFFLINE) ||
            (snapshot.calib_requested != 0u && snapshot.enable_requested != 0u))
            atomic_store(&safety_threads_failed, 1);
    } while (atomic_load(&safety_threads_done) != 3);
    return NULL;
}

static int ConcurrentSafetySnapshots(void)
{
    pthread_t imu_writer;
    pthread_t task_writer;
    pthread_t estop_writer;
    pthread_t reader;

    HealthyInputs();
    RobotSafetyRequestEnable();
    RobotSafetyUpdate();
    CHECK(pthread_create(&imu_writer, NULL, ToggleImuState, NULL) == 0);
    CHECK(pthread_create(&task_writer, NULL, ToggleTaskHealth, NULL) == 0);
    CHECK(pthread_create(&estop_writer, NULL, ToggleEstop, NULL) == 0);
    CHECK(pthread_create(&reader, NULL, ReadSafetySnapshots, NULL) == 0);
    CHECK(pthread_join(imu_writer, NULL) == 0);
    CHECK(pthread_join(task_writer, NULL) == 0);
    CHECK(pthread_join(estop_writer, NULL) == 0);
    CHECK(pthread_join(reader, NULL) == 0);
    CHECK(atomic_load(&safety_threads_failed) == 0);
    return 0;
}
#endif

int main(int argc, char **argv)
{
#ifdef MC02_THREAD_TEST
    if (argc > 1 && !strcmp(argv[1], "threads")) return ConcurrentSafetySnapshots();
#endif
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
