#include "robot_safety.h"
#include "bsp_log.h"

static Robot_Status_e robot_state = ROBOT_INIT;
static uint32_t fault_flags = ROBOT_FAULT_NONE;
static uint8_t rc_online = 0;
static uint8_t imu_valid = 0;
static uint8_t calib_valid = 0;
static uint8_t task_healthy = 1;
static uint8_t enable_requested = 0;
static uint8_t calib_requested = 0;

static uint8_t estop_latched = 0;
static Estop_Reason_e estop_reason = ESTOP_REASON_NONE;

static uint8_t imu_fault_logged = 0;
static uint8_t task_fault_logged = 0;

static void RobotSafetySetState(Robot_Status_e new_state)
{
    if (robot_state == new_state)
        return;

    LOGINFO("[safety] state %s -> %s",
            RobotSafetyStateName(robot_state),
            RobotSafetyStateName(new_state));
    robot_state = new_state;
}

void RobotSafetyInit(void)
{
    robot_state = ROBOT_INIT;
    fault_flags = ROBOT_FAULT_NONE;
    rc_online = 0;
    imu_valid = 0;
    calib_valid = 0;
    task_healthy = 1;
    enable_requested = 0;
    calib_requested = 0;
    estop_latched = 0;
    estop_reason = ESTOP_REASON_NONE;
    imu_fault_logged = 0;
    task_fault_logged = 0;

    RobotSafetySetState(ROBOT_SAFE);
}

void RobotSafetySetRcOnline(uint8_t online)
{
    rc_online = online ? 1 : 0;
    if (rc_online)
        RobotFaultClear(ROBOT_FAULT_RC_OFFLINE);
    else
        RobotFaultReport(ROBOT_FAULT_RC_OFFLINE);
}

void RobotSafetySetImuValid(uint8_t valid)
{
    imu_valid = valid ? 1 : 0;
    if (imu_valid)
    {
        RobotFaultClear(ROBOT_FAULT_IMU_INVALID);
        imu_fault_logged = 0;
    }
    else
    {
        RobotFaultReport(ROBOT_FAULT_IMU_INVALID);
        if (!imu_fault_logged)
        {
            LOGERROR("[safety] IMU invalid, actuators will be degraded");
            imu_fault_logged = 1;
        }
    }
}

void RobotSafetySetCalibValid(uint8_t valid)
{
    calib_valid = valid ? 1 : 0;
    if (calib_valid)
        RobotFaultClear(ROBOT_FAULT_CALIB_INVALID);
    else
        RobotFaultReport(ROBOT_FAULT_CALIB_INVALID);
}

void RobotSafetySetTaskHealthy(uint8_t healthy)
{
    task_healthy = healthy ? 1 : 0;
    if (task_healthy)
    {
        RobotFaultClear(ROBOT_FAULT_TASK_TIMEOUT);
        task_fault_logged = 0;
    }
    else
    {
        RobotFaultReport(ROBOT_FAULT_TASK_TIMEOUT);
        if (!task_fault_logged)
        {
            LOGERROR("[safety] task monitor timeout, stop feeding IWDG");
            task_fault_logged = 1;
        }
    }
}

void RobotSafetyRequestEnable(void)
{
    enable_requested = 1;
    calib_requested = 0;
}

void RobotSafetyRequestDisable(void)
{
    enable_requested = 0;
    calib_requested = 0;
}

void RobotSafetyRequestCalib(uint8_t enable)
{
    calib_requested = enable ? 1 : 0;
    if (calib_requested)
        enable_requested = 0;
}

void RobotFaultReport(Robot_Fault_e fault)
{
    fault_flags |= (uint32_t)fault;
}

void RobotFaultClear(Robot_Fault_e fault)
{
    fault_flags &= ~(uint32_t)fault;
}

void EstopRequest(Estop_Reason_e reason)
{
    if (estop_latched && estop_reason == reason)
        return;

    estop_latched = 1;
    estop_reason = reason;
    enable_requested = 0;
    LOGERROR("[safety] ESTOP latched, reason=%d", (int)reason);
}

uint8_t EstopClear(void)
{
    /* 故障未清除时不允许复位; 遥控离线必须等遥控重新上线 */
    if (fault_flags & (ROBOT_FAULT_RC_OFFLINE | ROBOT_FAULT_TASK_TIMEOUT))
        return 0;

    if (!estop_latched)
        return 1;

    estop_latched = 0;
    estop_reason = ESTOP_REASON_NONE;
    enable_requested = 0;
    LOGINFO("[safety] ESTOP cleared, robot in SAFE state");
    RobotSafetySetState(ROBOT_SAFE);
    return 1;
}

uint8_t EstopIsLatched(void)
{
    return estop_latched;
}

Estop_Reason_e EstopGetReason(void)
{
    return estop_reason;
}

Robot_Status_e RobotSafetyGetState(void)
{
    return robot_state;
}

uint8_t RobotSafetyIsReady(void)
{
    return (robot_state == ROBOT_READY) ? 1 : 0;
}

uint32_t RobotSafetyGetFaults(void)
{
    return fault_flags;
}

const char *RobotSafetyStateName(Robot_Status_e state)
{
    switch (state)
    {
    case ROBOT_INIT:
        return "INIT";
    case ROBOT_SAFE:
        return "SAFE";
    case ROBOT_CALIB:
        return "CALIB";
    case ROBOT_READY:
        return "READY";
    case ROBOT_FAULT:
        return "FAULT";
    case ROBOT_ESTOP:
        return "ESTOP";
    case ROBOT_STOP:
    default:
        return "STOP";
    }
}

void RobotSafetyUpdate(void)
{
    /* 遥控器离线是唯一会锁存急停的故障 */
    if (!rc_online)
        EstopRequest(ESTOP_REASON_RC_OFFLINE);

    if (estop_latched)
    {
        enable_requested = 0;
        RobotSafetySetState(ROBOT_ESTOP);
        return;
    }

    if (!task_healthy)
    {
        enable_requested = 0;
        RobotSafetySetState(ROBOT_FAULT);
        return;
    }

    if (calib_requested)
    {
        RobotSafetySetState(ROBOT_CALIB);
        return;
    }

    if (enable_requested)
    {
        if (!rc_online)
        {
            RobotSafetySetState(ROBOT_SAFE);
            return;
        }
        if (!imu_valid)
        {
            RobotSafetySetState(ROBOT_SAFE);
            return;
        }
        if (!calib_valid)
        {
            RobotSafetySetState(ROBOT_SAFE);
            return;
        }

        RobotSafetySetState(ROBOT_READY);
        return;
    }

    RobotSafetySetState(ROBOT_SAFE);
}
