/** @file robot_safety.c
 * @brief 汇总故障与操作请求并选择机器人状态，供控制任务读取同一时刻的许可信息。
 * @note 状态接口可由普通任务并发调用；日志在恢复中断后提交。
 */
#include "robot_safety.h"
#include "main.h"
#include "bsp_log.h"
#include <limits.h>

static Robot_Status_e robot_state = ROBOT_INIT;
static uint32_t fault_flags = ROBOT_FAULT_NONE;
static uint8_t rc_online = 0u;
static uint8_t imu_valid = 0u;
static uint8_t calib_valid = 0u;
static uint8_t task_healthy = 1u;
static uint8_t enable_requested = 0u;
static uint8_t calib_requested = 0u;
static uint8_t estop_latched = 0u;
static uint8_t remote_chassis_permit = 0u;
static uint32_t remote_chassis_source_time_ms = 0u;
static uint32_t remote_chassis_max_age_ms = 0u;
static uint32_t chassis_inhibit_generation = 0u;
static Estop_Reason_e estop_reason = ESTOP_REASON_NONE;
static uint8_t imu_fault_logged = 0u;
static uint8_t task_fault_logged = 0u;

static uint32_t RobotSafetyEnterCritical(void)
{
    const uint32_t previous = __get_PRIMASK();
    __disable_irq();
    return previous;
}

static void RobotSafetyExitCritical(uint32_t previous)
{
    __set_PRIMASK(previous);
}

#if defined(CHASSIS_BOARD)
static uint8_t RobotSafetyChassisLocalReadyLocked(void)
{
    return (uint8_t)(((fault_flags & (uint32_t)~ROBOT_FAULT_RC_OFFLINE) == ROBOT_FAULT_NONE &&
                      imu_valid != 0u && calib_valid != 0u && task_healthy != 0u &&
                      calib_requested == 0u && estop_latched == 0u) ? 1u : 0u);
}

static uint8_t RobotSafetyRemoteLeaseFreshLocked(uint32_t now_ms)
{
    return (uint8_t)(remote_chassis_permit != 0u && remote_chassis_max_age_ms != 0u &&
                     (uint32_t)(now_ms - remote_chassis_source_time_ms) <=
                         remote_chassis_max_age_ms);
}

static void RobotSafetyClearRemoteChassisPermitLocked(uint8_t advance_generation)
{
    if (remote_chassis_permit != 0u && advance_generation != 0u)
        chassis_inhibit_generation++;
    remote_chassis_permit = 0u;
    remote_chassis_source_time_ms = 0u;
    remote_chassis_max_age_ms = 0u;
}
#endif

static void RobotSafetyLogStateChange(Robot_Status_e old_state,
                                      Robot_Status_e new_state)
{
    if (old_state != new_state)
        BSP_LOG_EVENT_2(EV_MODULES_SAFETY_ROBOT_SAFETY_C_01,
            BSP_LOG_FIELD_CSTR_LIMIT((RobotSafetyStateName(old_state)), 46u),
            BSP_LOG_FIELD_CSTR_LIMIT((RobotSafetyStateName(new_state)), 46u));
}

static void RobotSafetyReportFaultLocked(Robot_Fault_e fault)
{
#if defined(CHASSIS_BOARD)
    const uint32_t inhibiting_faults = (uint32_t)fault & (uint32_t)~ROBOT_FAULT_RC_OFFLINE;
    if (inhibiting_faults != 0u)
    {
        if ((fault_flags & inhibiting_faults) != inhibiting_faults)
            chassis_inhibit_generation++;
        RobotSafetyClearRemoteChassisPermitLocked(0u);
    }
#endif
    fault_flags |= (uint32_t)fault;
}

static void RobotSafetyClearFaultLocked(Robot_Fault_e fault)
{
    const uint32_t permanent_faults = ROBOT_FAULT_INIT_FAILURE | ROBOT_FAULT_RESET_LOCKOUT;
    fault = (Robot_Fault_e)((uint32_t)fault & ~permanent_faults);
    fault_flags &= ~(uint32_t)fault;
}

static uint8_t RobotSafetyRequestEstopLocked(Estop_Reason_e reason)
{
    if (estop_latched && estop_reason == reason)
        return 0u;

    estop_latched = 1u;
    estop_reason = reason;
    enable_requested = 0u;
#if defined(CHASSIS_BOARD)
    RobotSafetyClearRemoteChassisPermitLocked(0u);
    chassis_inhibit_generation++;
#endif
    return 1u;
}

void RobotSafetyInit(void)
{
    Robot_Status_e old_state;
    const uint32_t previous = RobotSafetyEnterCritical();
    old_state = robot_state;
    robot_state = ROBOT_INIT;
    fault_flags = ROBOT_FAULT_NONE;
    rc_online = 0u;
    imu_valid = 0u;
    calib_valid = 0u;
    task_healthy = 1u;
    enable_requested = 0u;
    calib_requested = 0u;
    estop_latched = 0u;
    remote_chassis_permit = 0u;
    remote_chassis_source_time_ms = 0u;
    remote_chassis_max_age_ms = 0u;
    chassis_inhibit_generation = 0u;
    estop_reason = ESTOP_REASON_NONE;
    imu_fault_logged = 0u;
    task_fault_logged = 0u;
    robot_state = ROBOT_SAFE;
    RobotSafetyExitCritical(previous);

    RobotSafetyLogStateChange(old_state, ROBOT_SAFE);
}

void RobotSafetySetRcOnline(uint8_t online)
{
    const uint32_t previous = RobotSafetyEnterCritical();
    rc_online = online != 0u ? 1u : 0u;
    if (rc_online != 0u)
        RobotSafetyClearFaultLocked(ROBOT_FAULT_RC_OFFLINE);
    else
        RobotSafetyReportFaultLocked(ROBOT_FAULT_RC_OFFLINE);
    RobotSafetyExitCritical(previous);
}

void RobotSafetySetImuValid(uint8_t valid)
{
    uint8_t log_fault = 0u;
    const uint32_t previous = RobotSafetyEnterCritical();
    imu_valid = valid != 0u ? 1u : 0u;
    if (imu_valid != 0u)
    {
        RobotSafetyClearFaultLocked(ROBOT_FAULT_IMU_INVALID);
        imu_fault_logged = 0u;
    }
    else
    {
        RobotSafetyReportFaultLocked(ROBOT_FAULT_IMU_INVALID);
        if (imu_fault_logged == 0u)
        {
            imu_fault_logged = 1u;
            log_fault = 1u;
        }
    }
    RobotSafetyExitCritical(previous);

    if (log_fault != 0u)
        BSP_LOG_EVENT_0(EV_MODULES_SAFETY_ROBOT_SAFETY_C_02);
}

void RobotSafetySetCalibValid(uint8_t valid)
{
    const uint32_t previous = RobotSafetyEnterCritical();
    calib_valid = valid != 0u ? 1u : 0u;
    if (calib_valid != 0u)
        RobotSafetyClearFaultLocked(ROBOT_FAULT_CALIB_INVALID);
    else
        RobotSafetyReportFaultLocked(ROBOT_FAULT_CALIB_INVALID);
    RobotSafetyExitCritical(previous);
}

void RobotSafetySetTaskHealthy(uint8_t healthy)
{
    uint8_t log_fault = 0u;
    const uint32_t previous = RobotSafetyEnterCritical();
    task_healthy = healthy != 0u ? 1u : 0u;
    if (task_healthy != 0u)
    {
        RobotSafetyClearFaultLocked(ROBOT_FAULT_TASK_TIMEOUT);
        task_fault_logged = 0u;
    }
    else
    {
        RobotSafetyReportFaultLocked(ROBOT_FAULT_TASK_TIMEOUT);
        if (task_fault_logged == 0u)
        {
            task_fault_logged = 1u;
            log_fault = 1u;
        }
    }
    RobotSafetyExitCritical(previous);

    if (log_fault != 0u)
        BSP_LOG_EVENT_0(EV_MODULES_SAFETY_ROBOT_SAFETY_C_03);
}

void RobotSafetyRequestEnable(void)
{
    const uint32_t previous = RobotSafetyEnterCritical();
    if ((fault_flags & (ROBOT_FAULT_INIT_FAILURE | ROBOT_FAULT_RESET_LOCKOUT)) == 0u)
    {
        enable_requested = 1u;
        calib_requested = 0u;
    }
    RobotSafetyExitCritical(previous);
}

void RobotSafetyRequestDisable(void)
{
    const uint32_t previous = RobotSafetyEnterCritical();
    enable_requested = 0u;
    calib_requested = 0u;
#if defined(CHASSIS_BOARD)
    RobotSafetyClearRemoteChassisPermitLocked(0u);
    chassis_inhibit_generation++;
#endif
    RobotSafetyExitCritical(previous);
}

void RobotSafetyRequestCalib(uint8_t enable)
{
    const uint32_t previous = RobotSafetyEnterCritical();
    if (!enable || (fault_flags & (ROBOT_FAULT_INIT_FAILURE | ROBOT_FAULT_RESET_LOCKOUT)) == 0u)
    {
        calib_requested = enable != 0u ? 1u : 0u;
        if (calib_requested != 0u)
        {
            enable_requested = 0u;
#if defined(CHASSIS_BOARD)
            RobotSafetyClearRemoteChassisPermitLocked(0u);
            chassis_inhibit_generation++;
#endif
        }
    }
    RobotSafetyExitCritical(previous);
}

uint8_t RobotSafetyTryGrantRemoteChassisPermit(uint32_t expected_generation,
                                                uint32_t source_time_ms,
                                                uint32_t max_age_ms,
                                                uint32_t now_ms)
{
    const uint32_t previous = RobotSafetyEnterCritical();
#if defined(CHASSIS_BOARD)
    const uint8_t lease_valid = (uint8_t)(max_age_ms != 0u && max_age_ms <= INT32_MAX &&
        (uint32_t)(now_ms - source_time_ms) <= max_age_ms);
    const uint8_t granted = (uint8_t)(expected_generation == chassis_inhibit_generation &&
        RobotSafetyChassisLocalReadyLocked() != 0u && lease_valid != 0u);
    if (granted != 0u)
    {
        remote_chassis_permit = 1u;
        remote_chassis_source_time_ms = source_time_ms;
        remote_chassis_max_age_ms = max_age_ms;
    }
    else
        RobotSafetyClearRemoteChassisPermitLocked(1u);
#else
    (void)expected_generation;
    (void)source_time_ms;
    (void)max_age_ms;
    (void)now_ms;
    const uint8_t granted = 0u;
#endif
    RobotSafetyExitCritical(previous);
    return granted;
}

void RobotSafetyRevokeRemoteChassisPermit(void)
{
    const uint32_t previous = RobotSafetyEnterCritical();
#if defined(CHASSIS_BOARD)
    RobotSafetyClearRemoteChassisPermitLocked(1u);
#endif
    RobotSafetyExitCritical(previous);
}

uint8_t RobotSafetyIsChassisLocalReady(void)
{
#if defined(CHASSIS_BOARD)
    const uint32_t previous = RobotSafetyEnterCritical();
    const uint8_t ready = RobotSafetyChassisLocalReadyLocked();
    RobotSafetyExitCritical(previous);
    return ready;
#else
    return 0u;
#endif
}

void RobotFaultReport(Robot_Fault_e fault)
{
    const uint32_t previous = RobotSafetyEnterCritical();
    RobotSafetyReportFaultLocked(fault);
    RobotSafetyExitCritical(previous);
}

void RobotFaultClear(Robot_Fault_e fault)
{
    const uint32_t previous = RobotSafetyEnterCritical();
    RobotSafetyClearFaultLocked(fault);
    RobotSafetyExitCritical(previous);
}

void EstopRequest(Estop_Reason_e reason)
{
    const uint32_t previous = RobotSafetyEnterCritical();
    const uint8_t log_request = RobotSafetyRequestEstopLocked(reason);
    RobotSafetyExitCritical(previous);

    if (log_request != 0u)
        BSP_LOG_EVENT_1(EV_MODULES_SAFETY_ROBOT_SAFETY_C_04,
                        BSP_LOG_FIELD_I32((int)reason));
}

uint8_t EstopClear(void)
{
    Robot_Status_e old_state;
    const uint32_t previous = RobotSafetyEnterCritical();
    if ((fault_flags & (ROBOT_FAULT_RC_OFFLINE | ROBOT_FAULT_TASK_TIMEOUT |
                        ROBOT_FAULT_INIT_FAILURE | ROBOT_FAULT_RESET_LOCKOUT)) != 0u)
    {
        RobotSafetyExitCritical(previous);
        return 0u;
    }

    if (estop_latched == 0u)
    {
        RobotSafetyExitCritical(previous);
        return 1u;
    }

    estop_latched = 0u;
    estop_reason = ESTOP_REASON_NONE;
    enable_requested = 0u;
    old_state = robot_state;
    robot_state = ROBOT_SAFE;
    RobotSafetyExitCritical(previous);

    BSP_LOG_EVENT_0(EV_MODULES_SAFETY_ROBOT_SAFETY_C_05);
    RobotSafetyLogStateChange(old_state, ROBOT_SAFE);
    return 1u;
}

uint8_t EstopIsLatched(void)
{
    const uint32_t previous = RobotSafetyEnterCritical();
    const uint8_t latched = estop_latched;
    RobotSafetyExitCritical(previous);
    return latched;
}

Estop_Reason_e EstopGetReason(void)
{
    const uint32_t previous = RobotSafetyEnterCritical();
    const Estop_Reason_e reason = estop_reason;
    RobotSafetyExitCritical(previous);
    return reason;
}

Robot_Status_e RobotSafetyGetState(void)
{
    const uint32_t previous = RobotSafetyEnterCritical();
    const Robot_Status_e state = robot_state;
    RobotSafetyExitCritical(previous);
    return state;
}

uint8_t RobotSafetyIsReady(void)
{
#if defined(CHASSIS_BOARD)
    return RobotSafetyIsReadyAt(HAL_GetTick());
#else
    return RobotSafetyIsReadyAt(0u);
#endif
}

uint8_t RobotSafetyIsReadyAt(uint32_t now_ms)
{
    const uint32_t previous = RobotSafetyEnterCritical();
#if defined(CHASSIS_BOARD)
    const uint8_t ready = (robot_state == ROBOT_READY &&
                           RobotSafetyChassisLocalReadyLocked() != 0u &&
                           RobotSafetyRemoteLeaseFreshLocked(now_ms) != 0u) ? 1u : 0u;
#else
    (void)now_ms;
    const uint8_t ready = (robot_state == ROBOT_READY && fault_flags == ROBOT_FAULT_NONE &&
                           rc_online != 0u && imu_valid != 0u && calib_valid != 0u &&
                           task_healthy != 0u && enable_requested != 0u &&
                           calib_requested == 0u && estop_latched == 0u) ? 1u : 0u;
#endif
    RobotSafetyExitCritical(previous);
    return ready;
}

uint32_t RobotSafetyGetFaults(void)
{
    const uint32_t previous = RobotSafetyEnterCritical();
    const uint32_t faults = fault_flags;
    RobotSafetyExitCritical(previous);
    return faults;
}

uint8_t RobotSafetyGetSnapshot(RobotSafetySnapshot_t *snapshot)
{
    if (snapshot == NULL)
        return 0u;

    RobotSafetySnapshot_t candidate;
    const uint32_t previous = RobotSafetyEnterCritical();
    candidate.state = robot_state;
    candidate.fault_flags = fault_flags;
    candidate.rc_online = rc_online;
    candidate.imu_valid = imu_valid;
    candidate.calib_valid = calib_valid;
    candidate.task_healthy = task_healthy;
    candidate.enable_requested = enable_requested;
    candidate.calib_requested = calib_requested;
    candidate.estop_latched = estop_latched;
    candidate.remote_chassis_permit = remote_chassis_permit;
    candidate.chassis_inhibit_generation = chassis_inhibit_generation;
    candidate.estop_reason = estop_reason;
    RobotSafetyExitCritical(previous);

    *snapshot = candidate;
    return 1u;
}

const char *RobotSafetyStateName(Robot_Status_e state)
{
    switch (state)
    {
    case ROBOT_INIT: return "INIT";
    case ROBOT_SAFE: return "SAFE";
    case ROBOT_CALIB: return "CALIB";
    case ROBOT_READY: return "READY";
    case ROBOT_FAULT: return "FAULT";
    case ROBOT_ESTOP: return "ESTOP";
    case ROBOT_STOP:
    default: return "STOP";
    }
}

void RobotSafetyUpdate(void)
{
    Robot_Status_e old_state;
    Robot_Status_e new_state;
    Estop_Reason_e requested_estop_reason = ESTOP_REASON_NONE;
    const uint32_t previous = RobotSafetyEnterCritical();
    const uint32_t active_faults = fault_flags;

#if defined(CHASSIS_BOARD)
    // 任务周期之外也以租约期限判定；到期立刻撤销旧远端命令并要求新序号重接管。
    if (remote_chassis_permit != 0u && RobotSafetyRemoteLeaseFreshLocked(HAL_GetTick()) == 0u)
        RobotSafetyClearRemoteChassisPermitLocked(1u);
    // 底盘板不接收遥控器；运动授权来自云台板的新鲜命令并继续受本地状态约束。
    if ((active_faults & (ROBOT_FAULT_INIT_FAILURE | ROBOT_FAULT_RESET_LOCKOUT)) != 0u)
    {
        calib_requested = 0u;
        new_state = ROBOT_FAULT;
    }
    else if (estop_latched != 0u)
    {
        new_state = ROBOT_ESTOP;
    }
    else if (task_healthy == 0u ||
             (active_faults & (uint32_t)~ROBOT_FAULT_RC_OFFLINE) != 0u)
    {
        new_state = ROBOT_FAULT;
    }
    else if (calib_requested != 0u)
    {
        new_state = ROBOT_CALIB;
    }
    else if (imu_valid == 0u || calib_valid == 0u || remote_chassis_permit == 0u)
    {
        new_state = ROBOT_SAFE;
    }
    else
    {
        new_state = ROBOT_READY;
    }
#else
    if ((active_faults & (ROBOT_FAULT_INIT_FAILURE | ROBOT_FAULT_RESET_LOCKOUT)) != 0u)
    {
        enable_requested = 0u;
        calib_requested = 0u;
        new_state = ROBOT_FAULT;
    }
    else
    {
        if (rc_online == 0u || (active_faults & ROBOT_FAULT_RC_OFFLINE) != 0u)
        {
            if (RobotSafetyRequestEstopLocked(ESTOP_REASON_RC_OFFLINE) != 0u)
                requested_estop_reason = ESTOP_REASON_RC_OFFLINE;
        }

        if (estop_latched != 0u)
        {
            enable_requested = 0u;
            new_state = ROBOT_ESTOP;
        }
        else if (task_healthy == 0u ||
                 (active_faults & ROBOT_FAULT_TASK_TIMEOUT) != 0u)
        {
            enable_requested = 0u;
            new_state = ROBOT_FAULT;
        }
        else if (calib_requested != 0u)
        {
            new_state = ROBOT_CALIB;
        }
        else if ((active_faults & (ROBOT_FAULT_MOTOR_OFFLINE |
                                   ROBOT_FAULT_CAN_BUSOFF |
                                   ROBOT_FAULT_TILT)) != 0u)
        {
            new_state = ROBOT_FAULT;
        }
        else if (enable_requested != 0u)
        {
            if (active_faults != ROBOT_FAULT_NONE || rc_online == 0u ||
                imu_valid == 0u || calib_valid == 0u)
                new_state = ROBOT_SAFE;
            else
                new_state = ROBOT_READY;
        }
        else
        {
            new_state = ROBOT_SAFE;
        }
    }
#endif

    old_state = robot_state;
    robot_state = new_state;
    RobotSafetyExitCritical(previous);

    if (requested_estop_reason != ESTOP_REASON_NONE)
        BSP_LOG_EVENT_1(EV_MODULES_SAFETY_ROBOT_SAFETY_C_04,
                        BSP_LOG_FIELD_I32((int)requested_estop_reason));
    RobotSafetyLogStateChange(old_state, new_state);
}
