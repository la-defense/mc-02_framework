/**
 * @file vehicle_health_port_stm32.c
 * @brief 从现有安全、IMU、通信、CAN、USB、加热和日志只读接口采集健康数据。
 * @note 仅由RobotTask普通任务调用；各模块保留自己的来源时间和一致读取接口。
 */
#include "vehicle_health_source.h"
#include "config/build_selection.h"
#if defined(MC02_CONFIG_GENERATED)
#include "config/vehicle_selection.h"
#include "config/module_plan.h"
#endif
#include "main.h"
#include "robot_def.h"
#include "init_diagnostic.h"
#include "robot_safety.h"
#include "task_monitor.h"
#include "ins_task.h"
#include "imu_heater.h"
#if MC02_FEATURE_REMOTE
#include "remote_control.h"
#endif
#if MC02_FEATURE_VISION
#include "master_process.h"
#endif
#if MC02_FEATURE_REFEREE
#include "rm_referee.h"
#endif
#include "bsp_can.h"
#include "usb_transport.h"
#include "bsp_log.h"
#include <string.h>

// 将已采集到的模块加入健康快照可用位图。
static void MarkAvailable(VehicleHealthSourceSnapshot_t *output, VehicleHealthModule_t module)
{
    output->available_modules |= VEHICLE_HEALTH_MODULE_BIT(module);
}

// 将模块标记为降级，保留其他模块的可用状态。
static void MarkDegraded(VehicleHealthSourceSnapshot_t *output, VehicleHealthModule_t module)
{
    output->degraded_modules |= VEHICLE_HEALTH_MODULE_BIT(module);
}

#if defined(MC02_CONFIG_GENERATED)
// 把启动计划的失败和阻塞状态映射为健康快照中的模块状态。
static void MarkStartupDegraded(VehicleHealthSourceSnapshot_t *output)
{
    const uint32_t failed = robot_module_startup.failed | robot_module_startup.blocked;
    static const VehicleHealthModule_t health_module[ROBOT_MODULE_COUNT] = {
        VEHICLE_HEALTH_MODULE_SAFETY, VEHICLE_HEALTH_MODULE_IMU,
        VEHICLE_HEALTH_MODULE_COMMUNICATION, VEHICLE_HEALTH_MODULE_COMMAND,
        VEHICLE_HEALTH_MODULE_CHASSIS, VEHICLE_HEALTH_MODULE_GIMBAL,
        VEHICLE_HEALTH_MODULE_SHOOTER, VEHICLE_HEALTH_MODULE_REFEREE,
        VEHICLE_HEALTH_MODULE_VISION, VEHICLE_HEALTH_MODULE_REMOTE,
        VEHICLE_HEALTH_MODULE_ALARM, VEHICLE_HEALTH_MODULE_LCD,
        VEHICLE_HEALTH_MODULE_UI, VEHICLE_HEALTH_MODULE_SUPERCAP,
        VEHICLE_HEALTH_MODULE_LOG, VEHICLE_HEALTH_MODULE_HEATER};
    // 此表按 ROBOT_MODULE_* 的枚举顺序对应健康模块编号。
    for (unsigned module = 0u; module < ROBOT_MODULE_COUNT; ++module)
        if ((failed & ROBOT_MODULE_BIT(module)) != 0u)
            MarkDegraded(output, health_module[module]);
}
#endif

// 复制构建配置声明的角色与模块集合到本次健康快照。
static void ConfigureRole(VehicleHealthSourceSnapshot_t *output)
{
    VehicleHealthBuildProfile_t profile = {0};
    (void)VehicleHealthGetBuildProfile(&profile);
    output->role = profile.role;
    output->enabled_modules = profile.enabled_modules;
}

uint8_t VehicleHealthSourceCapture(VehicleHealthSourceSnapshot_t *output)
{
    if (output == NULL)
        return 0u;

    VehicleHealthSourceSnapshot_t candidate = {0};
    ConfigureRole(&candidate);
#if defined(MC02_CONFIG_GENERATED)
    MarkStartupDegraded(&candidate);
#endif

    InitDiagnosticSnapshot_s init = {0};
    InitDiagnosticGetSnapshot(&init);
    candidate.init_failed = init.failed;
    candidate.init_stage = init.stage;
    candidate.init_module = init.module;
    candidate.init_reason = init.reason;
    candidate.init_resource_id = init.resource_id;
    candidate.init_failure_count = init.failure_count;
    candidate.init_heartbeat = init.heartbeat;
    if (init.stage != INIT_STAGE_RESET)
        MarkAvailable(&candidate, VEHICLE_HEALTH_MODULE_INIT);
    if ((init.failed != 0u) || (init.stage < INIT_STAGE_TASKS))
        MarkDegraded(&candidate, VEHICLE_HEALTH_MODULE_INIT);

    RobotSafetySnapshot_t safety = {0};
    if (RobotSafetyGetSnapshot(&safety) != 0u)
    {
        candidate.safety_state = (uint8_t)safety.state;
        candidate.safety_fault_flags = safety.fault_flags;
        candidate.safety_estop_latched = safety.estop_latched;
        MarkAvailable(&candidate, VEHICLE_HEALTH_MODULE_SAFETY);
        if ((safety.fault_flags != 0u) || (safety.estop_latched != 0u))
            MarkDegraded(&candidate, VEHICLE_HEALTH_MODULE_SAFETY);
    }

    for (uint32_t task = 0u; task < TASK_MONITOR_COUNT; ++task)
    {
        TaskMonitorStatus_t status = {0};
        TaskMonitorGetStatus((TaskMonitor_Id_e)task, &status);
        if (status.fault != 0u)
            candidate.task_monitor_fault_mask |= (uint8_t)(1u << task);
    }
    if (candidate.task_monitor_fault_mask == 0u)
        MarkAvailable(&candidate, VEHICLE_HEALTH_MODULE_TASK_MONITOR);
    else
        MarkDegraded(&candidate, VEHICLE_HEALTH_MODULE_TASK_MONITOR);

    INS_Control_Snapshot_s imu = {0};
    candidate.imu_valid = INSGetControlSnapshot(&imu);
    candidate.calibration_valid = safety.calib_valid;
    candidate.imu_sample_ms = imu.timestamp_ms;
    candidate.imu_sequence = imu.sequence;
    if (candidate.imu_valid != 0u)
        MarkAvailable(&candidate, VEHICLE_HEALTH_MODULE_IMU);
    else
        MarkDegraded(&candidate, VEHICLE_HEALTH_MODULE_IMU);

    const uint32_t enabled = candidate.enabled_modules;
#if MC02_FEATURE_REMOTE
    if ((enabled & VEHICLE_HEALTH_MODULE_BIT(VEHICLE_HEALTH_MODULE_REMOTE)) != 0u)
    {
        RemoteControlDiagnostics_s remote_diagnostics = {0};
        if (RemoteControlGetDiagnostics(&remote_diagnostics) != 0u)
        {
            RemoteControlSnapshot_s remote = {0};
            RemoteControlGetSnapshot(&remote);
            candidate.remote_online = remote.remote_online;
            candidate.remote_pc_valid = remote.pc_valid;
            candidate.remote_last_valid_ms = remote_diagnostics.last_valid_tick;
            if (remote.remote_online != 0u)
                MarkAvailable(&candidate, VEHICLE_HEALTH_MODULE_REMOTE);
            else
                MarkDegraded(&candidate, VEHICLE_HEALTH_MODULE_REMOTE);
        }
        else
            MarkDegraded(&candidate, VEHICLE_HEALTH_MODULE_REMOTE);
    }
#endif

#if MC02_FEATURE_VISION
    if ((enabled & VEHICLE_HEALTH_MODULE_BIT(VEHICLE_HEALTH_MODULE_VISION)) != 0u)
    {
        Vision_Status_t vision = {0};
        VisionGetSnapshot(NULL, &vision);
        candidate.vision_online = vision.online;
        candidate.vision_has_valid_frame = vision.has_valid_frame;
        candidate.vision_last_rx_ms = vision.last_rx_ms;
        candidate.vision_rx_count = vision.rx_count;
        if ((vision.online != 0u) && (vision.has_valid_frame != 0u))
            MarkAvailable(&candidate, VEHICLE_HEALTH_MODULE_VISION);
        else
            MarkDegraded(&candidate, VEHICLE_HEALTH_MODULE_VISION);
    }
#endif

#if MC02_FEATURE_REFEREE
    if ((enabled & VEHICLE_HEALTH_MODULE_BIT(VEHICLE_HEALTH_MODULE_REFEREE)) != 0u)
    {
        RefereeSnapshot_t referee = {0};
        if (RefereeGetSnapshot(&referee) == REFEREE_OK)
        {
            candidate.referee_online = referee.link_online;
            candidate.referee_link_age_ms = referee.link_age_ms;
            candidate.referee_latest_source_ms = referee.link_source_rx_ms;
            for (uint32_t index = 0u; index < REFEREE_COMMAND_COUNT && index < 32u; ++index)
            {
                if (referee.command[index].state == REFEREE_DATA_FRESH)
                    candidate.referee_fresh_command_mask |= UINT32_C(1) << index;
                else if (referee.command[index].state == REFEREE_DATA_EVENT_RECORDED)
                    candidate.referee_event_command_mask |= UINT32_C(1) << index;
            }
            if ((referee.link_online != 0u) && (referee.has_valid_frame != 0u))
                MarkAvailable(&candidate, VEHICLE_HEALTH_MODULE_REFEREE);
            else
                MarkDegraded(&candidate, VEHICLE_HEALTH_MODULE_REFEREE);
        }
        else
            MarkDegraded(&candidate, VEHICLE_HEALTH_MODULE_REFEREE);
    }
#endif

    for (uint8_t bus = 0u; bus < VEHICLE_HEALTH_CAN_BUS_COUNT; ++bus)
    {
        const VehicleHealthModule_t module = (VehicleHealthModule_t)(VEHICLE_HEALTH_MODULE_CAN1 + bus);
        if ((enabled & VEHICLE_HEALTH_MODULE_BIT(module)) == 0u)
            continue;
        CAN_ObservationCounters_t counters = {0};
        if (CANGetObservationCounters(bus, &counters) == 0u)
            continue;
        if (counters.active != 0u)
            candidate.can_active_mask |= (uint8_t)(1u << bus);
        if (counters.status == (uint32_t)CAN_STATUS_BUSOFF)
            candidate.can_busoff_mask |= (uint8_t)(1u << bus);
        candidate.can_busoff_count[bus] = counters.busoff;
        if (counters.active != 0u)
            MarkAvailable(&candidate, module);
        if ((counters.active == 0u) || (counters.status == (uint32_t)CAN_STATUS_BUSOFF))
            MarkDegraded(&candidate, module);
    }

    if ((enabled & VEHICLE_HEALTH_MODULE_BIT(VEHICLE_HEALTH_MODULE_USB)) != 0u)
    {
        USBTransportState_t usb = {0};
        if (USBTransportGetState(&usb) == USB_OK)
        {
            candidate.usb_phase = (uint8_t)usb.phase;
            candidate.usb_restart_quota_known = 1u;
            candidate.usb_restart_limit = USB_RESTART_LIMIT;
            candidate.usb_restart_attempts = (uint8_t)(usb.restart_attempts > UINT8_MAX ? UINT8_MAX : usb.restart_attempts);
            candidate.usb_restart_remaining = (usb.restart_attempts >= USB_RESTART_LIMIT)
                                                  ? 0u
                                                  : (uint8_t)(USB_RESTART_LIMIT - usb.restart_attempts);
            if (usb.phase == USB_PHASE_READY)
                MarkAvailable(&candidate, VEHICLE_HEALTH_MODULE_USB);
            else
                MarkDegraded(&candidate, VEHICLE_HEALTH_MODULE_USB);
        }
        else
        {
            candidate.usb_restart_limit = UINT8_MAX;
            candidate.usb_restart_attempts = UINT8_MAX;
            candidate.usb_restart_remaining = UINT8_MAX;
            MarkDegraded(&candidate, VEHICLE_HEALTH_MODULE_USB);
        }
    }

    IMUHeaterStatus_t heater = {0};
    IMUHeaterGetStatus(&heater);
    candidate.heater_initialized = heater.initialized;
    candidate.heater_compiled_enabled = heater.enabled;
    candidate.heater_sensor_valid = heater.sensor_valid;
    candidate.heater_pwm_active = heater.heating;
    candidate.heater_fault = heater.fault;
    candidate.heater_overtemp = heater.overtemp;
    candidate.heater_timeout_fault = heater.timeout_fault;
    candidate.heater_duty = heater.duty;
    candidate.temperature_c = heater.temperature;
    if (heater.initialized != 0u)
        MarkAvailable(&candidate, VEHICLE_HEALTH_MODULE_HEATER);
    if ((heater.initialized == 0u) || (heater.sensor_valid == 0u) || (heater.fault != 0u))
        MarkDegraded(&candidate, VEHICLE_HEALTH_MODULE_HEATER);

    BSPLogState_t logging = {0};
    if (BSPLogGetState(&logging) == BSP_LOG_STATUS_OK)
    {
        if ((logging.backend_available != 0u) && (logging.task_started != 0u))
            MarkAvailable(&candidate, VEHICLE_HEALTH_MODULE_LOG);
        else
            MarkDegraded(&candidate, VEHICLE_HEALTH_MODULE_LOG);
    }

    // RobotInit 只有在所选应用模块完成初始化后才进入 TASKS 阶段。
    if ((init.failed == 0u) && (init.stage >= INIT_STAGE_TASKS))
    {
        if ((candidate.enabled_modules & VEHICLE_HEALTH_MODULE_BIT(VEHICLE_HEALTH_MODULE_CHASSIS)) != 0u)
            MarkAvailable(&candidate, VEHICLE_HEALTH_MODULE_CHASSIS);
        if ((candidate.enabled_modules & VEHICLE_HEALTH_MODULE_BIT(VEHICLE_HEALTH_MODULE_GIMBAL)) != 0u)
            MarkAvailable(&candidate, VEHICLE_HEALTH_MODULE_GIMBAL);
        if ((candidate.enabled_modules & VEHICLE_HEALTH_MODULE_BIT(VEHICLE_HEALTH_MODULE_SHOOTER)) != 0u)
            MarkAvailable(&candidate, VEHICLE_HEALTH_MODULE_SHOOTER);
    }

    *output = candidate;
    return 1u;
}

uint32_t VehicleHealthSourceNowMs(void)
{
    return HAL_GetTick();
}

uint8_t VehicleHealthSourceInIsr(void)
{
    return (__get_IPSR() != 0u) ? 1u : 0u;
}

uint32_t VehicleHealthSourceEnterCritical(void)
{
    const uint32_t previous = __get_PRIMASK();
    __disable_irq();
    return previous;
}

void VehicleHealthSourceExitCritical(uint32_t previous)
{
    __set_PRIMASK(previous);
}
