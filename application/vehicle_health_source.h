/**
 * @file vehicle_health_source.h
 * @brief 整车健康聚合器与硬件模块之间的内部快照边界。
 * @note 固件使用 vehicle_health_port_stm32.c；主机测试提供确定性替身。
 */
#ifndef MC02_VEHICLE_HEALTH_SOURCE_H
#define MC02_VEHICLE_HEALTH_SOURCE_H

#include "vehicle_health.h"

typedef struct
{
    VehicleHealthRole_t role;
    uint32_t enabled_modules;
    uint32_t available_modules;
    uint32_t degraded_modules;
    uint8_t init_failed, init_stage, init_module, init_reason;
    uint16_t init_resource_id;
    uint32_t init_failure_count, init_heartbeat;
    uint8_t safety_state, safety_estop_latched;
    uint32_t safety_fault_flags;
    uint8_t task_monitor_fault_mask;
    uint8_t imu_valid, calibration_valid;
    uint32_t imu_sample_ms, imu_sequence;
    uint8_t remote_online, remote_pc_valid;
    uint32_t remote_last_valid_ms;
    uint8_t vision_online, vision_has_valid_frame;
    uint32_t vision_last_rx_ms, vision_rx_count;
    uint8_t referee_online;
    uint32_t referee_fresh_command_mask, referee_event_command_mask;
    uint32_t referee_link_age_ms, referee_latest_source_ms;
    uint8_t can_active_mask, can_busoff_mask;
    uint32_t can_busoff_count[VEHICLE_HEALTH_CAN_BUS_COUNT];
    uint8_t usb_phase, usb_restart_quota_known;
    uint8_t usb_restart_limit, usb_restart_attempts, usb_restart_remaining;
    uint8_t heater_initialized, heater_compiled_enabled, heater_sensor_valid;
    uint8_t heater_pwm_active, heater_fault, heater_overtemp, heater_timeout_fault;
    uint16_t heater_duty;
    float temperature_c;
} VehicleHealthSourceSnapshot_t;

/** @brief 读取当前已启用模块的候选状态。
 * @param[out] output 接收完整候选记录；空指针或来源读取失败返回0，不提交到公共健康快照。
 * @return 全部必需来源均读取成功时返回非零。
 * @note 普通RobotTask上下文调用；不同编译配置只访问已启用的来源模块。
 */
uint8_t VehicleHealthSourceCapture(VehicleHealthSourceSnapshot_t *output);

/** @brief 读取本机HAL毫秒时刻。
 * @return 当前32位HAL tick；0合法，间隔通过无符号差值处理回绕。
 */
uint32_t VehicleHealthSourceNowMs(void);

/** @brief 判断当前调用是否处于中断上下文。
 * @return 中断或异常上下文返回非零，普通任务返回0。
 */
uint8_t VehicleHealthSourceInIsr(void);

/** @brief 进入短临界区以复制健康发布记录。
 * @return 进入前的中断屏蔽状态，原样传给Exit恢复。
 */
uint32_t VehicleHealthSourceEnterCritical(void);

/** @brief 离开健康记录临界区并恢复之前的中断屏蔽状态。
 * @param[in] previous EnterCritical返回的状态值。
 */
void VehicleHealthSourceExitCritical(uint32_t previous);

#endif
