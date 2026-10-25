/**
 * @file vehicle_health.h
 * @brief 汇总整车只读运行状态，并提供固定周期的一致复制快照。
 * @note 先由 RobotInit 初始化，再从 RobotTask 普通任务调用 Process；查询不刷新数据。
 */
#ifndef MC02_VEHICLE_HEALTH_H
#define MC02_VEHICLE_HEALTH_H

#include <stdint.h>
#include "init_diagnostic.h"

#define VEHICLE_HEALTH_UPDATE_PERIOD_MS 50u
#define VEHICLE_HEALTH_CAN_BUS_COUNT 3u
#define VEHICLE_HEALTH_UNKNOWN_QUOTA UINT32_MAX
#define VEHICLE_HEALTH_DIAGNOSTIC_SCHEMA_VERSION 1u
#define VEHICLE_HEALTH_DIAGNOSTIC_DATA_SIZE 118u

typedef enum
{
    VEHICLE_HEALTH_ROLE_BENCH = 0,
    VEHICLE_HEALTH_ROLE_SINGLE,
    VEHICLE_HEALTH_ROLE_CHASSIS,
    VEHICLE_HEALTH_ROLE_GIMBAL
} VehicleHealthRole_t;

typedef enum
{
    VEHICLE_HEALTH_MODULE_INIT = 0,
    VEHICLE_HEALTH_MODULE_SAFETY,
    VEHICLE_HEALTH_MODULE_TASK_MONITOR,
    VEHICLE_HEALTH_MODULE_IMU,
    VEHICLE_HEALTH_MODULE_REMOTE,
    VEHICLE_HEALTH_MODULE_VISION,
    VEHICLE_HEALTH_MODULE_REFEREE,
    VEHICLE_HEALTH_MODULE_CAN1,
    VEHICLE_HEALTH_MODULE_CAN2,
    VEHICLE_HEALTH_MODULE_CAN3,
    VEHICLE_HEALTH_MODULE_USB,
    VEHICLE_HEALTH_MODULE_HEATER,
    VEHICLE_HEALTH_MODULE_LOG,
    VEHICLE_HEALTH_MODULE_CHASSIS,
    VEHICLE_HEALTH_MODULE_GIMBAL,
    VEHICLE_HEALTH_MODULE_SHOOTER,
    VEHICLE_HEALTH_MODULE_COUNT
} VehicleHealthModule_t;

#define VEHICLE_HEALTH_MODULE_BIT(module_) (UINT32_C(1) << (uint32_t)(module_))

typedef enum
{
    VEHICLE_HEALTH_OK = 0,
    VEHICLE_HEALTH_ERROR_ARGUMENT,
    VEHICLE_HEALTH_ERROR_NOT_INITIALIZED,
    VEHICLE_HEALTH_ERROR_NOT_READY,
    VEHICLE_HEALTH_ERROR_CONTEXT,
    VEHICLE_HEALTH_ERROR_CAPTURE,
    VEHICLE_HEALTH_NOT_DUE,
    VEHICLE_HEALTH_ALREADY_INITIALIZED
} VehicleHealthStatus_t;

/** @brief 最近一次按固定周期采集的整车状态；所有源时间均保留原始 HAL 毫秒值。 */
typedef struct
{
    uint32_t sequence;                /**< 非零发布序号，回绕时跳过0。 */
    uint32_t captured_ms;             /**< 聚合完成时刻，单位HAL ms；0合法，不替换各模块源时刻。 */
    VehicleHealthRole_t role;
    uint32_t enabled_modules;         /**< 按当前固件角色配置的模块位图。 */
    uint32_t available_modules;       /**< 当前提供明确有效状态的模块位图；可与degraded同时置位。 */
    uint32_t degraded_modules;        /**< 已启用但不可用或来源报告故障的模块位图。 */
    uint8_t init_failed;
    uint8_t init_stage;
    uint8_t init_module;
    uint8_t init_reason;
    uint16_t init_resource_id;
    uint32_t init_failure_count;
    uint32_t init_heartbeat;
    uint8_t safety_state;
    uint32_t safety_fault_flags;
    uint8_t safety_estop_latched;
    uint8_t task_monitor_fault_mask; /**< 按TaskMonitor_Id_e位序记录未存活任务。 */
    uint8_t imu_valid;
    uint8_t calibration_valid;
    uint32_t imu_sample_ms;           /**< IMU样本源时间，单位HAL ms；0合法。 */
    uint32_t imu_sequence;
    uint8_t remote_online;
    uint8_t remote_pc_valid;
    uint32_t remote_last_valid_ms;    /**< 遥控最后合法接收时间，单位HAL ms。 */
    uint8_t vision_online;
    uint8_t vision_has_valid_frame;
    uint32_t vision_last_rx_ms;       /**< 视觉最后接收时间，单位HAL ms。 */
    uint32_t vision_rx_count;
    uint8_t referee_online;
    uint32_t referee_fresh_command_mask;
    uint32_t referee_event_command_mask;
    uint32_t referee_link_age_ms;     /**< 裁判链路年龄，单位ms；沿用模块计算结果。 */
    uint32_t referee_latest_source_ms;/**< 最近接收块来源时间，单位HAL ms，不代表服务器生成时间。 */
    uint8_t can_active_mask;          /**< CAN服务已激活；不证明总线上有物理报文或对端应答。 */
    uint8_t can_busoff_mask;
    uint32_t can_busoff_count[VEHICLE_HEALTH_CAN_BUS_COUNT]; /**< 每总线累计次数。 */
    uint8_t usb_phase;
    uint8_t usb_restart_quota_known;
    uint8_t usb_restart_limit;
    uint8_t usb_restart_attempts;
    uint8_t usb_restart_remaining;
    uint8_t system_reset_quota_known; /**< MC02-0067未实现时为0，不能把未知表示为零额度。 */
    uint32_t system_reset_quota_remaining; /**< 仅在known为1时有效；未知固定为UINT32_MAX。 */
    uint8_t heater_initialized;
    uint8_t heater_compiled_enabled;
    uint8_t heater_sensor_valid;
    uint8_t heater_pwm_active;       /**< 软件PWM比较值非零，不代表已测到物理电流。 */
    uint8_t heater_fault;
    uint8_t heater_overtemp;
    uint8_t heater_timeout_fault;
    uint8_t thermal_stable_known;     /**< 当前没有温稳判据，因此固定为0。 */
    uint8_t thermal_stable;
    uint16_t heater_duty;             /**< PWM比较值，范围沿用加热器0～500。 */
    float temperature_c;             /**< 摄氏度；heater_sensor_valid为0时仅保留诊断原值。 */
} VehicleHealthSnapshot_t;

/** @brief 当前固件编译时选定的板卡角色和启用模块。 */
typedef struct
{
    VehicleHealthRole_t role;
    uint32_t enabled_modules;
} VehicleHealthBuildProfile_t;

/** @brief 初始化固定状态；RobotInit中在BSP和Safety初始化后调用。
 * @return 首次成功返回OK；重复调用返回ALREADY_INITIALIZED，不清除记录。
 * @note 不创建任务、不分配内存、不改变安全许可。
 */
VehicleHealthStatus_t VehicleHealthInit(void);

/** @brief 复制编译配置；不依赖运行初始化，也不访问硬件。
 * @param[out] output 接收角色及启用模块位图；空指针返回参数错误。
 * @return 参数有效时返回OK；失败时不改写output。
 */
VehicleHealthStatus_t VehicleHealthGetBuildProfile(VehicleHealthBuildProfile_t *output);

/** @brief 从普通任务采集并整体发布状态；建议由5ms RobotTask调用。
 * @return 每50ms更新一次；未到期限返回NOT_DUE，来源失败不发布半份快照。
 * @note 捕获与发布不分配内存；时间间隔用无符号差值处理回绕。
 */
VehicleHealthStatus_t VehicleHealthProcess(void);

/** @brief 复制最近一次发布的快照；查询不采样、不续期。
 * @param[out] output 接收完整副本；失败时保持原值。
 * @return OK或参数、初始化、未就绪及上下文错误。
 */
VehicleHealthStatus_t VehicleHealthGetSnapshot(VehicleHealthSnapshot_t *output);

/** @brief 复制启动事件环；不消费记录，顺序从旧到新，最多16项。
 * @param[out] output 接收事件副本；失败时保持原值。
 * @return OK或参数错误。
 */
VehicleHealthStatus_t VehicleHealthGetInitEvents(InitDiagnosticEventSnapshot_t *output);

/** @brief 将健康快照编码为固定小端诊断负载，不依赖结构体填充布局。
 * @param[in] snapshot 已复制的快照。
 * @param[out] output 接收至少118字节；参数或容量错误时保持原缓冲。
 * @param[in] capacity 输出容量，单位字节。
 * @param[out] written 接收写入长度；成功时为118，失败时保持原值。
 * @return VEHICLE_HEALTH_OK或参数／容量错误。
 */
VehicleHealthStatus_t VehicleHealthEncodeDiagnostic(const VehicleHealthSnapshot_t *snapshot,
                                                     uint8_t *output,
                                                     uint32_t capacity,
                                                     uint32_t *written);

/** @brief 注册USART10的按需健康查询处理器。
 * @return 返回注册结果；参数错误、日志未初始化、上下文错误及重复注册分别保留对应状态。
 * @note 需先完成VehicleHealthInit和BSPLogInit，并在调度器启动前调用；失败不改控制许可。
 */
VehicleHealthStatus_t VehicleHealthDiagnosticInit(void);

#endif
