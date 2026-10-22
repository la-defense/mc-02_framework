/**
 * @file bsp_observation.h
 * @brief 提供固件身份和运行统计的固定只读记录。
 * @note BSPObservationInitEarly() 须在 main() 关闭输出后、初始化外设前调用。
 *       BSPObservationCapture() 由 LCD 任务约每 5 秒调用；记录供本机工具读取，
 *       不作为控制许可或安全判断输入。
 * @author MC-02 contributors
 * @license BSD-3-Clause; see LICENSE.
 */
#ifndef BSP_OBSERVATION_H
#define BSP_OBSERVATION_H

#include <stdint.h>

#define BSP_OBSERVATION_BUILD_ID_WORDS 5u
#define BSP_OBSERVATION_PROBE_COUNT 14u
#define BSP_OBSERVATION_CAN_BUS_COUNT 3u

#define BSP_OBSERVATION_IDENTITY_MAGIC 0x4F425331u
#define BSP_OBSERVATION_IDENTITY_VERSION 1u
#define BSP_OBSERVATION_IDENTITY_LAYOUT 0x4F425301u
#define BSP_OBSERVATION_TIMING_LAYOUT 0x4F425302u
#define BSP_OBSERVATION_CAN_LAYOUT 0x4F425303u
#define BSP_OBSERVATION_LCD_LAYOUT 0x4F425304u

typedef enum
{
    BSP_OBSERVATION_PROFILE_BENCH_SAFE = 1u,
    BSP_OBSERVATION_PROFILE_ROBOT = 2u,
} BSP_ObservationProfile_t;

typedef enum
{
    BSP_OBSERVATION_ROLE_SINGLE = 1u,
    BSP_OBSERVATION_ROLE_CHASSIS = 2u,
    BSP_OBSERVATION_ROLE_GIMBAL = 3u,
} BSP_ObservationRole_t;

typedef enum
{
    BSP_OBSERVATION_MODULE_INS = 1u << 0,
    BSP_OBSERVATION_MODULE_MOTOR = 1u << 1,
    BSP_OBSERVATION_MODULE_DJI = 1u << 2,
    BSP_OBSERVATION_MODULE_CAN = 1u << 3,
    BSP_OBSERVATION_MODULE_UART = 1u << 4,
    BSP_OBSERVATION_MODULE_USB = 1u << 5,
    BSP_OBSERVATION_MODULE_LCD = 1u << 6,
} BSP_ObservationModule_t;

typedef enum
{
    BSP_OBSERVATION_PROBE_TASK_MOTOR = 0u,
    BSP_OBSERVATION_PROBE_TASK_INS,
    BSP_OBSERVATION_PROBE_TASK_DAEMON,
    BSP_OBSERVATION_PROBE_TASK_ROBOT,
    BSP_OBSERVATION_PROBE_MOTOR_ALL,
    BSP_OBSERVATION_PROBE_DJI_PID,
    BSP_OBSERVATION_PROBE_DJI_SEND,
    BSP_OBSERVATION_PROBE_INS_ALL,
    BSP_OBSERVATION_PROBE_INS_READ,
    BSP_OBSERVATION_PROBE_INS_EKF,
    BSP_OBSERVATION_PROBE_INS_TEMP,
    BSP_OBSERVATION_PROBE_CAN_TX,
    BSP_OBSERVATION_PROBE_CAN_DISPATCH,
    BSP_OBSERVATION_PROBE_UART_TX,
} BSP_ObservationProbeIndex_t;

typedef struct
{
    uint32_t magic;
    uint32_t version;
    uint32_t size_bytes;
    uint32_t layout;
    uint32_t profile;
    uint32_t role;
    uint32_t module_mask;
} BSP_ObservationIdentityTemplate_t;

typedef struct
{
    uint32_t magic;
    uint32_t version;
    uint32_t size_bytes;
    uint32_t layout;
    uint32_t build_id[BSP_OBSERVATION_BUILD_ID_WORDS];
    uint32_t profile;
    uint32_t role;
    uint32_t module_mask;
} BSP_ObservationIdentity_t;

typedef struct
{
    uint32_t last_us;
    uint32_t max_us;
    uint32_t total_us;
    uint32_t calls;
} BSP_ObservationProbe_t;

typedef struct
{
    uint32_t layout;
    uint32_t generation;
    uint32_t sample_id;
    uint32_t captured_ms;
    uint32_t module_mask;
    BSP_ObservationProbe_t probes[BSP_OBSERVATION_PROBE_COUNT];
} BSP_ObservationTimingRecord_t;

typedef struct
{
    uint32_t active;
    uint32_t tx_ok;
    uint32_t tx_drop;
    uint32_t tx_error;
    uint32_t tx_done;
    uint32_t tx_fifo_empty_events;
    uint32_t busoff;
    uint32_t busoff_recover;
    uint32_t rx_lost;
    uint32_t rx_frames;
    uint32_t rx_queue_drop;
    uint32_t rx_queue_peak;
    uint32_t tx_error_count;
    uint32_t rx_error_count;
    uint32_t status;
} BSP_ObservationCANBus_t;

typedef struct
{
    uint32_t layout;
    uint32_t generation;
    uint32_t sample_id;
    uint32_t captured_ms;
    uint32_t active_bus_mask;
    BSP_ObservationCANBus_t bus[BSP_OBSERVATION_CAN_BUS_COUNT];
} BSP_ObservationCANRecord_t;

typedef struct
{
    uint32_t layout;
    uint32_t generation;
    uint32_t sample_id;
    uint32_t captured_ms;
    uint32_t module_mask;
    uint32_t sample_last_us;
    uint32_t sample_max_us;
    uint32_t update_last_us;
    uint32_t update_max_us;
    uint32_t draw_last_us;
    uint32_t draw_max_us;
    uint32_t init_us;
    uint32_t recover_us;
    uint32_t busy_us;
    uint32_t cycles;
    uint32_t clear_us;
    uint32_t row_us;
    uint32_t row_bytes;
    uint32_t spi_us;
    uint32_t rows;
    uint32_t spi_bytes;
    uint32_t spi_errors;
    uint32_t module_absent;
    uint32_t recover_count;
    uint32_t init_done;
} BSP_ObservationLCDRecord_t;

extern const BSP_ObservationIdentityTemplate_t bsp_observation_identity_template;
extern volatile BSP_ObservationIdentity_t bsp_observation_identity;
extern volatile BSP_ObservationTimingRecord_t bsp_observation_timing;
extern volatile BSP_ObservationCANRecord_t bsp_observation_can;
extern volatile BSP_ObservationLCDRecord_t bsp_observation_lcd;

/**
 * @brief 在 main() 首次关闭执行器输出后登记当前固件身份。
 * @return 1 表示链接器提供了 SHA-1 Build ID 并已发布身份，0 表示记录无效。
 * @note 必须在启动外设前调用；失败时身份标记保持为 0，观测工具会拒绝该固件。
 */
uint8_t BSPObservationInitEarly(void);

/**
 * @brief 复制已选模块的耗时、CAN 与 LCD 统计并发布三组记录。
 * @note 只允许普通任务上下文调用。LCD 任务每次发布任务表后约 5 秒调用一次；
 *       函数不分配内存、不访问控制输入，也不改变任务或外设状态。
 */
void BSPObservationCapture(void);

#endif /* BSP_OBSERVATION_H */
