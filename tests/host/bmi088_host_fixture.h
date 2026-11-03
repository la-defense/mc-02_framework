#ifndef MC02_BMI088_HOST_FIXTURE_H
#define MC02_BMI088_HOST_FIXTURE_H

#include "bmi088.h"
#include "bsp_spi.h" // 测试替身需要表示注入的 HAL 状态。
#include "bsp_param.h"

typedef struct
{
    uint32_t synthetic_time_us;
    uint32_t unyielded_busy_wait_us;
    uint32_t max_unyielded_busy_wait_us;
    uint32_t os_delay_call_count;
    uint32_t ins_monitor_feed_count;
    uint32_t transfer_count;
    uint32_t spi_unregister_count; // 统计启动失败回退时释放的有效设备。
    uint32_t failed_spi_transfer_count;
    uint32_t fail_transfer_number;
    uint32_t spi_write_transfer_count;
    uint32_t failed_spi_write_transfer_count;
    HAL_StatusTypeDef fail_status;
    uint8_t fail_next_transfer_after_param_init;
    HAL_StatusTypeDef param_init_failure_status;
    ParamImuCalibrationGroup_t committed_calibration_group;
    uint8_t calibration_group_committed;
    uint32_t legacy_calibration_parameter_access_count;
    ParamImuCalibMeta_t committed_calibration_meta;
    uint8_t calibration_meta_committed;
    uint32_t calibration_monitor_begin_count;
    uint32_t calibration_monitor_end_count;
    uint32_t calibration_heater_begin_count;
    uint32_t calibration_heater_end_count;
    uint8_t calibration_monitor_end_success;
    uint8_t calibration_heater_end_success;
    uint8_t calibration_monitor_allow;
    uint8_t scheduler_running;
    uint8_t accel_chip_id;
    uint8_t post_reset_dummy_reads;
    uint8_t post_reset_dummy_reads_remaining;
    uint32_t accel_soft_reset_count;
    uint32_t accel_chip_id_reads_after_reset;
} BMI088HostFixture_t;

extern BMI088HostFixture_t bmi088_host_fixture;

void BMI088HostReset(void);
void BMI088HostSetKernelRunning(uint8_t running);
void BMI088HostSetAccelChipId(uint8_t chip_id);
void BMI088HostModelPostResetDummyRead(uint8_t enabled);
void BMI088HostFailTransferAfter(uint32_t transfer_offset, HAL_StatusTypeDef status);
void BMI088HostFailNextTransferAfterParamInit(HAL_StatusTypeDef status);
BMI088Instance *BMI088HostCreateAcquireInstance(void);
BMI088Instance *BMI088HostRegisterOnline(void);

#endif
