#include "bmi088_host_fixture.h"
#include "bmi088_regNdef.h"
#include "imu_heater.h"
#include "task_monitor.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

BMI088HostFixture_t bmi088_host_fixture;

static SPIInstance accelerometer_spi = {.is_accelerometer = 1u};
static SPIInstance gyroscope_spi = {.is_accelerometer = 0u};
static uint8_t accelerometer_registers[128];
static uint8_t gyroscope_registers[128];
static uint8_t spi_register_count;
static BMI088Instance acquire_instance;

void BMI088HostReset(void)
{
    memset(&bmi088_host_fixture, 0, sizeof(bmi088_host_fixture));
    memset(&accelerometer_spi, 0, sizeof(accelerometer_spi));
    memset(&gyroscope_spi, 0, sizeof(gyroscope_spi));
    memset(&acquire_instance, 0, sizeof(acquire_instance));
    memset(accelerometer_registers, 0, sizeof(accelerometer_registers));
    memset(gyroscope_registers, 0, sizeof(gyroscope_registers));
    accelerometer_spi.is_accelerometer = 1u;
    spi_register_count = 0u;
    bmi088_host_fixture.calibration_monitor_allow = 1u;
    bmi088_host_fixture.accel_chip_id = BMI088_ACC_CHIP_ID_VALUE;
}

void BMI088HostSetKernelRunning(uint8_t running)
{
    bmi088_host_fixture.scheduler_running = running;
}

void BMI088HostSetAccelChipId(uint8_t chip_id)
{
    bmi088_host_fixture.accel_chip_id = chip_id;
}

BMI088Instance *BMI088HostCreateAcquireInstance(void)
{
    acquire_instance.work_mode = BMI088_BLOCK_PERIODIC_MODE;
    acquire_instance.cali_mode = BMI088_LOAD_PRE_CALI_MODE;
    acquire_instance.spi_acc = &accelerometer_spi;
    acquire_instance.spi_gyro = &gyroscope_spi;
    acquire_instance.acc_coef = BMI088_ACCEL_6G_SEN;
    acquire_instance.BMI088_GYRO_SEN = BMI088_GYRO_2000_SEN;
    return &acquire_instance;
}

void BMI088HostFailTransferAfter(uint32_t transfer_offset, HAL_StatusTypeDef status)
{
    bmi088_host_fixture.fail_transfer_number = bmi088_host_fixture.transfer_count + transfer_offset;
    bmi088_host_fixture.fail_status = status;
}

void BMI088HostFailNextTransferAfterParamInit(HAL_StatusTypeDef status)
{
    bmi088_host_fixture.fail_next_transfer_after_param_init = 1u;
    bmi088_host_fixture.param_init_failure_status = status;
}

BMI088Instance *BMI088HostRegisterOnline(void)
{
    BMI088_Init_Config_s config = {0};
    config.work_mode = BMI088_BLOCK_PERIODIC_MODE;
    config.cali_mode = BMI088_CALIBRATE_ONLINE_MODE;
    return BMI088Register(&config);
}

SPIInstance *SPIRegister(SPI_Init_Config_s *config)
{
    if (spi_register_count++ == 0u)
    {
        accelerometer_spi.spi_work_mode = config->spi_work_mode;
        accelerometer_spi.id = config->id;
        accelerometer_registers[BMI088_ACC_CHIP_ID] = bmi088_host_fixture.accel_chip_id;
        return &accelerometer_spi;
    }

    gyroscope_spi.spi_work_mode = config->spi_work_mode;
    gyroscope_spi.id = config->id;
    gyroscope_registers[BMI088_GYRO_CHIP_ID] = BMI088_GYRO_CHIP_ID_VALUE;
    return &gyroscope_spi;
}

HAL_StatusTypeDef SPITransRecv(SPIInstance *spi, uint8_t *rx, uint8_t *tx, uint8_t len)
{
    uint32_t transfer_number = ++bmi088_host_fixture.transfer_count;
    if (transfer_number == bmi088_host_fixture.fail_transfer_number)
    {
        if (len > 0u)
            rx[0] = 0xa5u; /* Model a HAL failure after a partial receive. */
        return bmi088_host_fixture.fail_status;
    }

    memset(rx, 0, len);
    uint8_t reg = (uint8_t)(tx[0] & 0x7fu);
    uint8_t *registers = spi->is_accelerometer ? accelerometer_registers : gyroscope_registers;
    uint8_t data_offset = spi->is_accelerometer ? 2u : 1u;
    for (uint8_t i = data_offset; i < len; ++i)
        rx[i] = registers[(uint8_t)(reg + i - data_offset)];

    if (spi->is_accelerometer && reg == BMI088_ACCEL_XOUT_L && len >= 8u)
    {
        const uint16_t raw_accel[3] = {5464u, 0u, 0u};
        for (uint8_t axis = 0; axis < 3u; ++axis)
        {
            rx[2u + axis * 2u] = (uint8_t)(raw_accel[axis] & 0xffu);
            rx[3u + axis * 2u] = (uint8_t)(raw_accel[axis] >> 8u);
        }
    }
    else if (spi->is_accelerometer && reg == BMI088_TEMP_M && len >= 4u)
    {
        rx[2] = 0xffu;
        rx[3] = 0xe0u;
    }

    return HAL_OK;
}

void SPITransmit(SPIInstance *spi, uint8_t *tx, uint8_t len)
{
    if (len >= 2u)
    {
        uint8_t *registers = spi->is_accelerometer ? accelerometer_registers : gyroscope_registers;
        registers[tx[0]] = tx[1];
    }
}

uint32_t DWT_ProbeStart(void) { return bmi088_host_fixture.synthetic_time_us; }
uint32_t DWT_ProbeElapsedUs(uint32_t start) { return bmi088_host_fixture.synthetic_time_us - start; }
void DWT_Delay(float seconds)
{
    uint32_t elapsed_us = (uint32_t)(seconds * 1000000.0f);
    bmi088_host_fixture.synthetic_time_us += elapsed_us;
    bmi088_host_fixture.unyielded_busy_wait_us += elapsed_us;
    if (bmi088_host_fixture.unyielded_busy_wait_us > bmi088_host_fixture.max_unyielded_busy_wait_us)
        bmi088_host_fixture.max_unyielded_busy_wait_us = bmi088_host_fixture.unyielded_busy_wait_us;
}
uint32_t HAL_GetTick(void) { return bmi088_host_fixture.synthetic_time_us / 1000u; }
float DWT_GetTimeline_s(void) { return (float)bmi088_host_fixture.synthetic_time_us / 1000000.0f; }
void BSP_WatchdogFeed(void) {}
uint8_t osKernelRunning(void) { return bmi088_host_fixture.scheduler_running; }
void osDelay(uint32_t milliseconds)
{
    bmi088_host_fixture.synthetic_time_us += milliseconds * 1000u;
    bmi088_host_fixture.os_delay_call_count++;
    bmi088_host_fixture.unyielded_busy_wait_us = 0u;
}
void TaskMonitorFeed(TaskMonitor_Id_e id)
{
    if (id == TASK_MONITOR_INS)
        bmi088_host_fixture.ins_monitor_feed_count++;
}

void TaskMonitorDelayMs(TaskMonitor_Id_e id, uint32_t milliseconds)
{
    if (bmi088_host_fixture.scheduler_running)
    {
        while (milliseconds > 0u)
        {
            osDelay(1u);
            TaskMonitorFeed(id);
            milliseconds--;
        }
    }
    else
    {
        DWT_Delay((float)milliseconds / 1000.0f);
    }
}

float NormOf3d(float vector[3])
{
    return sqrtf(vector[0] * vector[0] + vector[1] * vector[1] + vector[2] * vector[2]);
}

void *zmalloc(size_t size) { return calloc(1u, size); }
void PIDInit(PIDInstance *pid, PID_Init_Config_s *config) { (void)pid; (void)config; }
PWMInstance *PWMRegister(PWM_Init_Config_s *config) { (void)config; return NULL; }
GPIOInstance *GPIORegister(GPIO_Init_Config_s *config) { (void)config; return NULL; }
void GPIOSet(GPIOInstance *instance) { (void)instance; }
void GPIOReset(GPIOInstance *instance) { (void)instance; }
void RobotSafetySetCalibValid(uint8_t valid) { (void)valid; }
uint8_t TaskMonitorBeginLongOperation(TaskMonitor_Id_e id, TaskMonitor_LongOperation_e operation)
{
    if (id == TASK_MONITOR_INS && operation == TASK_MONITOR_LONG_OPERATION_CALIBRATION)
        bmi088_host_fixture.calibration_monitor_begin_count++;
    return bmi088_host_fixture.calibration_monitor_allow;
}
uint8_t TaskMonitorEndLongOperation(TaskMonitor_Id_e id, TaskMonitor_LongOperation_e operation)
{
    if (id == TASK_MONITOR_INS && operation == TASK_MONITOR_LONG_OPERATION_CALIBRATION)
    {
        bmi088_host_fixture.calibration_monitor_end_count++;
        bmi088_host_fixture.calibration_monitor_end_success = 1u;
    }
    return 1u;
}
uint8_t IMUHeaterBeginLongOperation(IMUHeater_LongOperation_e operation)
{
    if (operation == IMU_HEATER_LONG_OPERATION_CALIBRATION)
        bmi088_host_fixture.calibration_heater_begin_count++;
    return 1u;
}
void IMUHeaterEndLongOperation(IMUHeater_LongOperation_e operation, uint8_t success)
{
    if (operation == IMU_HEATER_LONG_OPERATION_CALIBRATION)
    {
        bmi088_host_fixture.calibration_heater_end_count++;
        bmi088_host_fixture.calibration_heater_end_success = success;
    }
}

uint8_t ParamInit(void)
{
    if (bmi088_host_fixture.fail_next_transfer_after_param_init)
    {
        bmi088_host_fixture.fail_next_transfer_after_param_init = 0u;
        BMI088HostFailTransferAfter(1u, bmi088_host_fixture.param_init_failure_status);
    }
    return 0u;
}
uint8_t ParamGet(uint16_t key, void *buf, uint16_t len) { (void)key; (void)buf; (void)len; return 0u; }
uint8_t ParamGetFloat(uint16_t key, float *out) { (void)key; (void)out; return 0u; }
uint8_t ParamGetFloats(uint16_t key, float *out, uint8_t count) { (void)key; (void)out; (void)count; return 0u; }
uint8_t ParamSet(uint16_t key, const void *buf, uint16_t len)
{
    if (key == PARAM_KEY_IMU_CALIB_META && len == sizeof(bmi088_host_fixture.committed_calibration_meta))
    {
        memcpy(&bmi088_host_fixture.committed_calibration_meta, buf, len);
        bmi088_host_fixture.calibration_meta_committed = 1u;
    }
    return 1u;
}
uint8_t ParamSetFloat(uint16_t key, float value) { (void)key; (void)value; return 1u; }
uint8_t ParamSetFloats(uint16_t key, const float *in, uint8_t count) { (void)key; (void)in; (void)count; return 1u; }
uint8_t ParamCommit(void) { return 1u; }
