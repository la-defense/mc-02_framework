#include "bmi088.h"
#include "bmi088_regNdef.h"
#include "bsp_param.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static SPIInstance accelerometer_spi = {.is_accelerometer = 1u};
static SPIInstance gyroscope_spi = {.is_accelerometer = 0u};
static uint32_t synthetic_time_us;
static uint8_t accelerometer_registers[128];
static uint8_t gyroscope_registers[128];
static uint8_t spi_register_count;
static ParamImuCalibMeta_t committed_calibration_meta;
static uint8_t calibration_meta_committed;

static void fail(const char *message)
{
    fprintf(stderr, "%s\n", message);
    exit(EXIT_FAILURE);
}

SPIInstance *SPIRegister(SPI_Init_Config_s *config)
{
    if (spi_register_count++ == 0u)
    {
        accelerometer_spi.spi_work_mode = config->spi_work_mode;
        accelerometer_spi.id = config->id;
        accelerometer_registers[BMI088_ACC_CHIP_ID] = BMI088_ACC_CHIP_ID_VALUE;
        return &accelerometer_spi;
    }

    gyroscope_spi.spi_work_mode = config->spi_work_mode;
    gyroscope_spi.id = config->id;
    gyroscope_registers[BMI088_GYRO_CHIP_ID] = BMI088_GYRO_CHIP_ID_VALUE;
    return &gyroscope_spi;
}

void SPITransRecv(SPIInstance *spi, uint8_t *rx, uint8_t *tx, uint8_t len)
{
    memset(rx, 0, len);

    uint8_t reg = (uint8_t)(tx[0] & 0x7fu);
    uint8_t *registers = spi->is_accelerometer ? accelerometer_registers : gyroscope_registers;
    uint8_t data_offset = spi->is_accelerometer ? 2u : 1u;
    for (uint8_t i = data_offset; i < len; ++i)
        rx[i] = registers[(uint8_t)(reg + i - data_offset)];

    if (spi->is_accelerometer && reg == BMI088_ACCEL_XOUT_L && len >= 8u)
    {
        /* 5464 LSB at the configured 6g sensitivity is approximately 9.8 m/s^2. */
        const uint16_t raw_accel[3] = {5464u, 0u, 0u};
        for (uint8_t axis = 0; axis < 3u; ++axis)
        {
            rx[2u + axis * 2u] = (uint8_t)(raw_accel[axis] & 0xffu);
            rx[3u + axis * 2u] = (uint8_t)(raw_accel[axis] >> 8u);
        }
    }
    else if (spi->is_accelerometer && reg == BMI088_TEMP_M && len >= 4u)
    {
        /* 2047 is -1 in signed 11-bit form, or 22.875 degrees Celsius. */
        rx[2] = 0xffu;
        rx[3] = 0xe0u;
    }
}

void SPITransmit(SPIInstance *spi, uint8_t *tx, uint8_t len)
{
    if (len >= 2u)
    {
        uint8_t *registers = spi->is_accelerometer ? accelerometer_registers : gyroscope_registers;
        registers[tx[0]] = tx[1];
    }
}

uint32_t DWT_ProbeStart(void)
{
    return synthetic_time_us;
}

uint32_t DWT_ProbeElapsedUs(uint32_t start)
{
    return synthetic_time_us - start;
}

void DWT_Delay(float seconds)
{
    synthetic_time_us += (uint32_t)(seconds * 1000000.0f);
}

void BSP_WatchdogFeed(void) {}
uint32_t HAL_GetTick(void) { return synthetic_time_us / 1000u; }
uint8_t osKernelRunning(void) { return 0u; }
void osDelay(uint32_t milliseconds) { synthetic_time_us += milliseconds * 1000u; }

float NormOf3d(float vector[3])
{
    return sqrtf(vector[0] * vector[0] + vector[1] * vector[1] + vector[2] * vector[2]);
}

void *zmalloc(size_t size)
{
    return calloc(1u, size);
}

void PIDInit(PIDInstance *pid, PID_Init_Config_s *config)
{
    (void)pid;
    (void)config;
}

PWMInstance *PWMRegister(PWM_Init_Config_s *config)
{
    (void)config;
    return NULL;
}

GPIOInstance *GPIORegister(GPIO_Init_Config_s *config)
{
    (void)config;
    return NULL;
}

void RobotSafetySetCalibValid(uint8_t valid) { (void)valid; }
void TaskMonitorPause(void) {}
void TaskMonitorResume(void) {}

uint8_t ParamInit(void) { return 0u; }
uint8_t ParamGet(uint16_t key, void *buf, uint16_t len)
{
    (void)key;
    (void)buf;
    (void)len;
    return 0u;
}
uint8_t ParamGetFloat(uint16_t key, float *out) { (void)key; (void)out; return 0u; }
uint8_t ParamGetFloats(uint16_t key, float *out, uint8_t count)
{
    (void)key;
    (void)out;
    (void)count;
    return 0u;
}
uint8_t ParamSet(uint16_t key, const void *buf, uint16_t len)
{
    if (key == PARAM_KEY_IMU_CALIB_META && len == sizeof(committed_calibration_meta))
    {
        memcpy(&committed_calibration_meta, buf, len);
        calibration_meta_committed = 1u;
    }
    return 1u;
}
uint8_t ParamSetFloat(uint16_t key, float value) { (void)key; (void)value; return 1u; }
uint8_t ParamSetFloats(uint16_t key, const float *in, uint8_t count)
{
    (void)key;
    (void)in;
    (void)count;
    return 1u;
}
uint8_t ParamCommit(void) { return 1u; }

void GPIOSet(GPIOInstance *instance) { (void)instance; }
void GPIOReset(GPIOInstance *instance) { (void)instance; }

int main(void)
{
    BMI088_Init_Config_s config = {0};
    config.work_mode = BMI088_BLOCK_PERIODIC_MODE;
    config.cali_mode = BMI088_CALIBRATE_ONLINE_MODE;

    BMI088Instance *instance = BMI088Register(&config);
    if (instance == NULL)
        fail("BMI088Register should initialize and calibrate stable synthetic samples");

    if (fabsf(instance->temperature - 22.875f) > 0.0001f)
        fail("calibration instance temperature must remain Celsius without rescaling");

    if (fabsf(bmi088_calib_temp - 22.875f) > 0.0001f)
        fail("published calibration temperature must remain Celsius without rescaling");

    if (!calibration_meta_committed || committed_calibration_meta.result != PARAM_IMU_CALIB_OK)
        fail("successful startup calibration must commit metadata through the parameter API");

    if (fabsf(committed_calibration_meta.temperature - 22.875f) > 0.0001f)
        fail("persisted calibration temperature must remain Celsius without rescaling");

    return EXIT_SUCCESS;
}
