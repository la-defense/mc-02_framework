#include "bmi088.h"
#include "bmi088_regNdef.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static SPIInstance accelerometer_spi = {.is_accelerometer = 1u};
static SPIInstance gyroscope_spi = {.is_accelerometer = 0u};
static uint32_t synthetic_time_us;

static void fail(const char *message)
{
    fprintf(stderr, "%s\n", message);
    exit(EXIT_FAILURE);
}

void SPITransRecv(SPIInstance *spi, uint8_t *rx, uint8_t *tx, uint8_t len)
{
    memset(rx, 0, len);

    uint8_t reg = (uint8_t)(tx[0] & 0x7fu);
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
    (void)spi;
    (void)tx;
    (void)len;
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
    (void)key;
    (void)buf;
    (void)len;
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
    BMI088Instance instance = {0};
    instance.work_mode = BMI088_BLOCK_PERIODIC_MODE;
    instance.cali_mode = BMI088_CALIBRATE_ONLINE_MODE;
    instance.spi_acc = &accelerometer_spi;
    instance.spi_gyro = &gyroscope_spi;

    if (!BMI088CalibrateIMU(&instance))
        fail("BMI088CalibrateIMU should accept stable synthetic samples");

    if (fabsf(instance.temperature - 22.875f) > 0.0001f)
        fail("calibration instance temperature must remain Celsius without rescaling");

    if (fabsf(bmi088_calib_temp - 22.875f) > 0.0001f)
        fail("published calibration temperature must remain Celsius without rescaling");

    return EXIT_SUCCESS;
}
