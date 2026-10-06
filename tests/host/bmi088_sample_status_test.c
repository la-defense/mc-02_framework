#include "bmi088_host_fixture.h"
#include "bmi088_regNdef.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static void fail(const char *message)
{
    fprintf(stderr, "%s\n", message);
    exit(EXIT_FAILURE);
}

static void expect_failed_sample(BMI088Instance *instance,
                                 BMI088_Data_t *sample,
                                 uint32_t transfer_offset,
                                 HAL_StatusTypeDef spi_status,
                                 BMI088_AcquireStatus_e expected_status)
{
    BMI088_Data_t previous_sample = *sample;
    float previous_acc[3];
    float previous_gyro[3];
    memcpy(previous_acc, instance->acc, sizeof(previous_acc));
    memcpy(previous_gyro, instance->gyro, sizeof(previous_gyro));
    float previous_temperature = instance->temperature;
    BMI088HostFailTransferAfter(transfer_offset, spi_status);

    BMI088_AcquireStatus_e actual_status = BMI088Acquire(instance, sample);
    if (actual_status != expected_status)
        fail("BMI088Acquire must return the SPI failure class");

    if (memcmp(sample, &previous_sample, sizeof(previous_sample)) != 0)
        fail("failed or partial BMI088 transfers must not publish a mixed sample");
    if (instance->sample_sequence != previous_sample.sequence)
        fail("failed BMI088 transfers must not advance the published sample sequence");
    if (memcmp(instance->acc, previous_acc, sizeof(previous_acc)) != 0 ||
        memcmp(instance->gyro, previous_gyro, sizeof(previous_gyro)) != 0 ||
        instance->temperature != previous_temperature)
        fail("failed BMI088 transfers must not update the instance's last complete sample");
    if (instance->last_acquire_status != expected_status)
        fail("BMI088 instance diagnostics must preserve the mapped acquire error class");
}

static void expect_non_finite_sample_rejected(BMI088Instance *instance, BMI088_Data_t *sample)
{
    BMI088_Data_t previous_sample = *sample;
    float previous_acc[3];
    float previous_gyro[3];
    memcpy(previous_acc, instance->acc, sizeof(previous_acc));
    memcpy(previous_gyro, instance->gyro, sizeof(previous_gyro));
    float previous_temperature = instance->temperature;
    uint32_t previous_sequence = instance->sample_sequence;

    if (BMI088Acquire(instance, sample) != BMI088_ACQUIRE_INVALID_DATA)
        fail("BMI088 must classify non-finite converted sensor data as invalid");
    if (instance->last_acquire_status != BMI088_ACQUIRE_INVALID_DATA)
        fail("BMI088 diagnostics must retain the invalid-sample status");
    if (memcmp(sample, &previous_sample, sizeof(previous_sample)) != 0)
        fail("non-finite BMI088 data must not replace the last published snapshot");
    if (instance->sample_sequence != previous_sequence)
        fail("non-finite BMI088 data must not advance the sample sequence");
    if (memcmp(instance->acc, previous_acc, sizeof(previous_acc)) != 0 ||
        memcmp(instance->gyro, previous_gyro, sizeof(previous_gyro)) != 0 ||
        instance->temperature != previous_temperature)
        fail("non-finite BMI088 data must not replace the last complete instance sample");
}

int main(void)
{
    BMI088HostReset();
    BMI088_Init_Config_s trigger_config = {0};
    trigger_config.work_mode = BMI088_BLOCK_TRIGGER_MODE;
    if (BMI088Register(&trigger_config) != NULL)
        fail("unsupported asynchronous mode must fail closed instead of publishing transient DMA buffers");

    BMI088Instance *instance = BMI088HostCreateAcquireInstance();

    BMI088_Data_t sample = {0};
    if (BMI088Acquire(instance, &sample) != BMI088_ACQUIRE_OK)
        fail("a complete accelerometer, gyroscope, and temperature read must succeed");
    if (!sample.valid || sample.sequence != 1u || sample.timestamp_ms != HAL_GetTick())
        fail("a complete BMI088 read must publish validity, sequence, and acquisition time");

    instance->acc_coef = NAN;
    expect_non_finite_sample_rejected(instance, &sample);
    instance->acc_coef = BMI088_ACCEL_6G_SEN;

    instance->BMI088_GYRO_SEN = INFINITY;
    expect_non_finite_sample_rejected(instance, &sample);
    instance->BMI088_GYRO_SEN = BMI088_GYRO_2000_SEN;

    instance->work_mode = BMI088_BLOCK_TRIGGER_MODE;
    instance->acc[0] = NAN;
    instance->acc_sample_timestamp_ms = HAL_GetTick();
    instance->gyro_sample_timestamp_ms = HAL_GetTick();
    instance->update_flag.acc = 1u;
    instance->update_flag.gyro = 1u;
    instance->update_flag.imu_ready = 1u;
    expect_non_finite_sample_rejected(instance, &sample);
    instance->work_mode = BMI088_BLOCK_PERIODIC_MODE;

    const HAL_StatusTypeDef spi_status[] = {HAL_ERROR, HAL_BUSY, HAL_TIMEOUT};
    const BMI088_AcquireStatus_e acquire_status[] = {
        BMI088_ACQUIRE_SPI_ERROR,
        BMI088_ACQUIRE_SPI_BUSY,
        BMI088_ACQUIRE_SPI_TIMEOUT,
    };
    for (uint32_t transfer = 1u; transfer <= 3u; ++transfer)
        for (uint32_t status = 0u; status < 3u; ++status)
            expect_failed_sample(instance, &sample, transfer,
                                 spi_status[status], acquire_status[status]);

    BMI088_Data_t wrap_sample = {
        .sequence = 8u,
        .timestamp_ms = UINT32_MAX - 2u,
        .valid = 1u,
    };
    if (!BMI088SampleIsFresh(&wrap_sample, 1u))
        fail("sample-age comparison must tolerate HAL tick wraparound");
    if (BMI088SampleIsFresh(&wrap_sample, 4u))
        fail("samples older than the 5ms IMU freshness limit must be rejected");
    wrap_sample.valid = 0u;
    if (BMI088SampleIsFresh(&wrap_sample, 1u))
        fail("invalid samples must never pass the shared freshness check");

    return EXIT_SUCCESS;
}
