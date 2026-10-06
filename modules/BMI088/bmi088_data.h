#ifndef BMI088_DATA_H
#define BMI088_DATA_H

#include <stdint.h>

#define BMI088_SAMPLE_MAX_AGE_MS 5u

typedef struct
{
    float gyro[3];
    float acc[3];
    float temperature;
    uint32_t sequence;
    uint32_t timestamp_ms;
    uint8_t valid;
} BMI088_Data_t;

static inline uint8_t BMI088SampleIsFresh(const BMI088_Data_t *sample, uint32_t now_ms)
{
    return (sample != 0 && sample->valid && sample->sequence != 0u &&
            (uint32_t)(now_ms - sample->timestamp_ms) <= BMI088_SAMPLE_MAX_AGE_MS) ? 1u : 0u;
}

#endif
