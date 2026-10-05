#ifndef MC02_BMI088_TEMPERATURE_H
#define MC02_BMI088_TEMPERATURE_H

#include <stdint.h>

#include "bmi088_regNdef.h"

/* Bosch BMI088 datasheet §5.3.7: signed 11-bit value, 0.125 °C/LSB, offset 23 °C. */
static inline float BMI088DecodeTemperature(uint8_t temp_msb, uint8_t temp_lsb)
{
    uint16_t raw = ((uint16_t)temp_msb << 3u) | (uint16_t)(temp_lsb >> 5u);
    int16_t signed_raw = raw <= 1023u ? (int16_t)raw : (int16_t)((int32_t)raw - 2048);
    return (float)signed_raw * BMI088_TEMP_FACTOR + BMI088_TEMP_OFFSET;
}

#endif
