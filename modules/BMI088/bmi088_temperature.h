/**
 * @file bmi088_temperature.h
 * @brief 将 BMI088 温度寄存器的两个字节换算为摄氏度。
 *
 * BMI088Acquire 读取成功后调用；在线标定使用并保存换算后的温度。
 */
#ifndef MC02_BMI088_TEMPERATURE_H
#define MC02_BMI088_TEMPERATURE_H

#include <stdint.h>

#include "bmi088_regNdef.h"

/**
 * @brief 按 Bosch BMI088 数据手册5.3.7节，将11位温度数据换算为摄氏度。
 * @param[in] temp_msb 从 BMI088_TEMP_M（0x22）读到的字节，提供温度数据的高8位。
 * @param[in] temp_lsb 从 BMI088_TEMP_L（0x23）读到的字节，只使用其中的高3位。
 * @return 温度，单位 °C；将还原后的数值乘0.125，再加23。
 * @note 只进行换算，读取是否成功、温度是否可用由调用方检查。
 */
static inline float BMI088DecodeTemperature(uint8_t temp_msb, uint8_t temp_lsb)
{
    // 将高字节的8位与低字节的高3位拼起来，低字节的其余5位不用。
    uint16_t raw = ((uint16_t)temp_msb << 3u) | (uint16_t)(temp_lsb >> 5u);
    // 大于1023表示相对23°C的温差为负，减2048还原带负号的数值。
    int16_t signed_raw = raw <= 1023u ? (int16_t)raw : (int16_t)((int32_t)raw - 2048);
    return (float)signed_raw * BMI088_TEMP_FACTOR + BMI088_TEMP_OFFSET;
}

#endif
