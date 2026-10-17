/**
 * @file bmi088_data.h
 * @brief 定义一份完整的 BMI088 采样数据，并提供使用前的时间检查。
 *
 * BMI088Acquire 填写数据、序号和时间；INS 与加热器通过 BMI088SampleIsFresh
 * 检查这份数据是否仍可使用。尚未采样时将结构体置0，采样后保留原序号和时间。
 */
#ifndef BMI088_DATA_H
#define BMI088_DATA_H

#include <stdint.h>

#define BMI088_SAMPLE_MAX_AGE_MS 5u /* 从 timestamp_ms 起允许使用的最长时间，单位 ms */

/** @brief 按传感器 x/y/z 轴排列的数据；安装方向的修正由使用数据的代码完成。 */
typedef struct
{
    float gyro[3];         /**< x/y/z 轴角速度，单位 rad/s（弧度每秒）。 */
    float acc[3];          /**< x/y/z 轴加速度，单位 m/s²，包含重力的影响。 */
    float temperature;     /**< 传感器温度，单位 °C。 */
    uint32_t sequence;     /**< 每次成功采样加1，超过最大值后从1继续；0表示尚无数据。 */
    uint32_t timestamp_ms; /**< 本次完整数据准备好时的 HAL 毫秒计数，来自 HAL_GetTick()。 */
    uint8_t valid;         /**< 非0表示已标记可用；使用时仍需检查序号和时间。 */
} BMI088_Data_t;

/**
 * @brief 检查数据是否已标记可用，并且距准备完成不超过5ms。
 * @param[in] sample 待检查数据，空指针返回0；函数不改写其中的内容。
 * @param[in] now_ms 当前毫秒计数，应与 timestamp_ms 一样来自 HAL_GetTick()。
 * @return sample 非空、valid 和 sequence 均非0，且经过时间不大于5ms时返回1；其余返回0。
 * @note 本函数不比较上次序号，也不检查温度、角速度或加速度的数值范围。
 */
static inline uint8_t BMI088SampleIsFresh(const BMI088_Data_t *sample, uint32_t now_ms)
{
    // && 从左到右检查，空指针时不读取字段；无符号相减可处理毫秒计数从0继续的情况。
    return (sample != 0 && sample->valid && sample->sequence != 0u &&
            (uint32_t)(now_ms - sample->timestamp_ms) <= BMI088_SAMPLE_MAX_AGE_MS) ? 1u : 0u;
}

#endif
