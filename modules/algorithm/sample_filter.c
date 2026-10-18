/**
 * @file sample_filter.c
 * @brief 先检查历史和候选平均值，再改写窗口；无硬件、分配及共享历史。
 * @author Wang Hongxi
 * @author modified by neozng
 */
#include "sample_filter.h"
#include <math.h>
#include <stdint.h>
#include <string.h>

MathStatus_t MovingAverageF32(float new_data, float *buffer, size_t capacity,
                            size_t length, float *output)
{
    if (buffer == NULL || output == NULL) return MATH_BAD_ARGUMENT;
    if (length == 0u || length > 255u) return MATH_BAD_RANGE;
    if (capacity < length) return MATH_BAD_ARGUMENT;
    size_t bytes = length * sizeof(float);
    uintptr_t start = (uintptr_t)buffer, out = (uintptr_t)output;
    // 地址先检查加法和对齐，避免后续读取无效或把结果写回历史中。
    if (start % _Alignof(float) != 0u || out % _Alignof(float) != 0u ||
        bytes > UINTPTR_MAX - start || sizeof(float) > UINTPTR_MAX - out ||
        (start < out + sizeof(float) && out < start + bytes))
        return MATH_BAD_ARGUMENT;
    if (!isfinite(new_data)) return MATH_NONFINITE;
    double sum = new_data;
    // 连将被丢弃的最早元素也检查，发现历史损坏时保持整窗供调用者处理。
    for (size_t i = 0u; i < length; ++i)
    {
        if (!isfinite(buffer[i])) return MATH_NONFINITE;
        if (i != 0u) sum += buffer[i];
    }
    // 有限float的平均值仍在float范围内，double累加避免中间求和溢出。
    float candidate = (float)(sum / (double)length);
    memmove(buffer, buffer + 1u, (length - 1u) * sizeof(float));
    buffer[length - 1u] = new_data;
    *output = candidate;
    return MATH_OK;
}
