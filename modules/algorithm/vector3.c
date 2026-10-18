/**
 * @file vector3.c
 * @brief 检查三维向量并在局部存储计算候选结果；用法见vector3.h。
 * @author Wang Hongxi
 * @author modified by neozng
 */
#include "vector3.h"
#include <float.h>
#include <math.h>
#include <stddef.h>
#include <string.h>

// 固定读取三个分量；可读容量和对齐由调用者保证。
static MathStatus_t check_vector(const float vector[3])
{
    if (vector == NULL) return MATH_BAD_ARGUMENT;
    for (size_t i = 0u; i < 3u; ++i)
        if (!isfinite(vector[i])) return MATH_NONFINITE;
    return MATH_OK;
}

// 缩放后最大分量为1，平方和不会先溢出或全部下溢成0。
static double scaled_length(const float vector[3], double scale)
{
    double sum = 0.0;
    for (size_t i = 0u; i < 3u; ++i)
    {
        double part = (double)vector[i] / scale;
        sum += part * part;
    }
    return sqrt(sum);
}

MathStatus_t Vec3NormF32(const float vector[3], float *output)
{
    if (output == NULL) return MATH_BAD_ARGUMENT;
    MathStatus_t status = check_vector(vector);
    if (status != MATH_OK) return status;
    double scale = fmax(fmax(fabs((double)vector[0]), fabs((double)vector[1])), fabs((double)vector[2]));
    double candidate = scale == 0.0 ? 0.0 : scale * scaled_length(vector, scale);
    if (candidate > FLT_MAX) return MATH_OVERFLOW;
    *output = (float)candidate;
    return MATH_OK;
}

MathStatus_t Vec3NormalizeF32(const float vector[3], float output[3])
{
    if (output == NULL) return MATH_BAD_ARGUMENT;
    MathStatus_t status = check_vector(vector);
    if (status != MATH_OK) return status;
    double scale = fmax(fmax(fabs((double)vector[0]), fabs((double)vector[1])), fabs((double)vector[2]));
    if (scale == 0.0) return MATH_ZERO_VECTOR;
    double length = scaled_length(vector, scale);
    float candidate[3];
    // 必须先读完输入，再写出三个结果，才能支持原地及部分重叠。
    for (size_t i = 0u; i < 3u; ++i)
        candidate[i] = (float)(((double)vector[i] / scale) / length);
    memcpy(output, candidate, sizeof(candidate));
    return MATH_OK;
}

MathStatus_t Vec3DotF32(const float a[3], const float b[3], float *output)
{
    if (output == NULL) return MATH_BAD_ARGUMENT;
    MathStatus_t status = check_vector(a);
    if (status != MATH_OK) return status;
    status = check_vector(b);
    if (status != MATH_OK) return status;
    double candidate = 0.0;
    for (size_t i = 0u; i < 3u; ++i)
        candidate += (double)a[i] * (double)b[i];
    if (fabs(candidate) > FLT_MAX) return MATH_OVERFLOW;
    *output = (float)candidate;
    return MATH_OK;
}

MathStatus_t Vec3CrossF32(const float a[3], const float b[3], float output[3])
{
    if (output == NULL) return MATH_BAD_ARGUMENT;
    MathStatus_t status = check_vector(a);
    if (status != MATH_OK) return status;
    status = check_vector(b);
    if (status != MATH_OK) return status;
    float candidate[3];
    // 用double暂存乘积，既检查最终范围，也允许大乘积相减后得到有限结果。
    for (size_t i = 0u; i < 3u; ++i)
    {
        size_t j = (i + 1u) % 3u, k = (i + 2u) % 3u;
        double part = (double)a[j] * b[k] - (double)a[k] * b[j];
        if (fabs(part) > FLT_MAX) return MATH_OVERFLOW;
        candidate[i] = (float)part;
    }
    memcpy(output, candidate, sizeof(candidate));
    return MATH_OK;
}
