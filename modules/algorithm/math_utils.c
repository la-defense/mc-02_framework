/**
 * @file math_utils.c
 * @brief 实现标量和角度检查；公开用法见 math_utils.h，不访问硬件或分配内存。
 * @author Wang Hongxi
 * @author modified by neozng
 */
#include "math_utils.h"
#include <math.h>
#include <stddef.h>

// 共同检查区间，零宽区间是否允许由具体入口决定。
static MathStatus_t check_range(float value, float minimum, float maximum)
{
    if (!isfinite(value) || !isfinite(minimum) || !isfinite(maximum))
        return MATH_NONFINITE;
    return minimum <= maximum ? MATH_OK : MATH_BAD_RANGE;
}

MathStatus_t MathSqrtF32(float value, float *output)
{
    if (output == NULL) return MATH_BAD_ARGUMENT;
    if (!isfinite(value)) return MATH_NONFINITE;
    if (value < 0.0f) return MATH_BAD_RANGE;
    *output = sqrtf(value);
    return MATH_OK;
}

MathStatus_t MathClampF32(float value, float minimum, float maximum, float *output)
{
    if (output == NULL) return MATH_BAD_ARGUMENT;
    MathStatus_t status = check_range(value, minimum, maximum);
    if (status != MATH_OK) return status;
    *output = value < minimum ? minimum : (value > maximum ? maximum : value);
    return MATH_OK;
}

MathStatus_t MathClampI16(int16_t value, int16_t minimum, int16_t maximum, int16_t *output)
{
    if (output == NULL) return MATH_BAD_ARGUMENT;
    if (minimum > maximum) return MATH_BAD_RANGE;
    *output = value < minimum ? minimum : (value > maximum ? maximum : value);
    return MATH_OK;
}

MathStatus_t MathAbsLimitF32(float value, float limit, float *output)
{
    if (output == NULL) return MATH_BAD_ARGUMENT;
    if (!isfinite(value) || !isfinite(limit)) return MATH_NONFINITE;
    if (limit < 0.0f) return MATH_BAD_RANGE;
    return MathClampF32(value, -limit, limit, output);
}

MathStatus_t MathSignF32(float value, float *output)
{
    if (output == NULL) return MATH_BAD_ARGUMENT;
    if (!isfinite(value)) return MATH_NONFINITE;
    *output = value < 0.0f ? -1.0f : 1.0f;
    return MATH_OK;
}

MathStatus_t MathDeadbandF32(float value, float minimum, float maximum, float *output)
{
    if (output == NULL) return MATH_BAD_ARGUMENT;
    MathStatus_t status = check_range(value, minimum, maximum);
    if (status != MATH_OK) return status;
    // 等于边界仍保留输入，延续原死区规则。
    *output = value > minimum && value < maximum ? 0.0f : value;
    return MATH_OK;
}

MathStatus_t MathNearlyEqualF32(float a, float b, float absolute_tolerance, uint8_t *output)
{
    if (output == NULL) return MATH_BAD_ARGUMENT;
    if (!isfinite(a) || !isfinite(b) || !isfinite(absolute_tolerance)) return MATH_NONFINITE;
    if (absolute_tolerance < 0.0f) return MATH_BAD_RANGE;
    // 用double求差，两个极大且符号相反的float也能正常比较。
    *output = (uint8_t)(fabs((double)a - (double)b) <= (double)absolute_tolerance);
    return MATH_OK;
}

MathStatus_t MathRoundI32(float value, int32_t *output)
{
    if (output == NULL) return MATH_BAD_ARGUMENT;
    if (!isfinite(value)) return MATH_NONFINITE;
    double candidate = round((double)value);
    // 先检查再转换，避免把int32_t上界附近的float直接强转为整数。
    if (candidate < (double)INT32_MIN || candidate > (double)INT32_MAX)
        return MATH_OVERFLOW;
    *output = (int32_t)candidate;
    return MATH_OK;
}

MathStatus_t MathWrapF32(float value, float minimum, float maximum, float *output)
{
    if (output == NULL) return MATH_BAD_ARGUMENT;
    MathStatus_t status = check_range(value, minimum, maximum);
    if (status != MATH_OK) return status;
    if (minimum == maximum) return MATH_BAD_RANGE;
    if (value >= minimum && value < maximum)
    {
        *output = value;
        return MATH_OK;
    }
    double width = (double)maximum - (double)minimum;
    // 分别取余再求差，极大输入不会在减去较小下界时丢掉该下界。
    double remainder = fmod(fmod((double)value, width) - fmod((double)minimum, width), width);
    if (remainder < 0.0) remainder += width;
    float candidate = (float)((double)minimum + remainder);
    // 单精度舍入可能到达上界，仍按半开区间返回下界。
    if (candidate >= maximum) candidate = minimum;
    if (candidate < minimum) candidate = minimum;
    *output = candidate;
    return MATH_OK;
}

MathStatus_t MathWrapDegreesF32(float degrees, float *output)
{
    return MathWrapF32(degrees, -180.0f, 180.0f, output);
}

MathStatus_t MathWrapRadiansF32(float radians, float *output)
{
    const float pi = 3.14159265358979323846f;
    return MathWrapF32(radians, -pi, pi, output);
}
