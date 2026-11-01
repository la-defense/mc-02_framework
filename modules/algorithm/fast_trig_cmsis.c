/** @file fast_trig_cmsis.c
 * @brief 把快速三角函数交给当前固定版本的 CMSIS-DSP 后端。
 * @note 输入范围和返回值见 fast_trig.h；不保存状态或申请内存。
 */
#include "fast_trig.h"
#include "arm_math.h"

float FastTrigSinF32(float radians)
{
    return arm_sin_f32(radians);
}

float FastTrigCosF32(float radians)
{
    return arm_cos_f32(radians);
}
