#ifndef MC02_HOST_CHASSIS_ARM_MATH_H
#define MC02_HOST_CHASSIS_ARM_MATH_H
#include <math.h>
static inline float arm_cos_f32(float value) { return cosf(value); }
static inline float arm_sin_f32(float value) { return sinf(value); }
#endif
