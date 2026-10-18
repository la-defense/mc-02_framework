/**
 * @file vector3.h
 * @brief 三维float向量纯计算；无需初始化，所有输入输出存储须覆盖本次调用。
 * @note 向量须非空、按float对齐且至少包含3个元素。输入输出允许完全或部分重叠；
 * 失败保持输出。不同调用使用独立可写存储时可以并发。
 * @author Wang Hongxi
 * @author modified by neozng
 */
#ifndef MC02_VECTOR3_H
#define MC02_VECTOR3_H
#include "math_utils.h"

/** @brief 计算三维向量模长，零向量返回0。
 * @param[in] vector 三个有限分量，模长沿用分量单位。
 * @param[out] output 非空float结果；允许指向输入元素，失败保持原值。
 * @return MATH_OK、MATH_BAD_ARGUMENT、MATH_NONFINITE 或 MATH_OVERFLOW。
 */
MathStatus_t Vec3NormF32(const float vector[3], float *output);

/** @brief 将非零向量缩放为单位方向。
 * @param[in] vector 三个有限分量，必须至少一个非零。
 * @param[out] output 三个无量纲结果，允许与输入重叠，失败全部保持原值。
 * @return MATH_OK、MATH_BAD_ARGUMENT、MATH_NONFINITE 或 MATH_ZERO_VECTOR。
 */
MathStatus_t Vec3NormalizeF32(const float vector[3], float output[3]);

/** @brief 求两个向量点积。
 * @param[in] a 三个有限分量。
 * @param[in] b 三个有限分量。
 * @param[out] output 非空结果，单位为a与b单位的乘积，失败保持原值。
 * @return MATH_OK、MATH_BAD_ARGUMENT、MATH_NONFINITE 或 MATH_OVERFLOW。
 */
MathStatus_t Vec3DotF32(const float a[3], const float b[3], float *output);

/** @brief 求a叉乘b，所有分量算完后才写结果。
 * @param[in] a 三个有限分量。
 * @param[in] b 三个有限分量。
 * @param[out] output 三个结果，单位为a与b单位的乘积，可与输入重叠；失败全部保持原值。
 * @return MATH_OK、MATH_BAD_ARGUMENT、MATH_NONFINITE 或 MATH_OVERFLOW。
 */
MathStatus_t Vec3CrossF32(const float a[3], const float b[3], float output[3]);
#endif
