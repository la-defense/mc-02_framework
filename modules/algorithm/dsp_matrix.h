/** @file dsp_matrix.h
 * @brief 用调用者工作区保护 CMSIS-DSP 求逆输入及最后有效输出。
 * @note 不分配内存，不访问硬件；先准备矩阵和独立工作区，再调用计算入口。
 */
#ifndef MC02_DSP_MATRIX_H
#define MC02_DSP_MATRIX_H
#include "arm_math.h"
#include <stddef.h>

/** @brief 求逆成功且结果有限后才写输出。
 * @param[in] input 非空方阵，行列范围1～255，pData至少可读 n*n 个 float。
 * @param[out] output 同形状方阵，pData至少可写 n*n 个 float，失败保持描述符和数据。
 * @param[in,out] workspace 至少 workspace_floats 个 float，按 float 对齐，内容可被改写。
 * @param[in] workspace_floats float 元素容量，须不小于2*n*n。
 * @return ARM_MATH_SUCCESS、ARGUMENT_ERROR、SIZE_MISMATCH、SINGULAR 或 NANINF。
 * @note 输入、输出和工作区不得重叠，生命周期覆盖调用；失败输入保持原样。
 */
arm_status DSPMatrixInverseF32(const arm_matrix_instance_f32 *input,
    arm_matrix_instance_f32 *output, float *workspace, size_t workspace_floats);
#endif
