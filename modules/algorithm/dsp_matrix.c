/** @file dsp_matrix.c
 * @brief 检查求逆数据范围，复制到两个独立工作区，成功后交付结果；接口见头文件。
 */
#include "dsp_matrix.h"
#include <math.h>
#include <stdint.h>
#include <string.h>

// 先检查地址加法，再判断两个实际使用区间是否相交。
static int overlap(const void *a, size_t an, const void *b, size_t bn)
{
    uintptr_t ap = (uintptr_t)a, bp = (uintptr_t)b;
    if (an > UINTPTR_MAX - ap || bn > UINTPTR_MAX - bp) return 1;
    return ap < bp + bn && bp < ap + an;
}

arm_status DSPMatrixInverseF32(const arm_matrix_instance_f32 *input,
    arm_matrix_instance_f32 *output, float *workspace, size_t workspace_floats)
{
    if (input == NULL || output == NULL || workspace == NULL ||
        input->pData == NULL || output->pData == NULL) return ARM_MATH_ARGUMENT_ERROR;
    size_t n = input->numRows;
    if (n == 0u || n > 255u || input->numCols != n || output->numRows != n || output->numCols != n)
        return ARM_MATH_SIZE_MISMATCH;
    size_t count = n * n, bytes = count * sizeof(float);
    if (workspace_floats < 2u * count || (uintptr_t)workspace % _Alignof(float) != 0u ||
        (uintptr_t)input->pData % _Alignof(float) != 0u || (uintptr_t)output->pData % _Alignof(float) != 0u ||
        overlap(input->pData, bytes, output->pData, bytes) ||
        overlap(input->pData, bytes, workspace, 2u * bytes) ||
        overlap(output->pData, bytes, workspace, 2u * bytes) ||
        overlap(input->pData, bytes, input, sizeof(*input)) ||
        overlap(input->pData, bytes, output, sizeof(*output)) ||
        overlap(output->pData, bytes, input, sizeof(*input)) ||
        overlap(output->pData, bytes, output, sizeof(*output)) ||
        overlap(output, sizeof(*output), workspace, 2u * bytes) ||
        overlap(input, sizeof(*input), workspace, 2u * bytes)) return ARM_MATH_ARGUMENT_ERROR;
    for (size_t i = 0u; i < count; ++i)
        if (!isfinite(input->pData[i])) return ARM_MATH_NANINF;
    // 厂商函数会对输入消元，不能直接把原输入交给它。
    memcpy(workspace, input->pData, bytes);
    arm_matrix_instance_f32 source = {(uint16_t)n, (uint16_t)n, workspace};
    arm_matrix_instance_f32 destination = {(uint16_t)n, (uint16_t)n, workspace + count};
    arm_status status = arm_mat_inverse_f32(&source, &destination);
    if (status != ARM_MATH_SUCCESS) return status;
    for (size_t i = 0u; i < count; ++i)
        if (!isfinite(destination.pData[i])) return ARM_MATH_NANINF;
    memcpy(output->pData, destination.pData, bytes);
    return ARM_MATH_SUCCESS;
}
