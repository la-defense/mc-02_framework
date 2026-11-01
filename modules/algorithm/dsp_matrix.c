/** @file dsp_matrix.c
 * @brief 检查求逆数据范围，复制到两个独立工作区，成功后交付结果；接口见头文件。
 */
#include "dsp_matrix.h"
#include "arm_math.h"
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

// 将厂商结果转换为框架状态；未知错误不被当作成功。
static DSPMatrixStatus_t matrix_status(arm_status status)
{
    switch (status)
    {
    case ARM_MATH_SUCCESS: return DSP_MATRIX_OK; // 后端成功完成计算。
    case ARM_MATH_ARGUMENT_ERROR: return DSP_MATRIX_ARGUMENT; // 后端拒绝参数。
    case ARM_MATH_LENGTH_ERROR: return DSP_MATRIX_LENGTH; // 后端拒绝数据长度。
    case ARM_MATH_SIZE_MISMATCH: return DSP_MATRIX_SIZE; // 矩阵形状不匹配。
    case ARM_MATH_NANINF: return DSP_MATRIX_NONFINITE; // 后端发现非有限值。
    case ARM_MATH_SINGULAR: return DSP_MATRIX_SINGULAR; // 方阵不可逆。
    default: return DSP_MATRIX_BACKEND; // 保留尚未识别的失败。
    }
}

// 将框架描述符复制到临时厂商描述符，避免用指针强转造成类型别名问题。
static arm_matrix_instance_f32 native_matrix(const DSPMatrixF32_t *matrix)
{
    arm_matrix_instance_f32 native = {matrix->numRows, matrix->numCols, matrix->pData};
    return native;
}

void DSPMatrixInitF32(DSPMatrixF32_t *matrix, uint16_t rows, uint16_t columns, float *data)
{
    if (matrix == NULL) return;
    matrix->numRows = rows; // 记录行数供后续尺寸检查。
    matrix->numCols = columns; // 记录列数供后续尺寸检查。
    matrix->pData = data; // 借用调用者保存的数据，不取得所有权。
}

DSPMatrixStatus_t DSPMatrixAddF32(const DSPMatrixF32_t *a, const DSPMatrixF32_t *b, DSPMatrixF32_t *output)
{
    if (a == NULL || b == NULL || output == NULL || a->pData == NULL || b->pData == NULL || output->pData == NULL)
        return DSP_MATRIX_ARGUMENT;
    arm_matrix_instance_f32 native_a = native_matrix(a); // 输入描述符只在本次调用使用。
    arm_matrix_instance_f32 native_b = native_matrix(b); // 保留第二个输入的原数据地址。
    arm_matrix_instance_f32 native_output = native_matrix(output); // 后端直接写调用者输出。
    return matrix_status(arm_mat_add_f32(&native_a, &native_b, &native_output)); // 返回转换后的结果。
}

DSPMatrixStatus_t DSPMatrixSubF32(const DSPMatrixF32_t *a, const DSPMatrixF32_t *b, DSPMatrixF32_t *output)
{
    if (a == NULL || b == NULL || output == NULL || a->pData == NULL || b->pData == NULL || output->pData == NULL)
        return DSP_MATRIX_ARGUMENT;
    arm_matrix_instance_f32 native_a = native_matrix(a); // 输入描述符只在本次调用使用。
    arm_matrix_instance_f32 native_b = native_matrix(b); // 保留减数的原数据地址。
    arm_matrix_instance_f32 native_output = native_matrix(output); // 后端直接写调用者输出。
    return matrix_status(arm_mat_sub_f32(&native_a, &native_b, &native_output)); // 返回转换后的结果。
}

DSPMatrixStatus_t DSPMatrixMultiplyF32(const DSPMatrixF32_t *a, const DSPMatrixF32_t *b, DSPMatrixF32_t *output)
{
    if (a == NULL || b == NULL || output == NULL || a->pData == NULL || b->pData == NULL || output->pData == NULL)
        return DSP_MATRIX_ARGUMENT;
    arm_matrix_instance_f32 native_a = native_matrix(a); // 转换左矩阵描述符。
    arm_matrix_instance_f32 native_b = native_matrix(b); // 转换右矩阵描述符。
    arm_matrix_instance_f32 native_output = native_matrix(output); // 结果仍写入调用者数组。
    return matrix_status(arm_mat_mult_f32(&native_a, &native_b, &native_output)); // 保留后端尺寸检查。
}

DSPMatrixStatus_t DSPMatrixTransposeF32(const DSPMatrixF32_t *input, DSPMatrixF32_t *output)
{
    if (input == NULL || output == NULL || input->pData == NULL || output->pData == NULL)
        return DSP_MATRIX_ARGUMENT;
    arm_matrix_instance_f32 native_input = native_matrix(input); // 输入尺寸和地址复制到后端格式。
    arm_matrix_instance_f32 native_output = native_matrix(output); // 输出数组由调用者持有。
    return matrix_status(arm_mat_trans_f32(&native_input, &native_output)); // 返回转换后的结果。
}

DSPMatrixStatus_t DSPMatrixInverseF32(const DSPMatrixF32_t *input,
    DSPMatrixF32_t *output, float *workspace, size_t workspace_floats)
{
    // 只有输入、输出和工作区都存在时才读取描述符。
    if (input == NULL || output == NULL || workspace == NULL ||
        input->pData == NULL || output->pData == NULL) return DSP_MATRIX_ARGUMENT;
    size_t n = input->numRows; // 用输入行数决定方阵大小。
    // 行列数和输出尺寸必须一致，并限制最大工作区维度。
    if (n == 0u || n > 255u || input->numCols != n || output->numRows != n || output->numCols != n)
        return DSP_MATRIX_SIZE;
    size_t count = n * n, bytes = count * sizeof(float); // 计算实际访问的元素及字节数。
    // 所有内存区必须对齐、足够大且互不覆盖，避免破坏输入或描述符。
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
        overlap(input, sizeof(*input), workspace, 2u * bytes)) return DSP_MATRIX_ARGUMENT;
    // 输入存在 NaN 或无穷值时不调用消元，输出保持旧值。
    for (size_t i = 0u; i < count; ++i)
        if (!isfinite(input->pData[i])) return DSP_MATRIX_NONFINITE;
    // 厂商函数会对输入消元，不能直接把原输入交给它。
    memcpy(workspace, input->pData, bytes); // 第一半工作区保存可消元的输入副本。
    arm_matrix_instance_f32 source = {(uint16_t)n, (uint16_t)n, workspace}; // 后端读取并改写副本。
    arm_matrix_instance_f32 destination = {(uint16_t)n, (uint16_t)n, workspace + count}; // 第二半保存候选结果。
    arm_status status = arm_mat_inverse_f32(&source, &destination); // 在独立工作区执行消元。
    if (status != ARM_MATH_SUCCESS) return matrix_status(status); // 求逆失败时不提交输出。
    // 后端报告成功后仍逐项检查，避免把非有限候选值发布给调用者。
    for (size_t i = 0u; i < count; ++i)
        if (!isfinite(destination.pData[i])) return DSP_MATRIX_NONFINITE;
    memcpy(output->pData, destination.pData, bytes); // 检查全部通过后才复制结果。
    return DSP_MATRIX_OK; // 告知调用者可以使用输出。
}
