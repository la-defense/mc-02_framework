/**
 * @file RLS.c
 * @brief 用调用者工作区执行递推最小二乘更新。
 * 入口见 RLS.h；模块依赖同源 CMSIS-DSP 矩阵函数，不申请堆内存。
 */
#include "RLS.h"

#include "arm_math.h"

#include <math.h>
#include <string.h>

#define RLS_INSTANCE_MAGIC 0x524C5331U

typedef struct
{
    float *covariance;
    float *covariance_candidate;
    float *outer_product;
    float *params;
    float *params_candidate;
    float *gain;
    float *px;
    float *row;
    double *ldl;
    double *diagonal;
} RLSWorkspaceLayout_t;

static RLSStatus_t RLSCalculateWorkspaceSize(uint16_t dimension,
                                             size_t *bytes,
                                             size_t *float_bytes)
{
    size_t square;
    size_t float_count;
    size_t double_count;
    size_t float_size;
    size_t double_offset;
    size_t total;
    size_t alignment = RLS_WORKSPACE_ALIGNMENT;

    /* 先限制维度，后续平方和字节数计算才不会接受超范围配置。 */
    if ((dimension == 0U) || (dimension > RLS_MAX_DIMENSION))
    {
        return RLS_ERROR_DIMENSION;
    }

    square = (size_t)dimension * (size_t)dimension;
    float_count = (3U * square) + (5U * (size_t)dimension);
    double_count = square + (size_t)dimension;
    /* 乘法前检查最大值，避免小尺寸类型上的字节数回绕。 */
    if ((float_count > (SIZE_MAX / sizeof(float))) ||
        (double_count > (SIZE_MAX / sizeof(double))))
    {
        return RLS_ERROR_DIMENSION;
    }

    float_size = float_count * sizeof(float);
    double_offset = float_size;
    if ((double_offset % alignment) != 0U)
    {
        double_offset += alignment - (double_offset % alignment);
    }
    if ((double_offset > SIZE_MAX - (double_count * sizeof(double))))
    {
        return RLS_ERROR_DIMENSION;
    }
    total = double_offset + (double_count * sizeof(double));

    *bytes = total;
    *float_bytes = float_size;
    return RLS_STATUS_OK;
}

static int RLSRangeEnd(const void *address, size_t bytes, uintptr_t *end)
{
    uintptr_t start;

    if ((address == NULL) || (bytes == 0U) || (end == NULL))
    {
        return 0;
    }

    start = (uintptr_t)address;
    if (bytes > (size_t)(UINTPTR_MAX - start))
    {
        return 0;
    }

    *end = start + (uintptr_t)bytes;
    return 1;
}

static RLSStatus_t RLSRangesOverlap(const void *first, size_t first_bytes,
                                   const void *second, size_t second_bytes,
                                   int *overlap)
{
    uintptr_t first_end;
    uintptr_t second_end;
    uintptr_t first_start;
    uintptr_t second_start;

    if ((overlap == NULL) ||
        !RLSRangeEnd(first, first_bytes, &first_end) ||
        !RLSRangeEnd(second, second_bytes, &second_end))
    {
        return RLS_ERROR_NULL;
    }

    first_start = (uintptr_t)first;
    second_start = (uintptr_t)second;
    *overlap = (first_start < second_end) && (second_start < first_end);
    return RLS_STATUS_OK;
}

static RLSStatus_t RLSCheckNoOverlap(const void *first, size_t first_bytes,
                                     const void *second, size_t second_bytes)
{
    int overlap;
    RLSStatus_t status = RLSRangesOverlap(first, first_bytes, second,
                                          second_bytes, &overlap);

    if (status != RLS_STATUS_OK)
    {
        return status;
    }
    return overlap ? RLS_ERROR_OVERLAP : RLS_STATUS_OK;
}

static void RLSBuildWorkspaceLayout(void *workspace, uint16_t dimension,
                                    size_t float_bytes,
                                    RLSWorkspaceLayout_t *layout)
{
    unsigned char *cursor = (unsigned char *)workspace;
    size_t alignment = RLS_WORKSPACE_ALIGNMENT;
    size_t double_offset = float_bytes;
    size_t square = (size_t)dimension * (size_t)dimension;

    layout->covariance = (float *)cursor;
    cursor += square * sizeof(float);
    layout->covariance_candidate = (float *)cursor;
    cursor += square * sizeof(float);
    layout->outer_product = (float *)cursor;
    cursor += square * sizeof(float);
    layout->params = (float *)cursor;
    cursor += (size_t)dimension * sizeof(float);
    layout->params_candidate = (float *)cursor;
    cursor += (size_t)dimension * sizeof(float);
    layout->gain = (float *)cursor;
    cursor += (size_t)dimension * sizeof(float);
    layout->px = (float *)cursor;
    cursor += (size_t)dimension * sizeof(float);
    layout->row = (float *)cursor;

    if ((double_offset % alignment) != 0U)
    {
        double_offset += alignment - (double_offset % alignment);
    }
    cursor = (unsigned char *)workspace + double_offset;
    layout->ldl = (double *)cursor;
    layout->diagonal = layout->ldl + square;
}

static int RLSAllFinite(const float *values, size_t count)
{
    size_t index;

    for (index = 0U; index < count; ++index)
    {
        if (!isfinite(values[index]))
        {
            return 0;
        }
    }
    return 1;
}

static RLSStatus_t RLSValidateInitialized(const RLS_t *rls,
                                          size_t *workspace_bytes,
                                          size_t *float_bytes)
{
    RLSStatus_t status;
    int overlap;

    if (rls == NULL)
    {
        return RLS_ERROR_NULL;
    }
    if ((rls->_magic != RLS_INSTANCE_MAGIC) ||
        (rls->_dimension == 0U) ||
        (rls->_dimension > RLS_MAX_DIMENSION) ||
        !isfinite(rls->_delta) || (rls->_delta <= 0.0f) ||
        !isfinite(rls->_lambda) || (rls->_lambda <= 0.0f) ||
        (rls->_lambda > 1.0f) || (rls->_workspace == NULL))
    {
        return RLS_ERROR_STATE;
    }

    status = RLSCalculateWorkspaceSize(rls->_dimension, workspace_bytes,
                                       float_bytes);
    if (status != RLS_STATUS_OK)
    {
        return RLS_ERROR_STATE;
    }
    if (rls->_workspace_bytes < *workspace_bytes)
    {
        return RLS_ERROR_STATE;
    }
    if (((uintptr_t)rls->_workspace % RLS_WORKSPACE_ALIGNMENT) != 0U)
    {
        return RLS_ERROR_ALIGNMENT;
    }
    status = RLSRangesOverlap(rls, sizeof(*rls), rls->_workspace,
                              *workspace_bytes, &overlap);
    if ((status != RLS_STATUS_OK) || overlap)
    {
        return RLS_ERROR_STATE;
    }
    return RLS_STATUS_OK;
}

static RLSStatus_t RLSCheckPositiveDefinite(const float *matrix,
                                            uint16_t dimension,
                                            RLSWorkspaceLayout_t *layout)
{
    size_t square = (size_t)dimension * (size_t)dimension;
    uint16_t row;
    uint16_t column;
    uint16_t previous;

    for (size_t index = 0U; index < square; ++index)
    {
        layout->ldl[index] = 0.0;
    }

    /* LDLᵀ 的每个对角主元都必须有限且大于0，才能保留协方差。 */
    for (row = 0U; row < dimension; ++row)
    {
        double pivot = (double)matrix[(size_t)row * dimension + row];
        for (previous = 0U; previous < row; ++previous)
        {
            double lower = layout->ldl[(size_t)row * dimension + previous];
            pivot -= lower * lower * layout->diagonal[previous];
        }
        if (!isfinite(pivot) || (pivot <= 0.0))
        {
            return RLS_ERROR_NUMERIC;
        }
        layout->diagonal[row] = pivot;
        layout->ldl[(size_t)row * dimension + row] = 1.0;

        for (column = (uint16_t)(row + 1U); column < dimension; ++column)
        {
            double value = (double)matrix[(size_t)column * dimension + row];
            for (previous = 0U; previous < row; ++previous)
            {
                value -= layout->ldl[(size_t)column * dimension + previous] *
                         layout->ldl[(size_t)row * dimension + previous] *
                         layout->diagonal[previous];
            }
            value /= pivot;
            if (!isfinite(value))
            {
                return RLS_ERROR_NUMERIC;
            }
            layout->ldl[(size_t)column * dimension + row] = value;
        }
    }
    return RLS_STATUS_OK;
}

static void RLSResetWorkspace(RLS_t *rls,
                              const float *initial_params,
                              RLSWorkspaceLayout_t *layout)
{
    size_t dimension = rls->_dimension;
    size_t square = dimension * dimension;
    size_t index;

    /* 先清整张矩阵，重置后旧的非对角值不会留在新状态里。 */
    memset(layout->covariance, 0, square * sizeof(float));
    memset(layout->covariance_candidate, 0, square * sizeof(float));
    memset(layout->outer_product, 0, square * sizeof(float));
    memset(layout->gain, 0, dimension * sizeof(float));
    memset(layout->px, 0, dimension * sizeof(float));
    memset(layout->row, 0, dimension * sizeof(float));
    for (index = 0U; index < dimension; ++index)
    {
        layout->params[index] = initial_params[index];
        layout->params_candidate[index] = initial_params[index];
        layout->covariance[index * dimension + index] = rls->_delta;
    }

    rls->_successful_update_count = 0U;
    rls->_last_prediction = 0.0f;
    rls->_last_residual = 0.0f;
    rls->_has_successful_update = 0U;
}

RLSStatus_t RLSGetWorkspaceRequirements(uint16_t dimension, size_t *bytes,
                                        size_t *alignment)
{
    RLSStatus_t status;
    size_t required_bytes;
    size_t float_bytes;

    if ((bytes == NULL) || (alignment == NULL))
    {
        return RLS_ERROR_NULL;
    }
    if (((uintptr_t)bytes % _Alignof(size_t)) != 0U ||
        ((uintptr_t)alignment % _Alignof(size_t)) != 0U)
    {
        return RLS_ERROR_ALIGNMENT;
    }
    status = RLSCheckNoOverlap(bytes, sizeof(*bytes), alignment,
                               sizeof(*alignment));
    if (status != RLS_STATUS_OK)
    {
        return status;
    }
    status = RLSCalculateWorkspaceSize(dimension, &required_bytes,
                                       &float_bytes);
    if (status != RLS_STATUS_OK)
    {
        return status;
    }

    /* 查询失败已在写输出前返回，因此两个输出总是一起更新。 */
    *bytes = required_bytes;
    *alignment = RLS_WORKSPACE_ALIGNMENT;
    return RLS_STATUS_OK;
}

RLSStatus_t RLSInit(RLS_t *rls, const RLSConfig_t *config,
                    const float *initial_params, size_t initial_count,
                    void *workspace, size_t workspace_bytes)
{
    RLSStatus_t status;
    size_t required_bytes;
    size_t float_bytes;
    size_t parameter_bytes;
    uint16_t dimension;
    float delta;
    float lambda;
    RLSWorkspaceLayout_t layout;

    if ((rls == NULL) || (config == NULL) || (initial_params == NULL) ||
        (workspace == NULL))
    {
        return RLS_ERROR_NULL;
    }
    if (((uintptr_t)rls % _Alignof(RLS_t)) != 0U ||
        ((uintptr_t)config % _Alignof(RLSConfig_t)) != 0U ||
        ((uintptr_t)initial_params % _Alignof(float)) != 0U)
    {
        return RLS_ERROR_ALIGNMENT;
    }

    /* 先复制配置，后续检查和写工作区都使用本地值。 */
    dimension = config->dimension;
    delta = config->delta;
    lambda = config->lambda;
    if ((dimension == 0U) || (dimension > RLS_MAX_DIMENSION))
    {
        return RLS_ERROR_DIMENSION;
    }
    if (!isfinite(delta) || (delta <= 0.0f) ||
        !isfinite(lambda) || (lambda <= 0.0f) || (lambda > 1.0f))
    {
        return RLS_ERROR_CONFIG;
    }
    if (initial_count != (size_t)dimension)
    {
        return RLS_ERROR_LENGTH;
    }
    status = RLSCalculateWorkspaceSize(dimension, &required_bytes,
                                       &float_bytes);
    if (status != RLS_STATUS_OK)
    {
        return status;
    }
    if (workspace_bytes < required_bytes)
    {
        return RLS_ERROR_CAPACITY;
    }
    if (((uintptr_t)workspace % RLS_WORKSPACE_ALIGNMENT) != 0U)
    {
        return RLS_ERROR_ALIGNMENT;
    }
    parameter_bytes = (size_t)dimension * sizeof(float);
    if (!RLSAllFinite(initial_params, dimension))
    {
        return RLS_ERROR_NUMERIC;
    }

    status = RLSCheckNoOverlap(rls, sizeof(*rls), workspace,
                               required_bytes);
    if (status != RLS_STATUS_OK)
    {
        return status;
    }
    status = RLSCheckNoOverlap(rls, sizeof(*rls), initial_params,
                               parameter_bytes);
    if (status != RLS_STATUS_OK)
    {
        return status;
    }
    status = RLSCheckNoOverlap(workspace, required_bytes, initial_params,
                               parameter_bytes);
    if (status != RLS_STATUS_OK)
    {
        return status;
    }
    status = RLSCheckNoOverlap(rls, sizeof(*rls), config, sizeof(*config));
    if (status != RLS_STATUS_OK)
    {
        return status;
    }
    status = RLSCheckNoOverlap(workspace, required_bytes, config,
                               sizeof(*config));
    if (status != RLS_STATUS_OK)
    {
        return status;
    }
    status = RLSCheckNoOverlap(initial_params, parameter_bytes, config,
                               sizeof(*config));
    if (status != RLS_STATUS_OK)
    {
        return status;
    }

    /* 所有拒绝条件都已检查；从这里开始才接管工作区并更新实例。 */
    memset(rls, 0, sizeof(*rls));
    rls->_magic = RLS_INSTANCE_MAGIC;
    rls->_dimension = dimension;
    rls->_delta = delta;
    rls->_lambda = lambda;
    rls->_workspace = workspace;
    rls->_workspace_bytes = required_bytes;
    RLSBuildWorkspaceLayout(workspace, dimension, float_bytes, &layout);
    RLSResetWorkspace(rls, initial_params, &layout);
    return RLS_STATUS_OK;
}

RLSStatus_t RLSReset(RLS_t *rls, const float *initial_params,
                     size_t initial_count)
{
    RLSStatus_t status;
    size_t workspace_bytes;
    size_t float_bytes;
    size_t parameter_bytes;
    RLSWorkspaceLayout_t layout;

    if (initial_params == NULL)
    {
        return RLS_ERROR_NULL;
    }
    status = RLSValidateInitialized(rls, &workspace_bytes, &float_bytes);
    if (status != RLS_STATUS_OK)
    {
        return status;
    }
    if (((uintptr_t)initial_params % _Alignof(float)) != 0U)
    {
        return RLS_ERROR_ALIGNMENT;
    }
    if (initial_count != (size_t)rls->_dimension)
    {
        return RLS_ERROR_LENGTH;
    }
    parameter_bytes = (size_t)rls->_dimension * sizeof(float);
    if (!RLSAllFinite(initial_params, rls->_dimension))
    {
        return RLS_ERROR_NUMERIC;
    }
    status = RLSCheckNoOverlap(rls, sizeof(*rls), initial_params,
                               parameter_bytes);
    if (status != RLS_STATUS_OK)
    {
        return status;
    }
    status = RLSCheckNoOverlap(rls->_workspace, workspace_bytes,
                               initial_params, parameter_bytes);
    if (status != RLS_STATUS_OK)
    {
        return status;
    }

    RLSBuildWorkspaceLayout(rls->_workspace, rls->_dimension, float_bytes,
                            &layout);
    RLSResetWorkspace(rls, initial_params, &layout);
    return RLS_STATUS_OK;
}

RLSStatus_t RLSUpdate(RLS_t *rls, const float *input, size_t input_count,
                      float actual_output)
{
    RLSStatus_t status;
    size_t workspace_bytes;
    size_t float_bytes;
    size_t dimension;
    size_t square;
    size_t index;
    size_t row;
    size_t column;
    size_t used_input_bytes;
    float denominator;
    float residual;
    float prediction;
    float q;
    float scalar_result = 0.0f;
    RLSWorkspaceLayout_t layout;
    arm_matrix_instance_f32 covariance_matrix;
    arm_matrix_instance_f32 input_column;
    arm_matrix_instance_f32 input_row;
    arm_matrix_instance_f32 params_column;
    arm_matrix_instance_f32 px_column;
    arm_matrix_instance_f32 row_matrix;
    arm_matrix_instance_f32 scalar_matrix;
    arm_matrix_instance_f32 outer_matrix;
    arm_matrix_instance_f32 candidate_matrix;

    status = RLSValidateInitialized(rls, &workspace_bytes, &float_bytes);
    if (status != RLS_STATUS_OK)
    {
        return status;
    }
    if (input == NULL)
    {
        return RLS_ERROR_NULL;
    }
    if (((uintptr_t)input % _Alignof(float)) != 0U)
    {
        return RLS_ERROR_ALIGNMENT;
    }
    if (input_count != (size_t)rls->_dimension)
    {
        return RLS_ERROR_LENGTH;
    }
    if (!isfinite(actual_output))
    {
        return RLS_ERROR_NUMERIC;
    }
    dimension = rls->_dimension;
    square = dimension * dimension;
    used_input_bytes = dimension * sizeof(float);
    if (!RLSAllFinite(input, dimension))
    {
        return RLS_ERROR_NUMERIC;
    }
    status = RLSCheckNoOverlap(rls, sizeof(*rls), input, used_input_bytes);
    if (status != RLS_STATUS_OK)
    {
        return status;
    }
    status = RLSCheckNoOverlap(rls->_workspace, workspace_bytes, input,
                               used_input_bytes);
    if (status != RLS_STATUS_OK)
    {
        return status;
    }
    if (rls->_successful_update_count == SIZE_MAX)
    {
        return RLS_ERROR_NUMERIC;
    }

    RLSBuildWorkspaceLayout(rls->_workspace, rls->_dimension, float_bytes,
                            &layout);
    covariance_matrix.numRows = rls->_dimension;
    covariance_matrix.numCols = rls->_dimension;
    covariance_matrix.pData = layout.covariance;
    input_column.numRows = rls->_dimension;
    input_column.numCols = 1U;
    input_column.pData = (float *)input;
    input_row.numRows = 1U;
    input_row.numCols = rls->_dimension;
    input_row.pData = (float *)input;
    params_column.numRows = rls->_dimension;
    params_column.numCols = 1U;
    params_column.pData = layout.params;
    px_column.numRows = rls->_dimension;
    px_column.numCols = 1U;
    px_column.pData = layout.px;
    row_matrix.numRows = 1U;
    row_matrix.numCols = rls->_dimension;
    row_matrix.pData = layout.row;
    scalar_matrix.numRows = 1U;
    scalar_matrix.numCols = 1U;
    scalar_matrix.pData = &scalar_result;
    outer_matrix.numRows = rls->_dimension;
    outer_matrix.numCols = rls->_dimension;
    outer_matrix.pData = layout.outer_product;
    candidate_matrix.numRows = rls->_dimension;
    candidate_matrix.numCols = rls->_dimension;
    candidate_matrix.pData = layout.covariance_candidate;

    /* 先按旧P求P*x和xᵀP；任一DSP步骤失败都只影响临时数组。 */
    if (arm_mat_mult_f32(&covariance_matrix, &input_column, &px_column) !=
        ARM_MATH_SUCCESS)
    {
        return RLS_ERROR_MATRIX;
    }
    if (arm_mat_mult_f32(&input_row, &covariance_matrix, &row_matrix) !=
        ARM_MATH_SUCCESS)
    {
        return RLS_ERROR_MATRIX;
    }
    if (arm_mat_mult_f32(&input_row, &px_column, &scalar_matrix) !=
        ARM_MATH_SUCCESS)
    {
        return RLS_ERROR_MATRIX;
    }
    q = scalar_result;
    if (!isfinite(q))
    {
        return RLS_ERROR_NUMERIC;
    }
    /* 正分母来自遗忘因子和输入方向上的协方差权重。 */
    denominator = rls->_lambda + q;
    if (!isfinite(denominator) || (denominator <= 0.0f))
    {
        return RLS_ERROR_NUMERIC;
    }
    for (index = 0U; index < dimension; ++index)
    {
        layout.px[index] /= denominator;
    }
    if (!RLSAllFinite(layout.px, dimension))
    {
        return RLS_ERROR_NUMERIC;
    }

    /* 预测使用旧参数，残差再与未乘残差的K相乘更新参数。 */
    scalar_result = 0.0f;
    if (arm_mat_mult_f32(&input_row, &params_column, &scalar_matrix) !=
        ARM_MATH_SUCCESS)
    {
        return RLS_ERROR_MATRIX;
    }
    prediction = scalar_result;
    if (!isfinite(prediction))
    {
        return RLS_ERROR_NUMERIC;
    }
    residual = actual_output - prediction;
    if (!isfinite(residual))
    {
        return RLS_ERROR_NUMERIC;
    }

    for (index = 0U; index < dimension; ++index)
    {
        layout.params_candidate[index] =
            layout.params[index] + (layout.px[index] * residual);
    }
    if (!RLSAllFinite(layout.params_candidate, dimension))
    {
        return RLS_ERROR_NUMERIC;
    }

    /* 协方差公式使用未缩放的K，零残差时P仍按lambda更新。 */
    if (arm_mat_mult_f32(&px_column, &row_matrix, &outer_matrix) !=
        ARM_MATH_SUCCESS)
    {
        return RLS_ERROR_MATRIX;
    }
    if (arm_mat_sub_f32(&covariance_matrix, &outer_matrix,
                        &candidate_matrix) != ARM_MATH_SUCCESS)
    {
        return RLS_ERROR_MATRIX;
    }
    for (index = 0U; index < square; ++index)
    {
        layout.covariance_candidate[index] /= rls->_lambda;
    }
    if (!RLSAllFinite(layout.covariance_candidate, square))
    {
        return RLS_ERROR_NUMERIC;
    }

    /* 舍入可能让上下三角略有差异，取平均后再检查整张候选协方差。 */
    for (row = 0U; row < dimension; ++row)
    {
        for (column = row + 1U; column < dimension; ++column)
        {
            size_t upper_index = row * dimension + column;
            size_t lower_index = column * dimension + row;
            float symmetric = (float)(((double)layout.covariance_candidate[upper_index] +
                                       (double)layout.covariance_candidate[lower_index]) *
                                      0.5);
            if (!isfinite(symmetric))
            {
                return RLS_ERROR_NUMERIC;
            }
            layout.covariance_candidate[upper_index] = symmetric;
            layout.covariance_candidate[lower_index] = symmetric;
        }
    }
    status = RLSCheckPositiveDefinite(layout.covariance_candidate,
                                      rls->_dimension, &layout);
    if (status != RLS_STATUS_OK)
    {
        return status;
    }

    /* 只有参数和协方差都通过检查后，才替换已提交结果。 */
    memcpy(layout.params, layout.params_candidate,
           dimension * sizeof(float));
    memcpy(layout.covariance, layout.covariance_candidate,
           square * sizeof(float));
    memcpy(layout.gain, layout.px, dimension * sizeof(float));
    rls->_last_prediction = prediction;
    rls->_last_residual = residual;
    rls->_successful_update_count++;
    rls->_has_successful_update = 1U;
    return RLS_STATUS_OK;
}

RLSStatus_t RLSGetState(const RLS_t *rls, float *params,
                        size_t params_capacity, float *covariance,
                        size_t covariance_capacity, float *gain,
                        size_t gain_capacity, RLSStateInfo_t *info)
{
    RLSStatus_t status;
    size_t workspace_bytes;
    size_t float_bytes;
    size_t dimension;
    size_t square;
    size_t parameter_bytes;
    size_t covariance_bytes;
    RLSWorkspaceLayout_t layout;
    RLSStateInfo_t candidate_info;

    if ((params == NULL) || (covariance == NULL) || (gain == NULL) ||
        (info == NULL))
    {
        return RLS_ERROR_NULL;
    }
    status = RLSValidateInitialized(rls, &workspace_bytes, &float_bytes);
    if (status != RLS_STATUS_OK)
    {
        return status;
    }
    dimension = rls->_dimension;
    square = dimension * dimension;
    if ((params_capacity < dimension) || (covariance_capacity < square) ||
        (gain_capacity < dimension))
    {
        return RLS_ERROR_CAPACITY;
    }
    if (((uintptr_t)params % _Alignof(float)) != 0U ||
        ((uintptr_t)covariance % _Alignof(float)) != 0U ||
        ((uintptr_t)gain % _Alignof(float)) != 0U ||
        ((uintptr_t)info % _Alignof(RLSStateInfo_t)) != 0U)
    {
        return RLS_ERROR_ALIGNMENT;
    }
    parameter_bytes = dimension * sizeof(float);
    covariance_bytes = square * sizeof(float);
    status = RLSCheckNoOverlap(rls, sizeof(*rls), params, parameter_bytes);
    if (status != RLS_STATUS_OK)
    {
        return status;
    }
    status = RLSCheckNoOverlap(rls, sizeof(*rls), covariance,
                               covariance_bytes);
    if (status != RLS_STATUS_OK)
    {
        return status;
    }
    status = RLSCheckNoOverlap(rls, sizeof(*rls), gain, parameter_bytes);
    if (status != RLS_STATUS_OK)
    {
        return status;
    }
    status = RLSCheckNoOverlap(rls, sizeof(*rls), info, sizeof(*info));
    if (status != RLS_STATUS_OK)
    {
        return status;
    }
    status = RLSCheckNoOverlap(rls->_workspace, workspace_bytes, params,
                               parameter_bytes);
    if (status != RLS_STATUS_OK)
    {
        return status;
    }
    status = RLSCheckNoOverlap(rls->_workspace, workspace_bytes, covariance,
                               covariance_bytes);
    if (status != RLS_STATUS_OK)
    {
        return status;
    }
    status = RLSCheckNoOverlap(rls->_workspace, workspace_bytes, gain,
                               parameter_bytes);
    if (status != RLS_STATUS_OK)
    {
        return status;
    }
    status = RLSCheckNoOverlap(rls->_workspace, workspace_bytes, info,
                               sizeof(*info));
    if (status != RLS_STATUS_OK)
    {
        return status;
    }
    status = RLSCheckNoOverlap(params, parameter_bytes, covariance,
                               covariance_bytes);
    if (status != RLS_STATUS_OK)
    {
        return status;
    }
    status = RLSCheckNoOverlap(params, parameter_bytes, gain, parameter_bytes);
    if (status != RLS_STATUS_OK)
    {
        return status;
    }
    status = RLSCheckNoOverlap(covariance, covariance_bytes, gain,
                               parameter_bytes);
    if (status != RLS_STATUS_OK)
    {
        return status;
    }
    status = RLSCheckNoOverlap(info, sizeof(*info), params, parameter_bytes);
    if (status != RLS_STATUS_OK)
    {
        return status;
    }
    status = RLSCheckNoOverlap(info, sizeof(*info), covariance,
                               covariance_bytes);
    if (status != RLS_STATUS_OK)
    {
        return status;
    }
    status = RLSCheckNoOverlap(info, sizeof(*info), gain, parameter_bytes);
    if (status != RLS_STATUS_OK)
    {
        return status;
    }

    /* 容量、地址和别名检查完成后才开始复制，失败不会写一部分结果。 */
    RLSBuildWorkspaceLayout(rls->_workspace, rls->_dimension, float_bytes,
                            &layout);
    memset(&candidate_info, 0, sizeof(candidate_info));
    candidate_info.successful_update_count = rls->_successful_update_count;
    candidate_info.last_prediction = rls->_last_prediction;
    candidate_info.last_residual = rls->_last_residual;
    candidate_info.has_successful_update = rls->_has_successful_update;
    memcpy(params, layout.params, parameter_bytes);
    memcpy(covariance, layout.covariance, covariance_bytes);
    memcpy(gain, layout.gain, parameter_bytes);
    *info = candidate_info;
    return RLS_STATUS_OK;
}
