/**
 * @file rls_test.c
 * @brief 经公开 RLS 接口验证递推公式、工作区和失败保护。
 * 在主机运行；Release 也使用 TEST_CHECK，矩阵失败注入仅用于 Linux 链接器。
 */
#include "RLS.h"
#include "test_check.h"

#include "arm_math.h"

#include <math.h>
#include <stdint.h>
#include <string.h>

#if defined(RLS_TEST_THREADS)
#include <pthread.h>
#endif

#if defined(RLS_TEST_ALLOC_WRAP)
#include <stdatomic.h>

extern void *__real_malloc(size_t size);
extern void *__real_calloc(size_t count, size_t size);
extern void *__real_realloc(void *pointer, size_t size);
extern void __real_free(void *pointer);

static atomic_int g_track_allocations;
static atomic_uint g_allocation_calls;

void *__wrap_malloc(size_t size)
{
    if (atomic_load(&g_track_allocations) != 0)
    {
        (void)atomic_fetch_add(&g_allocation_calls, 1U);
    }
    return __real_malloc(size);
}

void *__wrap_calloc(size_t count, size_t size)
{
    if (atomic_load(&g_track_allocations) != 0)
    {
        (void)atomic_fetch_add(&g_allocation_calls, 1U);
    }
    return __real_calloc(count, size);
}

void *__wrap_realloc(void *pointer, size_t size)
{
    if (atomic_load(&g_track_allocations) != 0)
    {
        (void)atomic_fetch_add(&g_allocation_calls, 1U);
    }
    return __real_realloc(pointer, size);
}

void __wrap_free(void *pointer)
{
    if (atomic_load(&g_track_allocations) != 0)
    {
        (void)atomic_fetch_add(&g_allocation_calls, 1U);
    }
    __real_free(pointer);
}
#endif

#if defined(RLS_TEST_MATRIX_WRAP)
extern arm_status __real_arm_mat_mult_f32(
    const arm_matrix_instance_f32 *source_a,
    const arm_matrix_instance_f32 *source_b,
    arm_matrix_instance_f32 *destination);
extern arm_status __real_arm_mat_sub_f32(
    const arm_matrix_instance_f32 *source_a,
    const arm_matrix_instance_f32 *source_b,
    arm_matrix_instance_f32 *destination);

static int g_fail_mult_call;
static int g_mult_calls;
static int g_fail_sub_call;
static int g_sub_calls;
static int g_corrupt_candidate;

arm_status __wrap_arm_mat_mult_f32(
    const arm_matrix_instance_f32 *source_a,
    const arm_matrix_instance_f32 *source_b,
    arm_matrix_instance_f32 *destination)
{
    ++g_mult_calls;
    if (g_mult_calls == g_fail_mult_call)
    {
        return ARM_MATH_ARGUMENT_ERROR;
    }
    return __real_arm_mat_mult_f32(source_a, source_b, destination);
}

arm_status __wrap_arm_mat_sub_f32(
    const arm_matrix_instance_f32 *source_a,
    const arm_matrix_instance_f32 *source_b,
    arm_matrix_instance_f32 *destination)
{
    ++g_sub_calls;
    if (g_sub_calls == g_fail_sub_call)
    {
        return ARM_MATH_ARGUMENT_ERROR;
    }
    if (__real_arm_mat_sub_f32(source_a, source_b, destination) !=
        ARM_MATH_SUCCESS)
    {
        return ARM_MATH_ARGUMENT_ERROR;
    }
    if ((g_corrupt_candidate != 0) && (destination->numRows > 0U))
    {
        /* 把候选矩阵改成非正定值，验证它不能进入已提交状态。 */
        destination->pData[0] = -1.0f;
    }
    return ARM_MATH_SUCCESS;
}
#endif

typedef union
{
    double alignment;
    unsigned char bytes[RLS_WORKSPACE_CAPACITY(4U)];
} RLSWorkspace4_t;

typedef union
{
    double alignment;
    unsigned char bytes[RLS_WORKSPACE_CAPACITY(3U)];
} RLSWorkspace3_t;

typedef union
{
    double alignment;
    unsigned char bytes[RLS_WORKSPACE_CAPACITY(16U)];
} RLSWorkspace16_t;

typedef union
{
    double alignment;
    unsigned char bytes[RLS_WORKSPACE_CAPACITY(2U)];
} RLSWorkspace2_t;

static _Alignas(double) unsigned char g_workspace255[RLS_WORKSPACE_CAPACITY(255U)];
static float g_params[255U];
static float g_covariance[255U * 255U];
static float g_gain[255U];
static float g_saved_params[255U];
static float g_saved_covariance[255U * 255U];
static float g_saved_gain[255U];

#if defined(RLS_TEST_THREADS)
typedef struct
{
    RLS_t *rls;
    const float *input;
    uint16_t dimension;
    float output;
    RLSStatus_t status;
} RLSThreadArgs_t;

static void *RLSWorker(void *opaque)
{
    RLSThreadArgs_t *args = (RLSThreadArgs_t *)opaque;
    size_t iteration;
    for (iteration = 0U; iteration < 2000U; ++iteration)
    {
        args->status = RLSUpdate(args->rls, args->input,
                                 args->dimension, args->output);
        if (args->status != RLS_STATUS_OK)
        {
            break;
        }
    }
    return NULL;
}
#endif

static int RLSNear(double actual, double expected)
{
    double tolerance = 5.0e-5 + (5.0e-4 * fabs(expected));
    return isfinite(actual) && (fabs(actual - expected) <= tolerance);
}

static void RLSGetStateOrFail(const RLS_t *rls, uint16_t dimension,
                              float *params, float *covariance, float *gain,
                              RLSStateInfo_t *info)
{
    TEST_CHECK(RLSGetState(rls, params, dimension, covariance,
                           (size_t)dimension * dimension, gain, dimension,
                           info) == RLS_STATUS_OK);
}

static void RLSCompareState(const RLS_t *rls, uint16_t dimension,
                            const float *params, const float *covariance,
                            const float *gain, const RLSStateInfo_t *info)
{
    RLSStateInfo_t actual_info;
    size_t square = (size_t)dimension * dimension;
    size_t index;

    RLSGetStateOrFail(rls, dimension, g_params, g_covariance, g_gain,
                      &actual_info);
    for (index = 0U; index < dimension; ++index)
    {
        TEST_CHECK(memcmp(&g_params[index], &params[index], sizeof(float)) == 0);
        TEST_CHECK(memcmp(&g_gain[index], &gain[index], sizeof(float)) == 0);
    }
    for (index = 0U; index < square; ++index)
    {
        TEST_CHECK(memcmp(&g_covariance[index], &covariance[index],
                          sizeof(float)) == 0);
    }
    TEST_CHECK(actual_info.successful_update_count == info->successful_update_count);
    TEST_CHECK(actual_info.last_prediction == info->last_prediction);
    TEST_CHECK(actual_info.last_residual == info->last_residual);
    TEST_CHECK(actual_info.has_successful_update == info->has_successful_update);
}

static void RLSReferenceQr(size_t sample_count, uint16_t dimension,
                           const float *initial_params, float delta,
                           float lambda, const float *samples,
                           const float *outputs, double *params,
                           double *covariance)
{
    double matrix[32U][4U] = {{0.0}};
    double vector[32U] = {0.0};
    double inverse_r[3U][3U] = {{0.0}};
    double upper[3U][3U] = {{0.0}};
    double rhs[3U] = {0.0};
    size_t rows = (size_t)dimension + sample_count;
    size_t row;
    uint16_t column;
    uint16_t pivot;

    TEST_CHECK(dimension <= 3U);
    TEST_CHECK(rows <= 32U);

    /* 初始协方差 delta*I 对应带权最小二乘中的先验行。 */
    {
        double prior_scale = sqrt(pow((double)lambda, (double)sample_count) /
                                  (double)delta);
        for (column = 0U; column < dimension; ++column)
        {
            matrix[column][column] = prior_scale;
            matrix[column][dimension] = prior_scale * initial_params[column];
        }
    }
    for (row = 0U; row < sample_count; ++row)
    {
        double weight = pow((double)lambda,
                            (double)(sample_count - 1U - row));
        double scale = sqrt(weight);
        for (column = 0U; column < dimension; ++column)
        {
            matrix[(size_t)dimension + row][column] =
                scale * samples[row * dimension + column];
        }
        matrix[(size_t)dimension + row][dimension] = scale * outputs[row];
    }

    /* Householder QR 同时变换右侧输出，避免用 RLS 递推式当参考答案。 */
    for (pivot = 0U; pivot < dimension; ++pivot)
    {
        double norm_squared = 0.0;
        double norm;
        double alpha;
        double denominator = 0.0;
        for (row = pivot; row < rows; ++row)
        {
            norm_squared += matrix[row][pivot] * matrix[row][pivot];
        }
        norm = sqrt(norm_squared);
        alpha = (matrix[pivot][pivot] >= 0.0) ? -norm : norm;
        for (row = pivot; row < rows; ++row)
        {
            vector[row] = matrix[row][pivot];
        }
        vector[pivot] -= alpha;
        for (row = pivot; row < rows; ++row)
        {
            denominator += vector[row] * vector[row];
        }
        TEST_CHECK(denominator > 0.0);
        for (column = pivot; column <= dimension; ++column)
        {
            double projection = 0.0;
            for (row = pivot; row < rows; ++row)
            {
                projection += vector[row] * matrix[row][column];
            }
            projection *= 2.0 / denominator;
            for (row = pivot; row < rows; ++row)
            {
                matrix[row][column] -= projection * vector[row];
            }
        }
        matrix[pivot][pivot] = alpha;
        for (row = (size_t)pivot + 1U; row < rows; ++row)
        {
            matrix[row][pivot] = 0.0;
        }
    }

    for (pivot = 0U; pivot < dimension; ++pivot)
    {
        for (column = 0U; column < dimension; ++column)
        {
            upper[pivot][column] = matrix[pivot][column];
        }
        rhs[pivot] = matrix[pivot][dimension];
    }
    for (pivot = dimension; pivot-- > 0U;)
    {
        double value = rhs[pivot];
        for (column = (uint16_t)(pivot + 1U); column < dimension; ++column)
        {
            value -= upper[pivot][column] * params[column];
        }
        params[pivot] = value / upper[pivot][pivot];
    }

    for (column = 0U; column < dimension; ++column)
    {
        int row_index;
        for (row_index = (int)dimension - 1; row_index >= 0; --row_index)
        {
            double value = ((uint16_t)row_index == column) ? 1.0 : 0.0;
            uint16_t next;
            for (next = (uint16_t)(row_index + 1); next < dimension; ++next)
            {
                value -= upper[row_index][next] *
                         inverse_r[next][column];
            }
            inverse_r[row_index][column] = value / upper[row_index][row_index];
        }
    }
    for (row = 0U; row < dimension; ++row)
    {
        for (column = 0U; column < dimension; ++column)
        {
            uint16_t inner;
            double value = 0.0;
            for (inner = 0U; inner < dimension; ++inner)
            {
                value += inverse_r[row][inner] * inverse_r[column][inner];
            }
            covariance[row * dimension + column] = value;
        }
    }
}

static void RLSCheckScalarAndReference(void)
{
    const RLSConfig_t scalar_config = {1U, 1.0f, 1.0f};
    const float zero_initial[1] = {0.0f};
    const float one_input[1] = {1.0f};
    const float zero_input[1] = {0.0f};
    RLS_t scalar_rls = {0};
    RLSWorkspace2_t scalar_workspace = {0};
    float params[1];
    float covariance[1];
    float gain[1];
    RLSStateInfo_t info;

    TEST_CHECK(RLSInit(&scalar_rls, &scalar_config, zero_initial, 1U,
                       scalar_workspace.bytes,
                       sizeof(scalar_workspace.bytes)) == RLS_STATUS_OK);
    TEST_CHECK(RLSUpdate(&scalar_rls, one_input, 1U, 0.0f) == RLS_STATUS_OK);
    RLSGetStateOrFail(&scalar_rls, 1U, params, covariance, gain, &info);
    TEST_CHECK(params[0] == 0.0f);
    TEST_CHECK(RLSNear(gain[0], 0.5));
    TEST_CHECK(RLSNear(covariance[0], 0.5));
    TEST_CHECK(info.has_successful_update == 1U);
    TEST_CHECK(info.successful_update_count == 1U);
    TEST_CHECK(info.last_prediction == 0.0f);
    TEST_CHECK(info.last_residual == 0.0f);

    /* 零输入仍是合法样本；它不改参数和增益，但遗忘因子仍作用于P。 */
    TEST_CHECK(RLSUpdate(&scalar_rls, zero_input, 1U, 2.0f) == RLS_STATUS_OK);
    RLSGetStateOrFail(&scalar_rls, 1U, params, covariance, gain, &info);
    TEST_CHECK(params[0] == 0.0f);
    TEST_CHECK(gain[0] == 0.0f);
    TEST_CHECK(RLSNear(covariance[0], 0.5));
    TEST_CHECK(info.successful_update_count == 2U);
    TEST_CHECK(info.last_residual == 2.0f);

    {
        const RLSConfig_t decaying_config = {1U, 1.0f, 0.5f};
        RLS_t decaying_rls = {0};
        RLSWorkspace2_t decaying_workspace = {0};
        TEST_CHECK(RLSInit(&decaying_rls, &decaying_config, zero_initial, 1U,
                           decaying_workspace.bytes,
                           sizeof(decaying_workspace.bytes)) == RLS_STATUS_OK);
        TEST_CHECK(RLSUpdate(&decaying_rls, zero_input, 1U, 2.0f) ==
                   RLS_STATUS_OK);
        RLSGetStateOrFail(&decaying_rls, 1U, params, covariance, gain, &info);
        TEST_CHECK(params[0] == 0.0f && gain[0] == 0.0f);
        TEST_CHECK(RLSNear(covariance[0], 2.0));
        TEST_CHECK(info.last_prediction == 0.0f && info.last_residual == 2.0f);
        TEST_CHECK(RLSUpdate(&decaying_rls, zero_input, 1U, 3.0f) ==
                   RLS_STATUS_OK);
        RLSGetStateOrFail(&decaying_rls, 1U, params, covariance, gain, &info);
        TEST_CHECK(RLSNear(covariance[0], 4.0));
        TEST_CHECK(info.successful_update_count == 2U);
    }

    {
        const uint16_t dimension = 3U;
        const float initial[3] = {0.4f, -0.8f, 1.2f};
        const RLSConfig_t config = {3U, 2.5f, 0.97f};
        const float samples[30] = {
            1.0f, 0.0f, 1.0f, 0.5f, -1.0f, 0.2f, -0.3f, 0.7f, 1.0f,
            1.2f, 0.4f, -0.8f, -1.0f, -0.2f, 0.3f, 0.3f, 1.1f, -0.4f,
            0.8f, -0.6f, 0.5f, -0.7f, 0.9f, 1.3f, 0.2f, 0.1f, -0.9f,
            1.1f, -0.5f, 0.6f};
        const float outputs[10] = {
            2.72f, 1.04f, 1.49f, 2.52f, -0.22f,
            0.89f, 2.76f, 2.03f, -1.37f, 2.35f};
        double reference_params[3] = {0.0};
        double previous_params[3] = {0.0};
        double reference_covariance[9] = {0.0};
        double previous_covariance[9] = {0.0};
        double expected_gain[3];
        size_t sample;
        RLS_t rls = {0};
        RLSWorkspace3_t workspace = {0};
        float actual_params[3];
        float actual_covariance[9];
        float actual_gain[3];

        TEST_CHECK(RLSInit(&rls, &config, initial, dimension,
                           workspace.bytes, sizeof(workspace.bytes)) ==
                   RLS_STATUS_OK);
        for (sample = 0U; sample < 10U; ++sample)
        {
            double denominator;
            double prediction = 0.0;
            double residual;
            uint16_t index;

            RLSReferenceQr(sample, dimension, initial, config.delta,
                           config.lambda, samples, outputs,
                           previous_params, previous_covariance);
            denominator = config.lambda;
            for (index = 0U; index < dimension; ++index)
            {
                double px = 0.0;
                uint16_t inner;
                prediction += samples[sample * dimension + index] *
                              previous_params[index];
                for (inner = 0U; inner < dimension; ++inner)
                {
                    px += previous_covariance[index * dimension + inner] *
                          samples[sample * dimension + inner];
                }
                expected_gain[index] = px;
            }
            for (index = 0U; index < dimension; ++index)
            {
                denominator += samples[sample * dimension + index] *
                               expected_gain[index];
            }
            for (index = 0U; index < dimension; ++index)
            {
                expected_gain[index] /= denominator;
            }
            residual = outputs[sample] - prediction;
            TEST_CHECK(RLSUpdate(&rls, &samples[sample * dimension], dimension,
                                 outputs[sample]) == RLS_STATUS_OK);
            RLSReferenceQr(sample + 1U, dimension, initial, config.delta,
                           config.lambda, samples, outputs,
                           reference_params, reference_covariance);
            {
                RLSStateInfo_t state_info;
                RLSGetStateOrFail(&rls, dimension, actual_params,
                                  actual_covariance, actual_gain, &state_info);
                for (index = 0U; index < dimension; ++index)
                {
                    TEST_CHECK(RLSNear(actual_params[index], reference_params[index]));
                    TEST_CHECK(RLSNear(actual_gain[index], expected_gain[index]));
                    TEST_CHECK(RLSNear(state_info.last_prediction, prediction));
                    TEST_CHECK(RLSNear(state_info.last_residual, residual));
                }
                for (index = 0U; index < dimension * dimension; ++index)
                {
                    TEST_CHECK(RLSNear(actual_covariance[index],
                                       reference_covariance[index]));
                }
                TEST_CHECK(state_info.successful_update_count == sample + 1U);
                TEST_CHECK(state_info.has_successful_update == 1U);
            }
            memcpy(previous_params, reference_params, sizeof(previous_params));
            memcpy(previous_covariance, reference_covariance,
                   sizeof(previous_covariance));
        }

        /* 重置必须清除旧的非对角协方差、增益和样本记录。 */
        {
            const float reset_params[3] = {-1.0f, 2.0f, 0.25f};
            RLSStateInfo_t state_info;
            TEST_CHECK(RLSReset(&rls, reset_params, dimension) == RLS_STATUS_OK);
            RLSGetStateOrFail(&rls, dimension, actual_params,
                              actual_covariance, actual_gain, &state_info);
            TEST_CHECK(memcmp(actual_params, reset_params,
                              sizeof(reset_params)) == 0);
            TEST_CHECK(actual_covariance[0] == config.delta);
            TEST_CHECK(actual_covariance[4] == config.delta);
            TEST_CHECK(actual_covariance[8] == config.delta);
            TEST_CHECK(actual_covariance[1] == 0.0f);
            TEST_CHECK(actual_covariance[2] == 0.0f);
            TEST_CHECK(actual_covariance[3] == 0.0f);
            TEST_CHECK(actual_covariance[5] == 0.0f);
            TEST_CHECK(actual_covariance[6] == 0.0f);
            TEST_CHECK(actual_covariance[7] == 0.0f);
            TEST_CHECK(actual_gain[0] == 0.0f && actual_gain[1] == 0.0f &&
                       actual_gain[2] == 0.0f);
            TEST_CHECK(state_info.has_successful_update == 0U);
            TEST_CHECK(state_info.successful_update_count == 0U);
            TEST_CHECK(state_info.last_prediction == 0.0f);
            TEST_CHECK(state_info.last_residual == 0.0f);
        }
    }
}

static void RLSCheckWorkspaceRules(void)
{
    size_t required_bytes = 17U;
    size_t alignment = 19U;
    size_t queried_bytes;
    size_t queried_alignment;
    size_t two_dimension_bytes;
    size_t two_dimension_alignment;
    RLSConfig_t config = {2U, 1.0f, 0.99f};
    float initial[2] = {0.0f, 0.0f};
    float input[2] = {0.0f, 0.0f};
    RLSStateInfo_t info;
    RLS_t rls = {0};
    RLSWorkspace4_t workspace = {0};
    _Alignas(double) unsigned char misaligned[RLS_WORKSPACE_CAPACITY(4U) + 1U];
    unsigned char saved_workspace[RLS_WORKSPACE_CAPACITY(4U)];
    float output_params[4] = {11.0f, 12.0f, 13.0f, 14.0f};
    float output_covariance[16] = {0.0f};
    float output_gain[4] = {21.0f, 22.0f, 23.0f, 24.0f};

    TEST_CHECK(RLSGetWorkspaceRequirements(0U, &required_bytes, &alignment) ==
               RLS_ERROR_DIMENSION);
    TEST_CHECK(required_bytes == 17U && alignment == 19U);
    TEST_CHECK(RLSGetWorkspaceRequirements(256U, &required_bytes,
                                           &alignment) == RLS_ERROR_DIMENSION);
    TEST_CHECK(RLSGetWorkspaceRequirements(4U, NULL, &alignment) ==
               RLS_ERROR_NULL);
    TEST_CHECK(RLSGetWorkspaceRequirements(4U, &required_bytes,
                                           &required_bytes) ==
               RLS_ERROR_OVERLAP);
    TEST_CHECK(RLSGetWorkspaceRequirements(4U, &queried_bytes,
                                           &queried_alignment) ==
               RLS_STATUS_OK);
    TEST_CHECK(queried_bytes <= sizeof(workspace.bytes));
    TEST_CHECK(queried_alignment == RLS_WORKSPACE_ALIGNMENT);
    TEST_CHECK(RLSGetWorkspaceRequirements(2U, &two_dimension_bytes,
                                           &two_dimension_alignment) ==
               RLS_STATUS_OK);
    TEST_CHECK(two_dimension_alignment == RLS_WORKSPACE_ALIGNMENT);

    TEST_CHECK(RLSInit(&rls, &config, initial, 2U, workspace.bytes,
                       two_dimension_bytes - 1U) == RLS_ERROR_CAPACITY);
    TEST_CHECK(rls._magic == 0U);
    TEST_CHECK(RLSInit(&rls, &config, initial, 2U, &misaligned[1],
                       two_dimension_bytes) == RLS_ERROR_ALIGNMENT);
    TEST_CHECK(rls._magic == 0U);
    TEST_CHECK(RLSInit(&rls, &config, initial, 2U, &rls,
                       two_dimension_bytes) == RLS_ERROR_OVERLAP);
    TEST_CHECK(rls._magic == 0U);

    ((float *)workspace.bytes)[0] = 0.0f;
    ((float *)workspace.bytes)[1] = 0.0f;
    TEST_CHECK(RLSInit(&rls, &config, (const float *)workspace.bytes, 2U,
                       workspace.bytes, two_dimension_bytes) == RLS_ERROR_OVERLAP);
    TEST_CHECK(rls._magic == 0U);

    TEST_CHECK(RLSInit(&rls, &config, initial, 2U, workspace.bytes,
                       two_dimension_bytes) == RLS_STATUS_OK);
    TEST_CHECK(RLSUpdate(&rls, input, 2U, 0.0f) == RLS_STATUS_OK);
    RLSGetStateOrFail(&rls, 2U, g_params, g_covariance, g_gain, &info);
    memcpy(g_saved_params, g_params, 2U * sizeof(float));
    memcpy(g_saved_covariance, g_covariance, 4U * sizeof(float));
    memcpy(g_saved_gain, g_gain, 2U * sizeof(float));
    memcpy(saved_workspace, workspace.bytes, two_dimension_bytes);
    TEST_CHECK(RLSInit(&rls, &config, initial, 2U, workspace.bytes,
                       two_dimension_bytes - 1U) == RLS_ERROR_CAPACITY);
    RLSCompareState(&rls, 2U, g_saved_params, g_saved_covariance,
                    g_saved_gain, &info);
    TEST_CHECK(memcmp(saved_workspace, workspace.bytes,
                      two_dimension_bytes) == 0);
    TEST_CHECK(RLSUpdate(&rls, (const float *)workspace.bytes, 2U, 1.0f) ==
               RLS_ERROR_OVERLAP);

    /* 输出容量和别名在复制前检查，出错时不触碰任一结果缓冲区。 */
    TEST_CHECK(RLSGetState(&rls, output_params, 1U, output_covariance, 16U,
                           output_gain, 2U, &info) == RLS_ERROR_CAPACITY);
    TEST_CHECK(output_params[0] == 11.0f && output_params[1] == 12.0f);
    TEST_CHECK(RLSGetState(&rls, output_params, 2U, output_covariance, 4U,
                           output_params, 2U, &info) == RLS_ERROR_OVERLAP);
    TEST_CHECK(output_params[0] == 11.0f && output_params[1] == 12.0f);

    config.dimension = 256U;
    TEST_CHECK(RLSInit(&rls, &config, initial, 2U, workspace.bytes,
                       sizeof(workspace.bytes)) == RLS_ERROR_DIMENSION);
    config.dimension = 2U;
    config.delta = NAN;
    TEST_CHECK(RLSInit(&rls, &config, initial, 2U, workspace.bytes,
                       sizeof(workspace.bytes)) == RLS_ERROR_CONFIG);
    config.delta = 1.0f;
    config.lambda = 0.0f;
    TEST_CHECK(RLSInit(&rls, &config, initial, 2U, workspace.bytes,
                       sizeof(workspace.bytes)) == RLS_ERROR_CONFIG);
    config.lambda = 1.0f;
    TEST_CHECK(RLSReset(&rls, initial, 1U) == RLS_ERROR_LENGTH);
    initial[0] = INFINITY;
    TEST_CHECK(RLSReset(&rls, initial, 2U) == RLS_ERROR_NUMERIC);
    RLSCompareState(&rls, 2U, g_saved_params, g_saved_covariance,
                    g_saved_gain, &info);
}

static void RLSCheckFailuresAndRecovery(void)
{
    const RLSConfig_t config = {2U, 1.0f, 0.99f};
    const float initial[2] = {0.5f, -0.25f};
    const float ordinary_input[2] = {0.5f, -0.75f};
    float huge_input[2] = {1.0e20f, 1.0e20f};
    RLS_t rls = {0};
    RLSWorkspace2_t workspace = {0};
    RLSStateInfo_t saved_info;
    RLSStateInfo_t after_info;
    float before_params[2];
    float before_covariance[4];
    float before_gain[2];

    TEST_CHECK(RLSInit(&rls, &config, initial, 2U, workspace.bytes,
                       sizeof(workspace.bytes)) == RLS_STATUS_OK);
    TEST_CHECK(RLSUpdate(&rls, ordinary_input, 2U, 1.75f) == RLS_STATUS_OK);
    RLSGetStateOrFail(&rls, 2U, before_params, before_covariance,
                      before_gain, &saved_info);

    TEST_CHECK(RLSUpdate(&rls, ordinary_input, 1U, 1.0f) == RLS_ERROR_LENGTH);
    TEST_CHECK(RLSUpdate(&rls, ordinary_input, 2U, NAN) == RLS_ERROR_NUMERIC);
    TEST_CHECK(RLSUpdate(&rls, NULL, 2U, 1.0f) == RLS_ERROR_NULL);
    TEST_CHECK(RLSUpdate(&rls, huge_input, 2U, 1.0f) == RLS_ERROR_NUMERIC);
    RLSCompareState(&rls, 2U, before_params, before_covariance, before_gain,
                    &saved_info);

#if defined(RLS_TEST_MATRIX_WRAP)
    {
        int fail_at;
        for (fail_at = 1; fail_at <= 5; ++fail_at)
        {
            g_mult_calls = 0;
            g_fail_mult_call = fail_at;
            g_sub_calls = 0;
            g_fail_sub_call = 0;
            TEST_CHECK(RLSUpdate(&rls, ordinary_input, 2U, 1.25f) ==
                       RLS_ERROR_MATRIX);
            RLSCompareState(&rls, 2U, before_params, before_covariance,
                            before_gain, &saved_info);
        }
        g_fail_mult_call = 0;
        g_mult_calls = 0;
        g_fail_sub_call = 1;
        g_sub_calls = 0;
        TEST_CHECK(RLSUpdate(&rls, ordinary_input, 2U, 1.25f) ==
                   RLS_ERROR_MATRIX);
        RLSCompareState(&rls, 2U, before_params, before_covariance,
                        before_gain, &saved_info);

        g_fail_sub_call = 0;
        g_sub_calls = 0;
        g_corrupt_candidate = 1;
        TEST_CHECK(RLSUpdate(&rls, ordinary_input, 2U, 1.25f) ==
                   RLS_ERROR_NUMERIC);
        g_corrupt_candidate = 0;
        RLSCompareState(&rls, 2U, before_params, before_covariance,
                        before_gain, &saved_info);
    }
#endif

    g_params[0] = 0.0f;
    g_params[1] = 0.0f;
    TEST_CHECK(RLSUpdate(&rls, ordinary_input, 2U, 1.25f) == RLS_STATUS_OK);
    RLSGetStateOrFail(&rls, 2U, g_params, g_covariance, g_gain, &after_info);
    TEST_CHECK(after_info.successful_update_count ==
               saved_info.successful_update_count + 1U);
    TEST_CHECK(after_info.has_successful_update == 1U);

#if defined(RLS_TEST_ALLOC_WRAP)
    {
        const RLSConfig_t tracked_config = {2U, 1.0f, 0.98f};
        const float tracked_initial[2] = {0.0f, 0.0f};
        RLS_t tracked_rls = {0};
        RLSWorkspace2_t tracked_workspace = {0};
        size_t bytes = 0U;
        size_t align = 0U;
        RLSStateInfo_t tracked_info;
        atomic_store(&g_allocation_calls, 0U);
        atomic_store(&g_track_allocations, 1);
        TEST_CHECK(RLSGetWorkspaceRequirements(2U, &bytes, &align) ==
                   RLS_STATUS_OK);
        TEST_CHECK(RLSInit(&tracked_rls, &tracked_config, tracked_initial, 2U,
                           tracked_workspace.bytes,
                           sizeof(tracked_workspace.bytes)) == RLS_STATUS_OK);
        TEST_CHECK(RLSUpdate(&tracked_rls, ordinary_input, 2U, 0.5f) ==
                   RLS_STATUS_OK);
        TEST_CHECK(RLSGetState(&tracked_rls, g_params, 2U, g_covariance, 4U,
                               g_gain, 2U, &tracked_info) == RLS_STATUS_OK);
        TEST_CHECK(RLSReset(&tracked_rls, tracked_initial, 2U) ==
                   RLS_STATUS_OK);
        atomic_store(&g_track_allocations, 0);
        TEST_CHECK(atomic_load(&g_allocation_calls) == 0U);
        TEST_CHECK(align == RLS_WORKSPACE_ALIGNMENT);
    }
#endif
}

static void RLSCheckMaximumAndMultipleInstances(void)
{
    const RLSConfig_t maximum_config = {255U, 0.25f, 1.0f};
    const RLSConfig_t dimension16_config = {16U, 1.0f, 1.0f};
    const RLSConfig_t first_config = {1U, 1.0f, 0.9f};
    const RLSConfig_t second_config = {2U, 2.0f, 0.99f};
    const float maximum_initial[255] = {0.0f};
    const float dimension16_initial[16] = {0.0f};
    const float dimension16_input[16] = {0.0f};
    const float first_initial[1] = {0.0f};
    const float second_initial[2] = {0.0f, 0.0f};
    const float first_input[1] = {1.0f};
    const float second_input[2] = {0.0f, 1.0f};
    RLS_t maximum_rls = {0};
    RLS_t dimension16_rls = {0};
    RLS_t first_rls = {0};
    RLS_t second_rls = {0};
    RLSWorkspace2_t first_workspace = {0};
    RLSWorkspace2_t second_workspace = {0};
    RLSWorkspace16_t dimension16_workspace = {0};
    RLSStateInfo_t first_info;
    RLSStateInfo_t second_info;
    float first_params[1];
    float first_covariance[1];
    float first_gain[1];
    float second_params[2];
    float second_covariance[4];
    float second_gain[2];

    TEST_CHECK(RLSInit(&maximum_rls, &maximum_config, maximum_initial, 255U,
                       g_workspace255, sizeof(g_workspace255)) ==
               RLS_STATUS_OK);
    TEST_CHECK(RLSReset(&maximum_rls, maximum_initial, 255U) ==
               RLS_STATUS_OK);
    TEST_CHECK(RLSInit(&dimension16_rls, &dimension16_config,
                       dimension16_initial, 16U,
                       dimension16_workspace.bytes,
                       sizeof(dimension16_workspace.bytes)) == RLS_STATUS_OK);
    TEST_CHECK(RLSUpdate(&dimension16_rls, dimension16_input, 16U, 0.0f) ==
               RLS_STATUS_OK);
    RLSGetStateOrFail(&dimension16_rls, 16U, g_params, g_covariance, g_gain,
                      &first_info);
    TEST_CHECK(first_info.successful_update_count == 1U);

    TEST_CHECK(RLSInit(&first_rls, &first_config, first_initial, 1U,
                       first_workspace.bytes,
                       sizeof(first_workspace.bytes)) == RLS_STATUS_OK);
    TEST_CHECK(RLSInit(&second_rls, &second_config, second_initial, 2U,
                       second_workspace.bytes,
                       sizeof(second_workspace.bytes)) == RLS_STATUS_OK);
    TEST_CHECK(RLSUpdate(&first_rls, first_input, 1U, 2.0f) == RLS_STATUS_OK);
    TEST_CHECK(RLSUpdate(&second_rls, second_input, 2U, -3.0f) == RLS_STATUS_OK);
    TEST_CHECK(RLSUpdate(&first_rls, first_input, 1U, 1.0f) == RLS_STATUS_OK);
    RLSGetStateOrFail(&first_rls, 1U, first_params, first_covariance,
                      first_gain, &first_info);
    RLSGetStateOrFail(&second_rls, 2U, second_params, second_covariance,
                      second_gain, &second_info);
    TEST_CHECK(first_info.successful_update_count == 2U);
    TEST_CHECK(second_info.successful_update_count == 1U);
    TEST_CHECK(first_params[0] != second_params[0]);

#if defined(RLS_TEST_THREADS)
    {
        static RLS_t thread_rls_a;
        static RLS_t thread_rls_b;
        static RLSWorkspace2_t thread_workspace_a;
        static RLSWorkspace2_t thread_workspace_b;
        static const float thread_initial[2] = {0.0f, 0.0f};
        static const float thread_input_a[2] = {0.3f, 0.8f};
        static const float thread_input_b[2] = {-0.6f, 0.2f};
        RLSThreadArgs_t args_a = {&thread_rls_a, thread_input_a, 2U, 0.7f,
                                  RLS_STATUS_OK};
        RLSThreadArgs_t args_b = {&thread_rls_b, thread_input_b, 2U, -1.2f,
                                  RLS_STATUS_OK};
        pthread_t thread_a;
        pthread_t thread_b;
        const RLSConfig_t threaded_config = {2U, 1.0f, 0.999f};
        TEST_CHECK(RLSInit(&thread_rls_a, &threaded_config, thread_initial,
                           2U, thread_workspace_a.bytes,
                           sizeof(thread_workspace_a.bytes)) == RLS_STATUS_OK);
        TEST_CHECK(RLSInit(&thread_rls_b, &threaded_config, thread_initial,
                           2U, thread_workspace_b.bytes,
                           sizeof(thread_workspace_b.bytes)) == RLS_STATUS_OK);
        TEST_CHECK(pthread_create(&thread_a, NULL, RLSWorker, &args_a) == 0);
        TEST_CHECK(pthread_create(&thread_b, NULL, RLSWorker, &args_b) == 0);
        TEST_CHECK(pthread_join(thread_a, NULL) == 0);
        TEST_CHECK(pthread_join(thread_b, NULL) == 0);
        TEST_CHECK(args_a.status == RLS_STATUS_OK);
        TEST_CHECK(args_b.status == RLS_STATUS_OK);
        RLSGetStateOrFail(&thread_rls_a, 2U, g_params, g_covariance, g_gain,
                          &first_info);
        RLSGetStateOrFail(&thread_rls_b, 2U, g_saved_params,
                          g_saved_covariance, g_saved_gain, &second_info);
        TEST_CHECK(first_info.successful_update_count == 2000U);
        TEST_CHECK(second_info.successful_update_count == 2000U);
        TEST_CHECK(memcmp(g_params, g_saved_params, 2U * sizeof(float)) != 0);
    }
#endif
}

int main(int argc, char **argv)
{
    const char *scenario = (argc > 1) ? argv[1] : "all";

    if ((strcmp(scenario, "all") == 0) || (strcmp(scenario, "math") == 0))
    {
        RLSCheckScalarAndReference();
    }
    if ((strcmp(scenario, "all") == 0) ||
        (strcmp(scenario, "workspace") == 0))
    {
        RLSCheckWorkspaceRules();
    }
    if ((strcmp(scenario, "all") == 0) ||
        (strcmp(scenario, "failures") == 0))
    {
        RLSCheckFailuresAndRecovery();
    }
    if ((strcmp(scenario, "all") == 0) ||
        (strcmp(scenario, "instances") == 0))
    {
        RLSCheckMaximumAndMultipleInstances();
    }
    TEST_CHECK((strcmp(scenario, "all") == 0) ||
               (strcmp(scenario, "math") == 0) ||
               (strcmp(scenario, "workspace") == 0) ||
               (strcmp(scenario, "failures") == 0) ||
               (strcmp(scenario, "instances") == 0));
    return 0;
}
