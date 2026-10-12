#include "arm_math.h"

void arm_mat_init_f32(arm_matrix_instance_f32 *matrix, uint16_t rows, uint16_t cols, float *data)
{
    matrix->numRows = rows;
    matrix->numCols = cols;
    matrix->pData = data;
}

arm_status arm_mat_add_f32(const arm_matrix_instance_f32 *a, const arm_matrix_instance_f32 *b,
                           arm_matrix_instance_f32 *out)
{
    (void)a;
    (void)b;
    (void)out;
    return ARM_MATH_SUCCESS;
}

arm_status arm_mat_sub_f32(const arm_matrix_instance_f32 *a, const arm_matrix_instance_f32 *b,
                           arm_matrix_instance_f32 *out)
{
    (void)a;
    (void)b;
    (void)out;
    return ARM_MATH_SUCCESS;
}

arm_status arm_mat_mult_f32(const arm_matrix_instance_f32 *a, const arm_matrix_instance_f32 *b,
                            arm_matrix_instance_f32 *out)
{
    (void)a;
    (void)b;
    (void)out;
    return ARM_MATH_SUCCESS;
}

arm_status arm_mat_trans_f32(const arm_matrix_instance_f32 *a, arm_matrix_instance_f32 *out)
{
    (void)a;
    (void)out;
    return ARM_MATH_SUCCESS;
}

arm_status arm_mat_inverse_f32(const arm_matrix_instance_f32 *a, arm_matrix_instance_f32 *out)
{
    (void)a;
    (void)out;
    return ARM_MATH_SUCCESS;
}
