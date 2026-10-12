#ifndef MC02_HOST_ARM_MATH_H
#define MC02_HOST_ARM_MATH_H
#include <stdint.h>
typedef enum { ARM_MATH_SUCCESS = 0, ARM_MATH_ARGUMENT_ERROR = -1, ARM_MATH_SIZE_MISMATCH = -2 } arm_status;
typedef struct { uint16_t numRows; uint16_t numCols; float *pData; } arm_matrix_instance_f32;
void arm_mat_init_f32(arm_matrix_instance_f32 *matrix, uint16_t rows, uint16_t cols, float *data);
arm_status arm_mat_add_f32(const arm_matrix_instance_f32 *, const arm_matrix_instance_f32 *, arm_matrix_instance_f32 *);
arm_status arm_mat_sub_f32(const arm_matrix_instance_f32 *, const arm_matrix_instance_f32 *, arm_matrix_instance_f32 *);
arm_status arm_mat_mult_f32(const arm_matrix_instance_f32 *, const arm_matrix_instance_f32 *, arm_matrix_instance_f32 *);
arm_status arm_mat_trans_f32(const arm_matrix_instance_f32 *, arm_matrix_instance_f32 *);
arm_status arm_mat_inverse_f32(const arm_matrix_instance_f32 *, arm_matrix_instance_f32 *);
#endif
