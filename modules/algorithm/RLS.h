#ifndef __RLS_H
#define __RLS_H

//#include "stm32f407xx.h"
#include "arm_math.h"
#include "math.h"
#include "stdint.h"
#include "stdlib.h"

// 若运算速度不够,可以使用q31代替f32,但是精度会降低
#define mat arm_matrix_instance_f32
#define Matrix_Init arm_mat_init_f32
#define Matrix_Add arm_mat_add_f32
#define Matrix_Subtract arm_mat_sub_f32
#define Matrix_Multiply arm_mat_mult_f32
#define Matrix_Transpose arm_mat_trans_f32
#define Matrix_Inverse arm_mat_inverse_f32

typedef struct rls_t
{
    uint8_t vectorSize;

    int8_t MatStatus;

    // definiion of struct mat: rows & cols & pointer to vars
    mat input_vector, t_input_vector;       // X X'
    mat gain_vector;        // G
    mat params_vector;      // P
    mat trans_matrix;       // T
    mat temp, temp_matrix, temp_matrix1, temp_vector, temp_vector1;

    // 矩阵存储空间指针
    float *input_data, *t_input_data;
    float *output_data;
    float *gain_data;
    float *params_data;
    float *trans_data;
    float *temp_data, *temp_matrix_data, *temp_matrix1_data, *temp_vector_data, *temp_vector1_data;

    double delta;            // 传递矩阵初始化
    double lambda;           // 渐消因子
} RLS_t;

void RLS_Init(RLS_t *rls, uint8_t vectorSize, float delta, float lambda);
void RLS_SetParam(RLS_t *rls, float* updateParams);
void RLS_Reset(RLS_t *rls);
void RLS_Update(RLS_t *rls, float *vector, float actualOutput);

#endif //__RLS_H
