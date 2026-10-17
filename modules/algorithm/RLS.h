/**
* @file RLS.h
* @brief 声明递推最小二乘滤波器。它根据输入向量和实际输出，逐次调整 params_data 中的参数。
* 初始化后要调用 RLS_Reset，再按每组样本调用 RLS_Update。
*/
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

/**
* @brief 保存递推最小二乘计算所需的矩阵和参数。
* 所有输入向量和输出值必须使用一致的单位；params_data 保存当前估计结果。
*/
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
    double lambda;           // 逐步减小较早样本影响的系数
} RLS_t;

/**
* @brief 分配矩阵空间并设置滤波器维度与计算参数。
* @param rls 目标滤波器对象，必须指向可写对象。
* @param vectorSize 每组输入的元素个数，后续输入数组也须有这么多元素。
* @param delta 初始化矩阵对角元素的数值，由 RLS_Reset 写入。
* @param lambda 每次更新时降低旧数据影响程度的系数；代码不检查它的取值。
* @note 本函数不检查空指针或内存分配结果，也不返回失败状态；每个动态空间都要成功分配。
*/
void RLS_Init(RLS_t *rls, uint8_t vectorSize, float delta, float lambda);
/**
* @brief 把调用者给出的参数复制到滤波器。
* @param rls 已成功初始化的滤波器。
* @param updateParams 至少包含 vectorSize 个 float 的数组。
* @note 不检查空指针；不会复制数组之后的内容。
*/
void RLS_SetParam(RLS_t *rls, float* updateParams);
/**
* @brief 将参数和增益清零，并把传递矩阵的对角线设为 delta。
* @param rls 已成功初始化的滤波器。
* @note 当前实现不清零传递矩阵的非对角元素；再次调用时这些元素会保留原值。
*/
void RLS_Reset(RLS_t *rls);
/**
* @brief 用一组输入和对应的实际输出更新参数估计。
* @param rls 已成功初始化并调用过 RLS_Reset 的滤波器。
* @param vector 至少包含 vectorSize 个 float 的输入数组。
* @param actualOutput 与输入向量对应的实际输出值。
* @note 结果写在 rls->params_data；本函数不检查空指针，也不返回状态。
*/
void RLS_Update(RLS_t *rls, float *vector, float actualOutput);

#endif //__RLS_H
