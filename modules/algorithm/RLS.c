/**
* @file RLS.c
* @brief 分配并更新递推最小二乘算法使用的向量和矩阵。
 */
#include "RLS.h"

#ifdef _CMSIS_OS_H
#define user_malloc pvPortMalloc
#else
#define user_malloc malloc
#endif

static uint16_t sizeof_float, sizeof_double;

/**
 * @brief 初始化矩阵维度信息并为矩阵分配空间
 *
 * @param rls rls类型定义
 * @param vectorSize 输入变量维度
* @param delta 重置时传递矩阵对角线的起始值
* @param lambda 逐步减小较早样本影响的系数
 */
void RLS_Init(RLS_t *rls, uint8_t vectorSize, float delta, float lambda)
{
    // 记录元素字节数，后面用它清零各块矩阵存储空间。
    sizeof_float = sizeof(float);
    sizeof_double = sizeof(double);

    rls->vectorSize = vectorSize;

    // 为输入、转置输入、增益和参数向量分别准备存储空间。
    rls->input_data = (float *)user_malloc(sizeof_float * vectorSize);
    memset(rls->input_data, 0, sizeof_float * vectorSize);
    Matrix_Init(&rls->input_vector, rls->vectorSize, 1, (float *)rls->input_data);
    rls->t_input_data = (float *)user_malloc(sizeof_float * vectorSize);
    memset(rls->t_input_data, 0, sizeof_float * vectorSize);
    Matrix_Init(&rls->t_input_vector, 1, rls->vectorSize, (float *)rls->t_input_data);

    rls->trans_data = (float *)user_malloc(sizeof_float * vectorSize * vectorSize);
    memset(rls->trans_data, 0, sizeof_float * vectorSize * vectorSize);
    Matrix_Init(&rls->trans_matrix, rls->vectorSize, rls->vectorSize, (float *)rls->trans_data);

    rls->gain_data = (float *)user_malloc(sizeof_float * vectorSize);
    memset(rls->gain_data, 0, sizeof_float * vectorSize);
    Matrix_Init(&rls->gain_vector, rls->vectorSize, 1, (float *)rls->gain_data);

    rls->params_data = (float *)user_malloc(sizeof_float * vectorSize);
    memset(rls->params_data, 0, sizeof_float * vectorSize);
    Matrix_Init(&rls->params_vector, rls->vectorSize, 1, (float *)rls->params_data);

    rls->temp_data = (float *)user_malloc(sizeof_float);
    memset(rls->temp_data, 0, sizeof_float);
    Matrix_Init(&rls->temp, 1, 1, (float *)rls->temp_data);

    rls->temp_vector_data = (float *)user_malloc(sizeof_float * vectorSize);
    memset(rls->temp_vector_data, 0, sizeof_float * vectorSize);
    Matrix_Init(&rls->temp_vector, rls->vectorSize, 1, (float *)rls->temp_vector_data);
    rls->temp_vector1_data = (float *)user_malloc(sizeof_float * vectorSize);
    memset(rls->temp_vector1_data, 0, sizeof_float * vectorSize);
    Matrix_Init(&rls->temp_vector1, rls->vectorSize, 1, (float *)rls->temp_vector1_data);

    rls->temp_matrix_data = (float *)user_malloc(sizeof_float * vectorSize * vectorSize);
    memset(rls->temp_matrix_data, 0, sizeof_float * vectorSize * vectorSize);
    Matrix_Init(&rls->temp_matrix, rls->vectorSize, rls->vectorSize, (float *)rls->temp_matrix_data);
    rls->temp_matrix1_data = (float *)user_malloc(sizeof_float * vectorSize * vectorSize);
    memset(rls->temp_matrix1_data, 0, sizeof_float * vectorSize * vectorSize);
    Matrix_Init(&rls->temp_matrix1, rls->vectorSize, rls->vectorSize, (float *)rls->temp_matrix1_data);

    // 保存重置时使用的对角线初值和每次更新的渐消因子。
    rls->delta = delta;
    rls->lambda = lambda;
}

/**
 * @brief 设置RLS矩阵参数
 *
 * @param rls rls类型定义
 * @param updateParams rls更新参数
 */
void RLS_SetParam(RLS_t *rls, float* updateParams)
{
    // 把调用者给出的每个参数依次保存到滤波器的结果数组。
    for (uint8_t i = 0; i < rls->vectorSize; ++i)
    {
        rls->params_data[i] = updateParams[i];
    }
}

/**
 * @brief 清除RLS矩阵参数
 *
 * @param rls rls类型定义
 */
void RLS_Reset(RLS_t *rls)
{
    // 只重设对角线；非对角元素不会在这里清除。
    for (uint8_t i = 0; i < rls->trans_matrix.numRows; ++i)
    {
        rls->trans_data[i * rls->trans_matrix.numCols + i] = 1.0f;
        rls->trans_data[i * rls->trans_matrix.numCols + i] *= rls->delta;
    }

    memset(rls->gain_data, 0, sizeof_float * rls->vectorSize);
    memset(rls->params_data, 0, sizeof_float * rls->vectorSize);
}

/**
 * @brief 更新RLS输出矩阵
 *
 * @param rls rls类型定义
 * @param vector 输入变量
 * @param actualOutput 实际输出
 */
void RLS_Update(RLS_t *rls, float *vector, float actualOutput)
{
    // 复制调用者的输入，后续矩阵运算都使用滤波器自己的缓冲区。
    for (uint8_t i = 0; i < rls->input_vector.numRows * rls->input_vector.numCols; ++i)
    {
        rls->input_data[i] = vector[i];
    }
    // rls->input_data[0] = vector[0];
    // rls->input_data[1] = vector[1];

    // 先计算 T·X 和 X'·T·X，供本轮增益计算使用。
    rls->temp_vector.numRows = rls->input_vector.numRows;
    rls->temp_vector.numCols = 1;
    rls->MatStatus = Matrix_Multiply(&rls->trans_matrix, &rls->input_vector, &rls->temp_vector);    // 计算传递矩阵对当前输入的加权结果。

    // 转置输入向量，后面用它计算预测输出和更新矩阵。
    rls->MatStatus = Matrix_Transpose(&rls->input_vector, &rls->t_input_vector);    // 将输入列向量转成行向量，供后续矩阵乘法使用。
    rls->temp_vector1.numRows = 1;
    rls->temp_vector1.numCols = rls->t_input_vector.numCols;
    rls->MatStatus = Matrix_Multiply(&rls->t_input_vector, &rls->trans_matrix, &rls->temp_vector1);  // 计算输入对传递矩阵的加权结果。
    rls->temp.numRows = 1;
    rls->temp.numCols = 1;
    rls->MatStatus = Matrix_Multiply(&rls->temp_vector1, &rls->input_vector, &rls->temp);     // 计算当前输入对应的标量权重。

    // 按输入与传递矩阵的关系计算本轮增益。
    // 按 lambda 调整增益，让较早样本对后续结果的影响逐步减小。
    for (uint8_t i = 0; i < rls->temp_vector.numRows * rls->temp_vector.numCols; ++i)
    {
        rls->temp_vector_data[i] /= 1.0f + (rls->temp_data[0] / rls->lambda);
        rls->temp_vector_data[i] /= rls->lambda;
    }
    for (uint8_t i = 0; i < rls->gain_vector.numRows * rls->gain_vector.numCols; ++i)
    {
        // 逐项保存上面得到的增益向量。
        rls->gain_data[i] = rls->temp_vector_data[i];
    }
    // rls->MatStatus = Matrix_Transpose(&rls->temp_vector, &rls->gain_vector);

    // 先用旧参数估计当前输出，再计算它与实际输出的差。
    rls->temp.numRows = 1;
    rls->temp.numCols = 1;
    rls->MatStatus = Matrix_Multiply(&rls->t_input_vector, &rls->params_vector, &rls->temp);        // 用旧参数估计当前输出。

    // 用输出误差调整参数；误差为零时本轮不会改变参数。
    actualOutput -= rls->temp_data[0];
    for (uint8_t i = 0; i < rls->gain_vector.numRows * rls->gain_vector.numCols; ++i)
    {
        rls->gain_data[i] *= actualOutput;
    }
    rls->MatStatus = Matrix_Add(&rls->params_vector, &rls->gain_vector, &rls->params_vector);    // 把增益乘以输出误差，累加到参数估计中。

    // 用增益和输入更新传递矩阵，为下一组样本做准备。
    rls->temp_matrix.numRows = rls->trans_matrix.numRows;
    rls->temp_matrix.numCols = rls->trans_matrix.numCols;
    rls->MatStatus = Matrix_Multiply(&rls->gain_vector, &rls->t_input_vector, &rls->temp_matrix);        // 用增益和输入构造本轮需要扣除的矩阵部分。
    rls->temp_matrix1.numRows = rls->trans_matrix.numRows;
    rls->temp_matrix1.numCols = rls->trans_matrix.numCols;
    rls->MatStatus = Matrix_Multiply(&rls->temp_matrix, &rls->trans_matrix, &rls->temp_matrix1);
    rls->temp_matrix1.numRows = rls->trans_matrix.numRows;
    rls->temp_matrix1.numCols = rls->trans_matrix.numCols;
    rls->MatStatus = Matrix_Subtract(&rls->trans_matrix, &rls->temp_matrix1, &rls->temp_matrix1);        // 从旧传递矩阵中减去本轮修正量。
    for (uint8_t i = 0; i < rls->trans_matrix.numRows * rls->trans_matrix.numCols; ++i)
    {
        rls->trans_data[i] = rls->temp_matrix1_data[i] / rls->lambda;                   // 再按 lambda 调整传递矩阵，为下一组输入做准备。
    }
}
