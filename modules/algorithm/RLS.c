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
 * @param delta 传递矩阵因子
 * @param lambda 渐消因子
 */
void RLS_Init(RLS_t *rls, uint8_t vectorSize, float delta, float lambda)
{
    // 计算出的矩阵数值依据精度而定，值不一定均为0
    // 使用double计算可能导致数据溢出
    sizeof_float = sizeof(float);
    sizeof_double = sizeof(double);

    rls->vectorSize = vectorSize;

    // init RLS vectors
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
    // Copy the input vector to the input data array
    for (uint8_t i = 0; i < rls->input_vector.numRows * rls->input_vector.numCols; ++i)
    {
        rls->input_data[i] = vector[i];
    }
    // rls->input_data[0] = vector[0];
    // rls->input_data[1] = vector[1];

    // Calculate the transpose of the input vector and store it in the temp vector
    rls->temp_vector.numRows = rls->input_vector.numRows;
    rls->temp_vector.numCols = 1;
    rls->MatStatus = Matrix_Multiply(&rls->trans_matrix, &rls->input_vector, &rls->temp_vector);    // temp_vector = T·X

    // Calculate the transpose of the input vector and store it in the t_input_vector
    rls->MatStatus = Matrix_Transpose(&rls->input_vector, &rls->t_input_vector);    // X=>X'
    rls->temp_vector1.numRows = 1;
    rls->temp_vector1.numCols = rls->t_input_vector.numCols;
    rls->MatStatus = Matrix_Multiply(&rls->t_input_vector, &rls->trans_matrix, &rls->temp_vector1);  // temp_vector1 = X'·T
    rls->temp.numRows = 1;
    rls->temp.numCols = 1;
    rls->MatStatus = Matrix_Multiply(&rls->temp_vector1, &rls->input_vector, &rls->temp);     // temp = X'·T·X

    // Calculate the gain vector
    for (uint8_t i = 0; i < rls->temp_vector.numRows * rls->temp_vector.numCols; ++i)
    {
        rls->temp_vector_data[i] /= 1.0f + (rls->temp_data[0] / rls->lambda);
        rls->temp_vector_data[i] /= rls->lambda;
    }
    for (uint8_t i = 0; i < rls->gain_vector.numRows * rls->gain_vector.numCols; ++i)
    {
        // Get gain vector = T·X ./ (1 + X'·T·X ./ lambda) ./ lambda
        rls->gain_data[i] = rls->temp_vector_data[i];
    }
    // rls->MatStatus = Matrix_Transpose(&rls->temp_vector, &rls->gain_vector);

    // Calculate the temp vector
    rls->temp.numRows = 1;
    rls->temp.numCols = 1;
    rls->MatStatus = Matrix_Multiply(&rls->t_input_vector, &rls->params_vector, &rls->temp);        // temp = X'·P

    // Update the actual output
    actualOutput -= rls->temp_data[0];
    for (uint8_t i = 0; i < rls->gain_vector.numRows * rls->gain_vector.numCols; ++i)
    {
        rls->gain_data[i] *= actualOutput;
    }
    rls->MatStatus = Matrix_Add(&rls->params_vector, &rls->gain_vector, &rls->params_vector);    // Get params vector = P + G.*(out - (X'·P))

    // Calculate the temp matrix
    rls->temp_matrix.numRows = rls->trans_matrix.numRows;
    rls->temp_matrix.numCols = rls->trans_matrix.numCols;
    rls->MatStatus = Matrix_Multiply(&rls->gain_vector, &rls->t_input_vector, &rls->temp_matrix);        // temp_matrix = G·X'
    rls->temp_matrix1.numRows = rls->trans_matrix.numRows;
    rls->temp_matrix1.numCols = rls->trans_matrix.numCols;
    rls->MatStatus = Matrix_Multiply(&rls->temp_matrix, &rls->trans_matrix, &rls->temp_matrix1);
    rls->temp_matrix1.numRows = rls->trans_matrix.numRows;
    rls->temp_matrix1.numCols = rls->trans_matrix.numCols;
    rls->MatStatus = Matrix_Subtract(&rls->trans_matrix, &rls->temp_matrix1, &rls->temp_matrix1);        // temp_matrix1 = T - (G·X'·T)
    for (uint8_t i = 0; i < rls->trans_matrix.numRows * rls->trans_matrix.numCols; ++i)
    {
        rls->trans_data[i] = rls->temp_matrix1_data[i] / rls->lambda;                   // Get trans matrix = (T - (G·X'·T)) ./ lambda
    }
}
