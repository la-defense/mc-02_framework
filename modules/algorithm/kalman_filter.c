/**
* @file kalman_filter.c
* @brief 分配卡尔曼滤波器所需空间，并按各阶段更新状态和方差。
* 自动调整模式会根据本轮非零测量重新设置 H、R 和 K 的行列数。
* @author Wang Hongxi
 * @version V1.2.2
* @date 2022/1/8
 */

#include "kalman_filter.h"
#include <string.h>

/** 确认矩阵地址、数据地址和行列数有效后，绑定已有的数据空间。 */
static uint8_t KalmanMatrixInit(mat *matrix, uint16_t rows, uint16_t cols, float *data)
{
    if (matrix == NULL || data == NULL || rows == 0u || cols == 0u)
        return 0u;
    Matrix_Init(matrix, rows, cols, data);
    return (uint8_t)(matrix->numRows == rows && matrix->numCols == cols && matrix->pData == data);
}

uint16_t sizeof_float, sizeof_double;

static void H_K_R_Adjustment(KalmanFilter_t *kf);

/**
* @brief 释放滤波器初始化时分配的数组。
* @param kf 已初始化的滤波器；NULL 时不执行操作。
 */
void Kalman_Filter_Release(KalmanFilter_t *kf)
{
    if (kf == NULL)
        return;
#define KF_FREE(field) do { if (kf->field != NULL) user_free(kf->field); kf->field = NULL; } while (0)
    KF_FREE(MeasurementMap); KF_FREE(MeasurementDegree); KF_FREE(MatR_DiagonalElements);
    KF_FREE(StateMinVariance); KF_FREE(temp); KF_FREE(FilteredValue); KF_FREE(MeasuredVector);
    KF_FREE(ControlVector); KF_FREE(xhat_data); KF_FREE(xhatminus_data); KF_FREE(u_data);
    KF_FREE(z_data); KF_FREE(P_data); KF_FREE(Pminus_data); KF_FREE(F_data); KF_FREE(FT_data);
    KF_FREE(B_data); KF_FREE(H_data); KF_FREE(HT_data); KF_FREE(Q_data); KF_FREE(R_data);
    KF_FREE(K_data); KF_FREE(S_data); KF_FREE(temp_matrix_data); KF_FREE(temp_matrix_data1);
    KF_FREE(temp_vector_data); KF_FREE(temp_vector_data1);
#undef KF_FREE
}

/**
* @brief 准备矩阵空间，全部成功后再交给目标对象。
* @param destination 要保存滤波器的对象。
* @param xhatSize 状态个数。
* @param uSize 控制输入个数；0 表示没有控制输入。
* @param zSize 测量项数。
* @return 成功返回1；指针或维数无效、分配失败时返回0。
 */
uint8_t Kalman_Filter_Init(KalmanFilter_t *destination, uint8_t xhatSize, uint8_t uSize, uint8_t zSize)
{
    if (destination == NULL || xhatSize == 0u || zSize == 0u)
        return 0u;

    // 先在临时对象中完成分配；有一步失败时，目标对象仍保留原内容。
    KalmanFilter_t prepared = {0};
    KalmanFilter_t *kf = &prepared;
#define KF_ALLOC(field, type, count)                                      \
    do                                                                     \
    {                                                                      \
        kf->field = (type *)user_malloc(sizeof(type) * (count));           \
        if (kf->field == NULL)                                             \
            goto allocation_failed;                                        \
        memset(kf->field, 0, sizeof(type) * (count));                      \
    } while (0)
#define KF_MATRIX(field, rows, cols, data)                                \
    do                                                                     \
    {                                                                      \
        if (!KalmanMatrixInit(&kf->field, (rows), (cols), kf->data))       \
            goto allocation_failed;                                        \
    } while (0)

    kf->xhatSize = xhatSize;
    kf->uSize = uSize;
    kf->zSize = zSize;

    kf->MeasurementValidNum = 0;

    // 为每项测量准备状态编号、系数、噪声和临时索引。
    KF_ALLOC(MeasurementMap, uint8_t, zSize);
    KF_ALLOC(MeasurementDegree, float, zSize);
    KF_ALLOC(MatR_DiagonalElements, float, zSize);
    KF_ALLOC(StateMinVariance, float, xhatSize);
    KF_ALLOC(temp, uint8_t, zSize);

    // 为输入、预测结果和最终状态准备缓冲区。
    KF_ALLOC(FilteredValue, float, xhatSize);
    KF_ALLOC(MeasuredVector, float, zSize);
    // 没有控制输入时不分配相关数组，后续计算也会跳过这一项。
    if (uSize != 0u)
        KF_ALLOC(ControlVector, float, uSize);

    // xhat x(k|k)
    KF_ALLOC(xhat_data, float, xhatSize);
    KF_MATRIX(xhat, kf->xhatSize, 1, xhat_data);

    // xhatminus x(k|k-1)
    KF_ALLOC(xhatminus_data, float, xhatSize);
    KF_MATRIX(xhatminus, kf->xhatSize, 1, xhatminus_data);

    if (uSize != 0)
    {
        // control vector u
        KF_ALLOC(u_data, float, uSize);
        KF_MATRIX(u, kf->uSize, 1, u_data);
    }

    // measurement vector z
    KF_ALLOC(z_data, float, zSize);
    KF_MATRIX(z, kf->zSize, 1, z_data);

    // covariance matrix P(k|k)
    KF_ALLOC(P_data, float, (size_t)xhatSize * xhatSize);
    KF_MATRIX(P, kf->xhatSize, kf->xhatSize, P_data);

    // create covariance matrix P(k|k-1)
    KF_ALLOC(Pminus_data, float, (size_t)xhatSize * xhatSize);
    KF_MATRIX(Pminus, kf->xhatSize, kf->xhatSize, Pminus_data);

    // state transition matrix F FT
    KF_ALLOC(F_data, float, (size_t)xhatSize * xhatSize);
    KF_ALLOC(FT_data, float, (size_t)xhatSize * xhatSize);
    KF_MATRIX(F, kf->xhatSize, kf->xhatSize, F_data);
    KF_MATRIX(FT, kf->xhatSize, kf->xhatSize, FT_data);

    if (uSize != 0)
    {
        // control matrix B
        KF_ALLOC(B_data, float, (size_t)xhatSize * uSize);
        KF_MATRIX(B, kf->xhatSize, kf->uSize, B_data);
    }

    // measurement matrix H
    KF_ALLOC(H_data, float, (size_t)zSize * xhatSize);
    KF_ALLOC(HT_data, float, (size_t)xhatSize * zSize);
    KF_MATRIX(H, kf->zSize, kf->xhatSize, H_data);
    KF_MATRIX(HT, kf->xhatSize, kf->zSize, HT_data);

    // process noise covariance matrix Q
    KF_ALLOC(Q_data, float, (size_t)xhatSize * xhatSize);
    KF_MATRIX(Q, kf->xhatSize, kf->xhatSize, Q_data);

    // measurement noise covariance matrix R
    KF_ALLOC(R_data, float, (size_t)zSize * zSize);
    KF_MATRIX(R, kf->zSize, kf->zSize, R_data);

    // kalman gain K
    KF_ALLOC(K_data, float, (size_t)xhatSize * zSize);
    KF_MATRIX(K, kf->xhatSize, kf->zSize, K_data);

    KF_ALLOC(S_data, float, (size_t)xhatSize * xhatSize);
    KF_ALLOC(temp_matrix_data, float, (size_t)xhatSize * xhatSize);
    KF_ALLOC(temp_matrix_data1, float, (size_t)xhatSize * xhatSize);
    KF_ALLOC(temp_vector_data, float, xhatSize);
    KF_ALLOC(temp_vector_data1, float, xhatSize);
    KF_MATRIX(S, kf->xhatSize, kf->xhatSize, S_data);
    KF_MATRIX(temp_matrix, kf->xhatSize, kf->xhatSize, temp_matrix_data);
    KF_MATRIX(temp_matrix1, kf->xhatSize, kf->xhatSize, temp_matrix_data1);
    KF_MATRIX(temp_vector, kf->xhatSize, 1, temp_vector_data);
    KF_MATRIX(temp_vector1, kf->xhatSize, 1, temp_vector_data1);

    kf->SkipEq1 = 0;
    kf->SkipEq2 = 0;
    kf->SkipEq3 = 0;
    kf->SkipEq4 = 0;
    kf->SkipEq5 = 0;

    // 所有申请都成功后，才一次性更新调用者的对象。
    *destination = prepared;
    sizeof_float = sizeof(float);
    sizeof_double = sizeof(double);
#undef KF_MATRIX
#undef KF_ALLOC
    return 1u;

// 任一申请失败都释放临时对象已取得的空间，并报告失败。
allocation_failed:
    Kalman_Filter_Release(kf);
#undef KF_MATRIX
#undef KF_ALLOC
    return 0u;
}

void Kalman_Filter_Measure(KalmanFilter_t *kf)
{
    // 按本轮有数据的传感器重新设置测量矩阵和参与计算的行数。
    if (kf->UseAutoAdjustment != 0)
        H_K_R_Adjustment(kf);
    else
    {
        memcpy(kf->z_data, kf->MeasuredVector, sizeof_float * kf->zSize);
        memset(kf->MeasuredVector, 0, sizeof_float * kf->zSize);
    }

    if (kf->uSize > 0u)
        memcpy(kf->u_data, kf->ControlVector, sizeof_float * kf->uSize);
}

void Kalman_Filter_xhatMinusUpdate(KalmanFilter_t *kf)
{
    if (!kf->SkipEq1)
    {
        if (kf->uSize > 0)
        {
            kf->temp_vector.numRows = kf->xhatSize;
            kf->temp_vector.numCols = 1;
            kf->MatStatus = Matrix_Multiply(&kf->F, &kf->xhat, &kf->temp_vector);
            kf->temp_vector1.numRows = kf->xhatSize;
            kf->temp_vector1.numCols = 1;
            kf->MatStatus = Matrix_Multiply(&kf->B, &kf->u, &kf->temp_vector1);
            kf->MatStatus = Matrix_Add(&kf->temp_vector, &kf->temp_vector1, &kf->xhatminus);
        }
        else
        {
            kf->MatStatus = Matrix_Multiply(&kf->F, &kf->xhat, &kf->xhatminus);
        }
    }
}

void Kalman_Filter_PminusUpdate(KalmanFilter_t *kf)
{
    if (!kf->SkipEq2)
    {
        kf->MatStatus = Matrix_Transpose(&kf->F, &kf->FT);
        kf->MatStatus = Matrix_Multiply(&kf->F, &kf->P, &kf->Pminus);
        kf->temp_matrix.numRows = kf->Pminus.numRows;
        kf->temp_matrix.numCols = kf->FT.numCols;
        kf->MatStatus = Matrix_Multiply(&kf->Pminus, &kf->FT, &kf->temp_matrix); // temp_matrix = F P(k-1) FT
        kf->MatStatus = Matrix_Add(&kf->temp_matrix, &kf->Q, &kf->Pminus);
    }
}
void Kalman_Filter_SetK(KalmanFilter_t *kf)
{
    if (!kf->SkipEq3)
    {
        kf->MatStatus = Matrix_Transpose(&kf->H, &kf->HT); // z|x => x|z
        kf->temp_matrix.numRows = kf->H.numRows;
        kf->temp_matrix.numCols = kf->Pminus.numCols;
        kf->MatStatus = Matrix_Multiply(&kf->H, &kf->Pminus, &kf->temp_matrix); // temp_matrix = H·P'(k)
        kf->temp_matrix1.numRows = kf->temp_matrix.numRows;
        kf->temp_matrix1.numCols = kf->HT.numCols;
        kf->MatStatus = Matrix_Multiply(&kf->temp_matrix, &kf->HT, &kf->temp_matrix1); // temp_matrix1 = H·P'(k)·HT
        kf->S.numRows = kf->R.numRows;
        kf->S.numCols = kf->R.numCols;
        kf->MatStatus = Matrix_Add(&kf->temp_matrix1, &kf->R, &kf->S); // S = H P'(k) HT + R
        kf->MatStatus = Matrix_Inverse(&kf->S, &kf->temp_matrix1);     // temp_matrix1 = inv(H·P'(k)·HT + R)
        kf->temp_matrix.numRows = kf->Pminus.numRows;
        kf->temp_matrix.numCols = kf->HT.numCols;
        kf->MatStatus = Matrix_Multiply(&kf->Pminus, &kf->HT, &kf->temp_matrix); // temp_matrix = P'(k)·HT
        kf->MatStatus = Matrix_Multiply(&kf->temp_matrix, &kf->temp_matrix1, &kf->K);
    }
}
void Kalman_Filter_xhatUpdate(KalmanFilter_t *kf)
{
    if (!kf->SkipEq4)
    {
        kf->temp_vector.numRows = kf->H.numRows;
        kf->temp_vector.numCols = 1;
        kf->MatStatus = Matrix_Multiply(&kf->H, &kf->xhatminus, &kf->temp_vector); // temp_vector = H xhat'(k)
        kf->temp_vector1.numRows = kf->z.numRows;
        kf->temp_vector1.numCols = 1;
        kf->MatStatus = Matrix_Subtract(&kf->z, &kf->temp_vector, &kf->temp_vector1); // temp_vector1 = z(k) - H·xhat'(k)
        kf->temp_vector.numRows = kf->K.numRows;
        kf->temp_vector.numCols = 1;
        kf->MatStatus = Matrix_Multiply(&kf->K, &kf->temp_vector1, &kf->temp_vector); // temp_vector = K(k)·(z(k) - H·xhat'(k))
        kf->MatStatus = Matrix_Add(&kf->xhatminus, &kf->temp_vector, &kf->xhat);
    }
}
void Kalman_Filter_P_Update(KalmanFilter_t *kf)
{
    if (!kf->SkipEq5)
    {
        kf->temp_matrix.numRows = kf->K.numRows;
        kf->temp_matrix.numCols = kf->H.numCols;
        kf->temp_matrix1.numRows = kf->temp_matrix.numRows;
        kf->temp_matrix1.numCols = kf->Pminus.numCols;
        kf->MatStatus = Matrix_Multiply(&kf->K, &kf->H, &kf->temp_matrix);                 // temp_matrix = K(k)·H
        kf->MatStatus = Matrix_Multiply(&kf->temp_matrix, &kf->Pminus, &kf->temp_matrix1); // temp_matrix1 = K(k)·H·P'(k)
        kf->MatStatus = Matrix_Subtract(&kf->Pminus, &kf->temp_matrix1, &kf->P);
    }
}


float *Kalman_Filter_Update(KalmanFilter_t *kf)
{
    // 0. 读取本轮传感器数据。
    Kalman_Filter_Measure(kf);
    if (kf->User_Func0_f != NULL)
        kf->User_Func0_f(kf);

    // 先验估计
    // 1. xhat'(k)= A·xhat(k-1) + B·u
    Kalman_Filter_xhatMinusUpdate(kf);
    if (kf->User_Func1_f != NULL)
        kf->User_Func1_f(kf);

    // 预测更新
    // 2. P'(k) = A·P(k-1)·AT + Q
    Kalman_Filter_PminusUpdate(kf);
    if (kf->User_Func2_f != NULL)
        kf->User_Func2_f(kf);

    // 自动调整模式没有有效测量时只做预测；固定矩阵模式仍按已配置的测量矩阵更新。
    if (kf->MeasurementValidNum != 0 || kf->UseAutoAdjustment == 0)
    {
        // 用有效测量修正预测结果。
        // 3. K(k) = P'(k)·HT / (H·P'(k)·HT + R)
        Kalman_Filter_SetK(kf);

        if (kf->User_Func3_f != NULL)
            kf->User_Func3_f(kf);

        // 融合
        // 4. xhat(k) = xhat'(k) + K(k)·(z(k) - H·xhat'(k))
        Kalman_Filter_xhatUpdate(kf);

        if (kf->User_Func4_f != NULL)
            kf->User_Func4_f(kf);

        // 修正方差
        // 5. P(k) = (1-K(k)·H)·P'(k) ==> P(k) = P'(k)-K(k)·H·P'(k)
        Kalman_Filter_P_Update(kf);
    }
    else
    {
        // 没有可用测量时只保留预测结果。
        // xhat(k) = xhat'(k)
        // P(k) = P'(k)
        memcpy(kf->xhat_data, kf->xhatminus_data, sizeof_float * kf->xhatSize);
        memcpy(kf->P_data, kf->Pminus_data, sizeof_float * kf->xhatSize * kf->xhatSize);
    }

    // 自定义函数,可以提供后处理等
    if (kf->User_Func5_f != NULL)
        kf->User_Func5_f(kf);

    // 防止状态方差变得过小；每个对角元素都保留至少配置的值。
    for (uint8_t i = 0; i < kf->xhatSize; ++i)
    {
        if (kf->P_data[i * kf->xhatSize + i] < kf->StateMinVariance[i])
            kf->P_data[i * kf->xhatSize + i] = kf->StateMinVariance[i];
    }

    memcpy(kf->FilteredValue, kf->xhat_data, sizeof_float * kf->xhatSize);

    if (kf->User_Func6_f != NULL)
        kf->User_Func6_f(kf);

    return kf->FilteredValue;
}

static void H_K_R_Adjustment(KalmanFilter_t *kf)
{
    kf->MeasurementValidNum = 0;

    memcpy(kf->z_data, kf->MeasuredVector, sizeof_float * kf->zSize);
    memset(kf->MeasuredVector, 0, sizeof_float * kf->zSize);

    // 根据本轮数据重建 H、R 和 K，使矩阵行数与可用测量数一致。
    memset(kf->R_data, 0, sizeof_float * kf->zSize * kf->zSize);
    memset(kf->H_data, 0, sizeof_float * kf->xhatSize * kf->zSize);
    // 逐项保留非零测量；结果按原顺序放到数组前面。
    for (uint8_t i = 0; i < kf->zSize; ++i)
    {
        // 当前实现把数值0当作没有新数据；真实测量值为0时也会被跳过。
        if (kf->z_data[i] != 0)
        {
            // 将有效数据移到数组前面，形成本轮使用的测量向量。
            kf->z_data[kf->MeasurementValidNum] = kf->z_data[i];
            kf->temp[kf->MeasurementValidNum] = i;
            // MeasurementMap 的状态编号从1开始，写入矩阵时减1；调用者要传入有效编号。
            kf->H_data[kf->xhatSize * kf->MeasurementValidNum + kf->MeasurementMap[i] - 1] = kf->MeasurementDegree[i];
            kf->MeasurementValidNum++;
        }
    }
    // R 只为本轮保留的测量填写对应噪声值。
    for (uint8_t i = 0; i < kf->MeasurementValidNum; ++i)
    {
        // 将每项有效测量的噪声值放到 R 矩阵对角线上。
        kf->R_data[i * kf->MeasurementValidNum + i] = kf->MatR_DiagonalElements[kf->temp[i]];
    }

    // 更新矩阵行列数，后续运算只处理本轮有效测量。
    kf->H.numRows = kf->MeasurementValidNum;
    kf->H.numCols = kf->xhatSize;
    kf->HT.numRows = kf->xhatSize;
    kf->HT.numCols = kf->MeasurementValidNum;
    kf->R.numRows = kf->MeasurementValidNum;
    kf->R.numCols = kf->MeasurementValidNum;
    kf->K.numRows = kf->xhatSize;
    kf->K.numCols = kf->MeasurementValidNum;
    kf->z.numRows = kf->MeasurementValidNum;
}
