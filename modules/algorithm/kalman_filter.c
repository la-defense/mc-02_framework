/**
 ******************************************************************************
 * @file    kalman filter.c
 * @author  Wang Hongxi
 * @version V1.2.2
 * @date    2022/1/8
 * @brief   C implementation of kalman filter
 ******************************************************************************
 * @attention
 * 该卡尔曼滤波器可以在传感器采样频率不同的情况下，动态调整矩阵H R和K的维数与数值。
 * This implementation of kalman filter can dynamically adjust dimension and
 * value of matrix H R and K according to the measurement validity under any
 * circumstance that the sampling rate of component sensors are different.
 *
 * 因此矩阵H和R的初始化会与矩阵P A和Q有所不同。另外的，在初始化量测向量z时需要额外写
 * 入传感器量测所对应的状态与这个量测的方式，详情请见例程
 * Therefore, the initialization of matrix P, F, and Q is sometimes different
 * from that of matrices H R. when initialization. Additionally, the corresponding
 * state and the method of the measurement should be provided when initializing
 * measurement vector z. For more details, please see the example.
 *
 * 若不需要动态调整量测向量z，可简单将结构体中的Use_Auto_Adjustment初始化为0，并像初
 * 始化矩阵P那样用常规方式初始化z H R即可。
 * If automatic adjustment is not required, assign zero to the UseAutoAdjustment
 * and initialize z H R in the normal way as matrix P.
 *
 * 要求量测向量z与控制向量u在传感器回调函数中更新。整数0意味着量测无效，即自上次卡尔曼
 * 滤波更新后无传感器数据更新。因此量测向量z与控制向量u会在卡尔曼滤波更新过程中被清零
 * MeasuredVector and ControlVector are required to be updated in the sensor
 * callback function. Integer 0 in measurement vector z indicates the invalidity
 * of current measurement, so MeasuredVector and ControlVector will be reset
 * (to 0) during each update.
 *
 * 此外，矩阵P过度收敛后滤波器将难以再适应状态的缓慢变化，从而产生滤波估计偏差。该算法
 * 通过限制矩阵P最小值的方法，可有效抑制滤波器的过度收敛，详情请见例程。
 * Additionally, the excessive convergence of matrix P will make filter incapable
 * of adopting the slowly changing state. This implementation can effectively
 * suppress filter excessive convergence through boundary limiting for matrix P.
 * For more details, please see the example.
 *
 * @example:
 * x =
 *   |   height   |
 *   |  velocity  |
 *   |acceleration|
 *
 * KalmanFilter_t Height_KF;
 *
 * void INS_Task_Init(void)
 * {
 *     static float P_Init[9] =
 *     {
 *         10, 0, 0,
 *         0, 30, 0,
 *         0, 0, 10,
 *     };
 *     static float F_Init[9] =
 *     {
 *         1, dt, 0.5*dt*dt,
 *         0, 1, dt,
 *         0, 0, 1,
 *     };
 *     static float Q_Init[9] =
 *     {
 *         0.25*dt*dt*dt*dt, 0.5*dt*dt*dt, 0.5*dt*dt,
 *         0.5*dt*dt*dt,        dt*dt,         dt,
 *         0.5*dt*dt,              dt,         1,
 *     };
 *
 *     // 设置最小方差
 *     static float state_min_variance[3] = {0.03, 0.005, 0.1};
 *
 *     // 开启自动调整
 *     Height_KF.UseAutoAdjustment = 1;
 *
 *     // 气压测得高度 GPS测得高度 加速度计测得z轴运动加速度
 *     static uint8_t measurement_reference[3] = {1, 1, 3}
 *
 *     static float measurement_degree[3] = {1, 1, 1}
 *     // 根据measurement_reference与measurement_degree生成H矩阵如下（在当前周期全部测量数据有效情况下）
 *       |1   0   0|
 *       |1   0   0|
 *       |0   0   1|
 *
 *     static float mat_R_diagonal_elements = {30, 25, 35}
 *     //根据mat_R_diagonal_elements生成R矩阵如下（在当前周期全部测量数据有效情况下）
 *       |30   0   0|
 *       | 0  25   0|
 *       | 0   0  35|
 *
 *     Kalman_Filter_Init(&Height_KF, 3, 0, 3);
 *
 *     // 设置矩阵值
 *     memcpy(Height_KF.P_data, P_Init, sizeof(P_Init));
 *     memcpy(Height_KF.F_data, F_Init, sizeof(F_Init));
 *     memcpy(Height_KF.Q_data, Q_Init, sizeof(Q_Init));
 *     memcpy(Height_KF.MeasurementMap, measurement_reference, sizeof(measurement_reference));
 *     memcpy(Height_KF.MeasurementDegree, measurement_degree, sizeof(measurement_degree));
 *     memcpy(Height_KF.MatR_DiagonalElements, mat_R_diagonal_elements, sizeof(mat_R_diagonal_elements));
 *     memcpy(Height_KF.StateMinVariance, state_min_variance, sizeof(state_min_variance));
 * }
 *
 * void INS_Task(void const *pvParameters)
 * {
 *     // 循环更新
 *     Kalman_Filter_Update(&Height_KF);
 *     vTaskDelay(ts);
 * }
 *
 * // 测量数据更新应按照以下形式 即向MeasuredVector赋值
 * void Barometer_Read_Over(void)
 * {
 *     ......
 *     INS_KF.MeasuredVector[0] = baro_height;
 * }
 * void GPS_Read_Over(void)
 * {
 *     ......
 *     INS_KF.MeasuredVector[1] = GPS_height;
 * }
 * void Acc_Data_Process(void)
 * {
 *     ......
 *     INS_KF.MeasuredVector[2] = acc.z;
 * }
 ******************************************************************************
 */

#include "kalman_filter.h"
#include <string.h>

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
 * @brief 初始化矩阵维度信息并为矩阵分配空间
 *
 * @param kf kf类型定义
 * @param xhatSize 状态变量维度
 * @param uSize 控制变量维度
 * @param zSize 观测量维度
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

uint8_t Kalman_Filter_Init(KalmanFilter_t *destination, uint8_t xhatSize, uint8_t uSize, uint8_t zSize)
{
    if (destination == NULL || xhatSize == 0u || zSize == 0u)
        return 0u;

    /* Build off to the side so failure never publishes a partial filter. */
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

    // measurement flags
    KF_ALLOC(MeasurementMap, uint8_t, zSize);
    KF_ALLOC(MeasurementDegree, float, zSize);
    KF_ALLOC(MatR_DiagonalElements, float, zSize);
    KF_ALLOC(StateMinVariance, float, xhatSize);
    KF_ALLOC(temp, uint8_t, zSize);

    // filter data
    KF_ALLOC(FilteredValue, float, xhatSize);
    KF_ALLOC(MeasuredVector, float, zSize);
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

    *destination = prepared;
    sizeof_float = sizeof(float);
    sizeof_double = sizeof(double);
#undef KF_MATRIX
#undef KF_ALLOC
    return 1u;

allocation_failed:
    Kalman_Filter_Release(kf);
#undef KF_MATRIX
#undef KF_ALLOC
    return 0u;
}

void Kalman_Filter_Measure(KalmanFilter_t *kf)
{
    // 矩阵H K R根据量测情况自动调整
    // matrix H K R auto adjustment
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

/**
 * @brief 执行卡尔曼滤波黄金五式,提供了用户定义函数,可以替代五个中的任意一个环节,方便自行扩展为EKF/UKF/ESKF/AUKF等
 * 
 * @param kf kf类型定义
 * @return float* 返回滤波值
 */
float *Kalman_Filter_Update(KalmanFilter_t *kf)
{
    // 0. 获取量测信息
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

    if (kf->MeasurementValidNum != 0 || kf->UseAutoAdjustment == 0)
    {
        // 量测更新
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
        // 无有效量测,仅预测
        // xhat(k) = xhat'(k)
        // P(k) = P'(k)
        memcpy(kf->xhat_data, kf->xhatminus_data, sizeof_float * kf->xhatSize);
        memcpy(kf->P_data, kf->Pminus_data, sizeof_float * kf->xhatSize * kf->xhatSize);
    }

    // 自定义函数,可以提供后处理等
    if (kf->User_Func5_f != NULL)
        kf->User_Func5_f(kf);

    // 避免滤波器过度收敛
    // suppress filter excessive convergence
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

    // 识别量测数据有效性并调整矩阵H R K
    // recognize measurement validity and adjust matrices H R K
    memset(kf->R_data, 0, sizeof_float * kf->zSize * kf->zSize);
    memset(kf->H_data, 0, sizeof_float * kf->xhatSize * kf->zSize);
    for (uint8_t i = 0; i < kf->zSize; ++i)
    {
        if (kf->z_data[i] != 0)
        {
            // 重构向量z
            // rebuild vector z
            kf->z_data[kf->MeasurementValidNum] = kf->z_data[i];
            kf->temp[kf->MeasurementValidNum] = i;
            // 重构矩阵H
            // rebuild matrix H
            kf->H_data[kf->xhatSize * kf->MeasurementValidNum + kf->MeasurementMap[i] - 1] = kf->MeasurementDegree[i];
            kf->MeasurementValidNum++;
        }
    }
    for (uint8_t i = 0; i < kf->MeasurementValidNum; ++i)
    {
        // 重构矩阵R
        // rebuild matrix R
        kf->R_data[i * kf->MeasurementValidNum + i] = kf->MatR_DiagonalElements[kf->temp[i]];
    }

    // 调整矩阵维数
    // adjust the dimensions of system matrices
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
