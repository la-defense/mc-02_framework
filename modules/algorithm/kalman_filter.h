/**
* @file kalman_filter.h
* @brief 声明通用卡尔曼滤波器的数据结构和计算步骤。
* 初始化后填写模型矩阵、初始状态和最小方差，再周期调用 Kalman_Filter_Update。
* 自动调整测量矩阵时，MeasuredVector 中的0表示该传感器本轮没有新数据；固定矩阵模式会在读取后清空输入数组。
* @author Wang Hongxi
 * @version V1.2.2
* @date 2022/1/8
 */
#ifndef __KALMAN_FILTER_H
#define __KALMAN_FILTER_H

// cortex-m4 DSP lib
/*
#define __CC_ARM    // Keil
#define ARM_MATH_CM4
#define ARM_MATH_MATRIX_CHECK
#define ARM_MATH_ROUNDING
#define ARM_MATH_DSP    // define in arm_math.h
*/

//#include "stm32f407xx.h"
#include "arm_math.h"
//#include "dsp/matrix_functions.h"
#include "math.h"
#include "stdint.h"
#include "stdlib.h"

#ifndef user_malloc
#ifdef _CMSIS_OS_H
#define user_malloc pvPortMalloc
#define user_free pvPortFree
#else
#define user_malloc malloc
#define user_free free
#endif
#endif

// 若运算速度不够,可以使用q31代替f32,但是精度会降低
#define mat arm_matrix_instance_f32
#define Matrix_Init arm_mat_init_f32
#define Matrix_Add arm_mat_add_f32
#define Matrix_Subtract arm_mat_sub_f32
#define Matrix_Multiply arm_mat_mult_f32
#define Matrix_Transpose arm_mat_trans_f32
#define Matrix_Inverse arm_mat_inverse_f32

/**
* @brief 保存卡尔曼滤波器的矩阵、数据缓冲区和扩展回调。
* 矩阵使用单精度浮点数；数组长度按 xhatSize、uSize 和 zSize 分配。
 */
typedef struct kf_t
{
    float *FilteredValue;
    float *MeasuredVector;
    float *ControlVector;

    uint8_t xhatSize;
    uint8_t uSize;
    uint8_t zSize;

    uint8_t UseAutoAdjustment; // 非零时按本轮数据调整测量矩阵；0时使用调用者准备的矩阵
    uint8_t MeasurementValidNum; // 本轮实际参与计算的测量数

    /* 自动调整测量矩阵时使用：每项测量对应的状态编号从1开始。 */
    uint8_t *MeasurementMap;      // 自动调整模式下，每项测量对应的状态编号，从1开始
    /* 自动调整测量矩阵时，写入 H 矩阵的系数。 */
    float *MeasurementDegree;     // 每项测量在 H 矩阵中使用的系数
    /* 自动调整测量矩阵时，各项数据对应的 R 矩阵对角值。 */
    float *MatR_DiagonalElements; // 每项测量对应的噪声值，写入 R 矩阵对角线
    /* 每个状态方差允许的最小值，更新末尾会逐项检查。 */
    float *StateMinVariance;      // 每个状态方差允许的最小值；每轮更新结束时都会检查
    uint8_t *temp; // 记录有效测量在原数组中的位置

    // 对应位置非零时跳过预测、方差预测、权重计算、状态修正或方差更新步骤。
    uint8_t SkipEq1, SkipEq2, SkipEq3, SkipEq4, SkipEq5;

    // 每个 mat 保存矩阵的行数、列数和数据地址。
    mat xhat;      // 当前修正后的状态
    mat xhatminus; // 根据上一轮结果预测的状态
    mat u;         // 控制输入
    mat z;         // 本轮参与计算的测量数据
    mat P;         // 当前状态估计的方差
    mat Pminus;    // 预测状态的方差
    mat F, FT;     // 描述状态怎样变化的矩阵及其转置
    mat B;         // 描述控制输入怎样影响状态
    mat H, HT;     // 描述状态与测量数据关系的矩阵及其转置
    mat Q;         // 状态预测误差的大小
    mat R;         // 传感器测量误差的大小
    mat K;         // 把预测结果向测量结果修正时使用的权重
    mat S, temp_matrix, temp_matrix1, temp_vector, temp_vector1;

    int8_t MatStatus; // 最近一次矩阵计算的状态

    // 用户定义函数,可以替换或扩展基准KF的功能
    void (*User_Func0_f)(struct kf_t *kf);
    void (*User_Func1_f)(struct kf_t *kf);
    void (*User_Func2_f)(struct kf_t *kf);
    void (*User_Func3_f)(struct kf_t *kf);
    void (*User_Func4_f)(struct kf_t *kf);
    void (*User_Func5_f)(struct kf_t *kf);
    void (*User_Func6_f)(struct kf_t *kf);

    // 矩阵存储空间指针
    float *xhat_data, *xhatminus_data;
    float *u_data;
    float *z_data;
    float *P_data, *Pminus_data;
    float *F_data, *FT_data;
    float *B_data;
    float *H_data, *HT_data;
    float *Q_data;
    float *R_data;
    float *K_data;
    float *S_data, *temp_matrix_data, *temp_matrix_data1, *temp_vector_data, *temp_vector_data1;
} KalmanFilter_t;

extern uint16_t sizeof_float, sizeof_double;

/**
* @brief 为滤波器分配矩阵和数据缓冲区。
* @param kf 目标对象；初始化成功后由它保存分配结果。
* @param xhatSize 状态个数，须大于0且不小于 zSize。
* @param uSize 控制输入个数；为0时不分配控制向量和控制矩阵。
* @param zSize 测量个数，须大于0。
* @return 成功返回1；空指针、维数不合要求或内存分配失败时返回0。
* @note 重新初始化已分配过的对象前，先调用 Kalman_Filter_Release，避免旧内存没有释放。
 */
uint8_t Kalman_Filter_Init(KalmanFilter_t *kf, uint8_t xhatSize, uint8_t uSize, uint8_t zSize);
/**
* @brief 释放初始化时分配的缓冲区。
* @param kf 已初始化的对象；传入 NULL 时直接返回。
 */
void Kalman_Filter_Release(KalmanFilter_t *kf);
/**
* @brief 把回调写入的测量和控制输入送入本轮计算。
* @param kf 已初始化的滤波器；测量数组会在读取后清零，控制数组会被复制。
 */
void Kalman_Filter_Measure(KalmanFilter_t *kf);
/**
* @brief 根据上一时刻状态预测当前状态；SkipEq1 非零时跳过。
* @param kf 已初始化且模型矩阵已填写的滤波器。
 */
void Kalman_Filter_xhatMinusUpdate(KalmanFilter_t *kf);
/**
* @brief 预测当前状态的方差；SkipEq2 非零时跳过。
* @param kf 已初始化且模型矩阵已填写的滤波器。
 */
void Kalman_Filter_PminusUpdate(KalmanFilter_t *kf);
/**
* @brief 根据测量矩阵、测量噪声和预测方差计算修正权重 K；SkipEq3 非零时跳过。
* @param kf 已初始化且本轮测量矩阵已准备好的滤波器。
 */
void Kalman_Filter_SetK(KalmanFilter_t *kf);
/**
* @brief 用本轮测量修正预测状态；SkipEq4 非零时跳过。
* @param kf 已初始化且增益已经计算好的滤波器。
 */
void Kalman_Filter_xhatUpdate(KalmanFilter_t *kf);
/**
* @brief 根据 Kalman 增益更新状态方差；SkipEq5 非零时跳过。
* @param kf 已初始化的滤波器。
 */
void Kalman_Filter_P_Update(KalmanFilter_t *kf);
/**
* @brief 按预测、测量修正和方差更新的顺序完成一轮计算。
* @param kf 已初始化并填好模型数据的滤波器。
* @return 指向内部状态结果数组的指针，内容会在下次更新时改写。
* @note 自动调整模式下，MeasuredVector 中等于0的项会被当作本轮无数据；矩阵运算状态写在 MatStatus 中。
*       返回指针由滤波器持有，调用者不要释放。
 */
float *Kalman_Filter_Update(KalmanFilter_t *kf);

#endif //__KALMAN_FILTER_H
