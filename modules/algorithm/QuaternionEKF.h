/**
* @file QuaternionEKF.h
* @brief 声明用陀螺仪和加速度计更新姿态的扩展卡尔曼滤波器。
 *
* 本文件定义滤波器状态和初始化、更新接口。调用更新接口前，先成功调用初始化接口。
* Roll、Pitch、Yaw 和 YawTotalAngle 以度表示；陀螺仪输入用 rad/s，加速度输入用 m/s²。
* @author Wang Hongxi
 * @version V1.2.0
* @date 2022/3/8
 */
#ifndef _QUAT_EKF_H
#define _QUAT_EKF_H
#include "kalman_filter.h"

/* 工程尚未定义时提供的布尔值常量。 */
#ifndef TRUE
#define TRUE 1 /**< 真 */
#endif

#ifndef FALSE
#define FALSE 0 /**< 假 */
#endif

/**
* @brief 保存姿态滤波器的输入、状态和输出。
* q 是姿态四元数；GyroBias 是陀螺仪零偏估计；角度输出以度表示。
 */
typedef struct
{
    uint8_t Initialized;
    KalmanFilter_t IMU_QuaternionEKF;
    uint8_t ConvergeFlag;
    uint8_t StableFlag;
    uint64_t ErrorCount;
    uint64_t UpdateCount;

    float q[4];        // 四元数估计值
    float GyroBias[3]; // 陀螺仪零偏估计值

    float Gyro[3];
    float Accel[3];

    float OrientationCosine[3];

    float accLPFcoef;
    float gyro_norm;
    float accl_norm;
    float AdaptiveGainScale;

    float Roll;
    float Pitch;
    float Yaw;

    float YawTotalAngle;

    float Q1; // 四元数更新过程噪声
    float Q2; // 陀螺仪零偏过程噪声
    float R;  // 加速度计测量噪声

    float dt; // 姿态更新周期
    mat ChiSquare;
    float ChiSquare_Data[1];      // 本轮加速度方向与预测方向的差距
    float ChiSquareTestThreshold; // 用来判断方向差距是否过大的界限
    float lambda;                 // 控制滤波器逐渐减少对旧零偏估计的依赖

    int16_t YawRoundCount;

    float YawAngleLast;
} QEKF_INS_t;

/** @brief 本模块使用的姿态滤波器状态。 */
extern QEKF_INS_t QEKF_INS;
extern float chiSquare;
extern float ChiSquareTestThreshold;
/**
* @brief 设置姿态滤波器并载入初始四元数。
* @param init_quaternion 指向至少4个 float 的数组，顺序为 q0、q1、q2、q3；不能为 NULL。
* @param process_noise1 四元数状态的过程噪声参数。
* @param process_noise2 陀螺仪零偏（静止时偏离0的读数）的过程噪声参数。
* @param measure_noise 加速度计测量噪声参数。
* @param lambda 调整旧零偏估计影响程度的系数；大于1时按1保存，函数不检查下限。
* @param lpf 加速度低通滤波时间参数，单位 s。
* @return 成功返回1；空指针或卡尔曼滤波器初始化失败时返回0。
* @note 参数没有范围检查。成功初始化后再调用 IMU_QuaternionEKF_Update。
 */
uint8_t IMU_QuaternionEKF_Init(float* init_quaternion,float process_noise1, float process_noise2, float measure_noise, float lambda, float lpf);
/**
* @brief 融合陀螺仪和加速度计，更新四元数及欧拉角。
* @param gx 陀螺仪 x 轴角速度，单位 rad/s。
* @param gy 陀螺仪 y 轴角速度，单位 rad/s。
* @param gz 陀螺仪 z 轴角速度，单位 rad/s。
* @param ax 加速度计 x 轴数据，单位 m/s²。
* @param ay 加速度计 y 轴数据，单位 m/s²。
* @param az 加速度计 z 轴数据，单位 m/s²。
* @param dt 两次更新之间的时间，单位 s。
* @note 本函数不检查初始化状态或输入范围；角度保存在 QEKF_INS 中。
 */
void IMU_QuaternionEKF_Update(float gx, float gy, float gz, float ax, float ay, float az, float dt);

#endif
