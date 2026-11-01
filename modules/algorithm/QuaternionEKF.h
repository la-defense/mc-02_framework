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
    DSPMatrixF32_t ChiSquare; // 指向本轮方向差距的 1×1 矩阵。
    float ChiSquare_Data[1];      // 本轮加速度方向与预测方向的差距
    float ChiSquareTestThreshold; // 用来判断方向差距是否过大的界限
    float lambda;                 // 控制滤波器逐渐减少对旧零偏估计的依赖

    int16_t YawRoundCount;

    float YawAngleLast;
} QEKF_INS_t;

/** @brief 本模块使用的姿态滤波器状态。 */
extern QEKF_INS_t QEKF_INS;
/** @brief 启动固定的六状态／三量测EKF；预测时限及方差比由调用方显式配置。 */
typedef struct {
    float process_noise1,process_noise2,measure_noise,lambda,lpf;
    uint32_t max_prediction_ms,max_measurement_age_ms;
    float variance_ratio,max_dt_s;
} QEKFConfig_t;
/** @brief 初始化调用者保存的独立 EKF 实例，不分配内存。
 * @param[in,out] instance 长期保存的滤波器状态；不能与工作区、配置或初值重叠。
 * @param[in] initial_quaternion 4个有限float，模长非零，顺序w/x/y/z。
 * @param[in] config 合法噪声、时间和预测方差配置；成功后复制。
 * @param[in,out] workspace KF_WORKSPACE_BYTES(6,0,3)字节，满足KF_WORKSPACE_ALIGNMENT。
 * @param[in] bytes 工作区可写容量。
 * @return KF_OK或具体错误；失败不改写实例和工作区，成功后等待首次校正。
 */
KFStatus_t IMU_QuaternionEKF_InstanceInit(QEKF_INS_t *instance,const float *initial_quaternion,
    const QEKFConfig_t *config,void *workspace,size_t bytes);
/** @brief 更新指定EKF实例；不同实例可并行，同一实例须由调用者串行访问。
 * @param[in,out] instance 已初始化实例。
 * @param[in] gx 陀螺仪x轴角速度，单位rad/s。
 * @param[in] gy 陀螺仪y轴角速度，单位rad/s。
 * @param[in] gz 陀螺仪z轴角速度，单位rad/s。
 * @param[in] ax 加速度x轴分量，单位m/s²；任一分量非有限表示本轮没有量测。
 * @param[in] ay 加速度y轴分量，单位m/s²。
 * @param[in] az 加速度z轴分量，单位m/s²。
 * @param[in] dt_s 本机实测间隔，单位秒。
 * @param[in] now_ms 当前毫秒时刻。
 * @param[in] measurement_ms 样本源毫秒时刻；与now_ms使用同一32位时钟并允许回绕。
 * @param[in] sequence 样本序号；重复或过期样本被拒绝。
 * @return KF_OK、KF_PREDICTED或具体错误；失败保留旧数值并撤销有效标记。
 */
KFStatus_t IMU_QuaternionEKF_InstanceUpdate(QEKF_INS_t *instance,float gx,float gy,float gz,
    float ax,float ay,float az,float dt_s,uint32_t now_ms,uint32_t measurement_ms,uint32_t sequence);
/** @brief 接受起点、配置及工作区，不分配内存。
 * @param[in] initial_quaternion 4个有限float，模长非零，顺序w/x/y/z。
 * @param[in] config Q1/Q2非负，R>0，0<lambda<=1，lpf>=0；所有浮点参数有限。
 * @param[in,out] workspace KF_WORKSPACE_BYTES(6,0,3)字节，按KF_WORKSPACE_ALIGNMENT对齐，长期独占。
 * @param[in] bytes 实际可写字节数，不得与参数或QEKF_INS重叠。
 * @return KF_OK或具体错误；失败保持原实例，成功后等待首次校正。
 */
KFStatus_t IMU_QuaternionEKF_Init(const float *initial_quaternion,const QEKFConfig_t *config,
    void *workspace,size_t bytes);
/** @brief 用新样本推进EKF，返回校正、有限预测或无效；严重预测失败保持无效直到重新初始化。
 * @param[in] gx x轴角速度，rad/s，必须有限。
 * @param[in] gy y轴角速度，rad/s，必须有限。
 * @param[in] gz z轴角速度，rad/s，必须有限。
 * @param[in] ax x轴加速度，m/s²；无效时仅尝试预测。
 * @param[in] ay y轴加速度，m/s²。
 * @param[in] az z轴加速度，m/s²。
 * @param[in] dt_s 实测间隔，秒，须>0且不超过配置上限。
 * @param[in] now_ms 本机当前毫秒数。
 * @param[in] measurement_ms 原始样本的本机接收毫秒数，预测不能延长它。
 * @param[in] sequence 原始样本序号，相同序号不能再次用于校正。
 * @return KF_OK、KF_PREDICTED或负数；无效时q和角度保留旧值，result.valid清零。
 */
KFStatus_t IMU_QuaternionEKF_Update(float gx,float gy,float gz,float ax,float ay,float az,
    float dt_s,uint32_t now_ms,uint32_t measurement_ms,uint32_t sequence);
#endif
