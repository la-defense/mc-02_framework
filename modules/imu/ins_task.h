/**
* @file ins_task.h
* @brief 声明 IMU 姿态解算任务和姿态数据。
* StartINSTASK 在 INS 专属任务中调用 INS_Init，随后以1kHz调用 INS_Task。
* 角度使用度，角速度使用 rad/s，加速度使用 m/s²。
* @author Wang Hongxi
* @author annotation and modification by NeoZeng
 * @version V2.0.0
* @date 2022/2/23
 */
#ifndef __INS_TASK_H
#define __INS_TASK_H

#include "stdint.h"
#include "QuaternionEKF.h"

/* 用 enum 而非宏定义坐标轴索引, 避免 X/Y/Z 宏污染全局预处理器
 * (曾与 CMSIS core_cm7.h 位域成员 Z 冲突) */
enum
{
    X = 0,
    Y = 1,
    Z = 2
};

#define INS_TASK_PERIOD 1

typedef struct
{
    float Gyro[3];  // x/y/z 轴角速度，单位 rad/s
    float Accel[3]; // x/y/z 轴加速度，单位 m/s²，包含重力
    float Roll;
    float Pitch;
    float Yaw;
    float YawTotalAngle;
} attitude_t; // 姿态角单位度；YawTotalAngle 会累积多圈转动

typedef struct
{
    float q[4]; // q0、q1、q2、q3 四元数估计值

    float MotionAccel_b[3]; // 去掉重力后的机体坐标系加速度，单位 m/s²
    float MotionAccel_n[3]; // 同一加速度转到导航坐标系后的结果，单位 m/s²

    float AccelLPF; // 加速度低通滤波时间参数，单位 s

    // bodyframe在绝对系的向量表示
    float xn[3];
    float yn[3];
    float zn[3];

    // 加速度在机体系和XY两轴的夹角
    // float atanxz;
    // float atanyz;

    // IMU 测量数据
    float Gyro[3];  // x/y/z 轴角速度，单位 rad/s
    float Accel[3]; // x/y/z 轴加速度，单位 m/s²
    // 位姿
    float Roll;
    float Pitch;
    float Yaw;
    float YawTotalAngle;

    uint8_t init;
} INS_t;

/* 用于修正安装误差的参数 */
typedef struct
{
    uint8_t flag;

    float scale[3];

    float Yaw;
    float Pitch;
    float Roll;
} IMU_Param_t;

/**
* @brief 注册 BMI088 并准备姿态解算所需的滤波器。
* @return 返回内部姿态数据地址；卡尔曼滤波器初始化失败时返回 NULL。
* @note 由 StartINSTASK 在 INS 专属任务中调用。BMI088 初始化失败时仍返回地址，
*       但 IMU 状态会标记为无效，应用不得使用旧姿态作为有效数据。
 */
attitude_t *INS_Init(void);

/**
* @brief 读取一份新样本，更新姿态和视觉上行数据。
* @note 由 INS 专属任务每1ms调用；样本无效时清除有效状态并向视觉发送零姿态。
 */
void INS_Task(void);

/**
* @brief 取得 INS 内部姿态数据的地址，不会重新初始化。
* @return INS 开始初始化后返回内部地址；调用者不要释放，内容由 INS_Task 更新。
 */
attitude_t *INS_GetAttitude(void);

/** @brief 在 INS 任务启动前取得长期有效的内部存储地址；此时内容尚未准备好。 */
attitude_t *INS_GetStorage(void);

/**
* @brief 根据陀螺仪角速度推进四元数。
* @param q 至少包含4个可写 float，顺序为 q0、q1、q2、q3。
* @param gx x轴角速度，单位 rad/s。
* @param gy y轴角速度，单位 rad/s。
* @param gz z轴角速度，单位 rad/s。
* @param dt 与上次更新的间隔，单位 s。
* @note 不检查空指针或数组长度。
 */
void QuaternionUpdate(float *q, float gx, float gy, float gz, float dt);

/**
* @brief 将四元数换成 ZYX 顺序的偏航、俯仰和横滚角。
* @param q 至少包含4个 float，顺序为 q0、q1、q2、q3。
* @param Yaw 接收偏航角的可写地址，单位度。
* @param Pitch 接收俯仰角的可写地址，单位度。
* @param Roll 接收横滚角的可写地址，单位度。
* @note 不检查空指针或数组长度。
 */
void QuaternionToEularAngle(float *q, float *Yaw, float *Pitch, float *Roll);

/**
* @brief 将 ZYX 顺序的偏航、俯仰和横滚角换成四元数。
* @param Yaw 偏航角，单位度。
* @param Pitch 俯仰角，单位度。
* @param Roll 横滚角，单位度。
* @param q 接收 q0、q1、q2、q3 的可写数组，至少包含4个 float。
* @note 不检查空指针或数组长度。
 */
void EularAngleToQuaternion(float Yaw, float Pitch, float Roll, float *q);

/**
* @brief 使用四元数把三维向量从机体坐标系转到导航坐标系。
* @param vecBF 至少包含3个 float 的输入数组。
* @param vecEF 至少包含3个 float 的可写输出数组。
* @param q 至少包含4个 float 的姿态四元数。
* @note 不检查空指针或数组长度。
 */
void BodyFrameToEarthFrame(const float *vecBF, float *vecEF, float *q);

/**
* @brief 使用四元数把三维向量从导航坐标系转到机体坐标系。
* @param vecEF 至少包含3个 float 的输入数组。
* @param vecBF 至少包含3个 float 的可写输出数组。
* @param q 至少包含4个 float 的姿态四元数。
* @note 不检查空指针或数组长度。
 */
void EarthFrameToBodyFrame(const float *vecEF, float *vecBF, float *q);

#endif
