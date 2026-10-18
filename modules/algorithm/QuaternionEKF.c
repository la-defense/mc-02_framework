/**
* @file QuaternionEKF.c
* @brief 根据陀螺仪预测姿态，再用加速度计修正，并估计陀螺仪零偏。
* @author Wang Hongxi
 * @version V1.2.0
* @date 2022/3/8
 */
#include "QuaternionEKF.h"
#include "dsp_matrix.h"
#include <math.h>
#include <float.h>
#include <stdint.h>
#include <string.h>

#if FLT_RADIX != 2 || FLT_MANT_DIG != 24 || FLT_MAX_EXP != 128
#error "QuaternionEKF invSqrt requires IEEE-754 binary32 float"
#endif

_Static_assert(sizeof(float) == sizeof(uint32_t), "invSqrt requires 32-bit float");

QEKF_INS_t QEKF_INS;

const float IMU_QuaternionEKF_F[36] = {1, 0, 0, 0, 0, 0,
                                       0, 1, 0, 0, 0, 0,
                                       0, 0, 1, 0, 0, 0,
                                       0, 0, 0, 1, 0, 0,
                                       0, 0, 0, 0, 1, 0,
                                       0, 0, 0, 0, 0, 1};
float IMU_QuaternionEKF_P[36] = {100000, 0.1, 0.1, 0.1, 0.1, 0.1,
                                 0.1, 100000, 0.1, 0.1, 0.1, 0.1,
                                 0.1, 0.1, 100000, 0.1, 0.1, 0.1,
                                 0.1, 0.1, 0.1, 100000, 0.1, 0.1,
                                 0.1, 0.1, 0.1, 0.1, 100, 0.1,
                                 0.1, 0.1, 0.1, 0.1, 0.1, 100};
float IMU_QuaternionEKF_K[18];
float IMU_QuaternionEKF_H[18];

static const float k_initial_P[36] = {100000, 0.1, 0.1, 0.1, 0.1, 0.1,
                                 0.1, 100000, 0.1, 0.1, 0.1, 0.1,
                                 0.1, 0.1, 100000, 0.1, 0.1, 0.1,
                                 0.1, 0.1, 0.1, 100000, 0.1, 0.1,
                                 0.1, 0.1, 0.1, 0.1, 100, 0.1,
                                 0.1, 0.1, 0.1, 0.1, 0.1, 100};
static float invSqrt(float x);
static KFStatus_t IMU_QuaternionEKF_F_Linearization_P_Fading(KalmanFilter_t *,void *);
static KFStatus_t IMU_QuaternionEKF_SetH(KalmanFilter_t *,void *);
static KFStatus_t IMU_QuaternionEKF_xhatUpdate(KalmanFilter_t *,void *);
// 每次矩阵调用立即检查，后续成功不能覆盖前一次失败。
static KFStatus_t check_matrix(arm_status status,const arm_matrix_instance_f32 *output)
{
    if(status!=ARM_MATH_SUCCESS) return KF_ERROR_MATRIX;
    for(size_t i=0;i<(size_t)output->numRows*output->numCols;++i)
        if(!isfinite(output->pData[i])) return KF_ERROR_NUMERIC;
    return KF_OK;
}
#define Q_MATRIX(call,output) do { KFStatus_t s=check_matrix((call),(output)); if(s!=KF_OK) return s; } while(0)

KFStatus_t IMU_QuaternionEKF_Init(const float *initial,const QEKFConfig_t *c,void *workspace,size_t bytes)
{
    if(initial==NULL || c==NULL) return KF_ERROR_ARGUMENT;
    if(!isfinite(c->process_noise1) || c->process_noise1<0.0f ||
       !isfinite(c->process_noise2) || c->process_noise2<0.0f ||
       !isfinite(c->measure_noise) || c->measure_noise<=0.0f ||
       !isfinite(c->lambda) || c->lambda<=0.0f || c->lambda>1.0f ||
       !isfinite(c->lpf) || c->lpf<0.0f) return KF_ERROR_CONFIG;
    float norm=0.0f;
    for(unsigned i=0;i<4u;++i) { if(!isfinite(initial[i])) return KF_ERROR_NUMERIC; norm+=initial[i]*initial[i]; }
    if(!isfinite(norm) || norm<1e-6f) return KF_ERROR_NUMERIC;
    float state[6]={initial[0],initial[1],initial[2],initial[3],0,0};
    QEKF_INS_t prepared={0};
    KFConfig_t config={6u,0u,3u,c->max_prediction_ms,20u,c->variance_ratio,c->max_dt_s,
        IMU_QuaternionEKF_F_Linearization_P_Fading,IMU_QuaternionEKF_SetH,IMU_QuaternionEKF_xhatUpdate};
    // 不允许工作区盖住公开实例，否则初始化清零会破坏旧结果。
    uintptr_t w=(uintptr_t)workspace,q=(uintptr_t)&QEKF_INS;
    if(bytes>UINTPTR_MAX-w || (w<q+sizeof(QEKF_INS) && q<w+bytes)) return KF_ERROR_WORKSPACE;
    uintptr_t a=(uintptr_t)initial,b=(uintptr_t)c;
    if(a>UINTPTR_MAX-4u*sizeof(float) || b>UINTPTR_MAX-sizeof(*c) ||
       (w<a+4u*sizeof(float) && a<w+bytes) || (w<b+sizeof(*c) && b<w+bytes)) return KF_ERROR_WORKSPACE;
    KFStatus_t status=KFInit(&prepared.IMU_QuaternionEKF,&config,workspace,bytes,state,k_initial_P);
    if(status!=KF_OK) return status;
    prepared.Q1=c->process_noise1; prepared.Q2=c->process_noise2; prepared.R=c->measure_noise;
    prepared.lambda=c->lambda; prepared.accLPFcoef=c->lpf;
    prepared.ChiSquareTestThreshold=1e-8f; prepared.Initialized=1u;
    memcpy(prepared.q,initial,4u*sizeof(float));
    memcpy(prepared.IMU_QuaternionEKF.F_data,IMU_QuaternionEKF_F,sizeof(IMU_QuaternionEKF_F));
    prepared.ChiSquare=(arm_matrix_instance_f32){1,1,prepared.ChiSquare_Data};
    QEKF_INS=prepared;
    // 结构复制后重新绑定自身数组，避免留下栈上对象的地址。
    QEKF_INS.ChiSquare.pData=QEKF_INS.ChiSquare_Data;
    return KF_OK;
}

KFStatus_t IMU_QuaternionEKF_Update(float gx,float gy,float gz,float ax,float ay,float az,
    float dt,uint32_t now_ms,uint32_t measurement_ms,uint32_t sequence)
{
    if(!QEKF_INS.Initialized) return KF_ERROR_STATE;
    QEKF_INS_t trial=QEKF_INS;
    QEKF_INS_t *owner=&trial;
    owner->ChiSquare.pData=owner->ChiSquare_Data;
    // 0.5(Ohm-Ohm^bias)*deltaT,用于更新工作点处的状态转移F矩阵
    float halfgxdt, halfgydt, halfgzdt;
    float accelInvNorm;

    /*   F, number with * represent vals to be set
     0      1*     2*     3*     4     5
     6*     7      8*     9*    10    11
    12*    13*    14     15*    16    17
    18*    19*    20*    21     22    23
    24     25     26     27     28    29
    30     31     32     33     34    35
    */
    owner->dt = dt;

    owner->Gyro[0] = gx - owner->GyroBias[0];
    owner->Gyro[1] = gy - owner->GyroBias[1];
    owner->Gyro[2] = gz - owner->GyroBias[2];

    // set F
    halfgxdt = 0.5f * owner->Gyro[0] * dt;
    halfgydt = 0.5f * owner->Gyro[1] * dt;
    halfgzdt = 0.5f * owner->Gyro[2] * dt;

    // 此部分设定状态转移矩阵F的左上角部分 4x4子矩阵,即0.5(Ohm-Ohm^bias)*deltaT,右下角有一个2x2单位阵已经初始化好了
    // 注意在predict步F的右上角是4x2的零矩阵,因此每次predict的时候都会调用memcpy用单位阵覆盖前一轮线性化后的矩阵
    memcpy(owner->IMU_QuaternionEKF.F_data, IMU_QuaternionEKF_F, sizeof(IMU_QuaternionEKF_F));

    owner->IMU_QuaternionEKF.F_data[1] = -halfgxdt;
    owner->IMU_QuaternionEKF.F_data[2] = -halfgydt;
    owner->IMU_QuaternionEKF.F_data[3] = -halfgzdt;

    owner->IMU_QuaternionEKF.F_data[6] = halfgxdt;
    owner->IMU_QuaternionEKF.F_data[8] = halfgzdt;
    owner->IMU_QuaternionEKF.F_data[9] = -halfgydt;

    owner->IMU_QuaternionEKF.F_data[12] = halfgydt;
    owner->IMU_QuaternionEKF.F_data[13] = -halfgzdt;
    owner->IMU_QuaternionEKF.F_data[15] = halfgxdt;

    owner->IMU_QuaternionEKF.F_data[18] = halfgzdt;
    owner->IMU_QuaternionEKF.F_data[19] = halfgydt;
    owner->IMU_QuaternionEKF.F_data[20] = -halfgxdt;

    int accel_finite=isfinite(ax)&&isfinite(ay)&&isfinite(az);
    float rawNormSq=ax*ax+ay*ay+az*az;
    accel_finite=accel_finite && isfinite(rawNormSq) && rawNormSq>=1e-6f;
    if(accel_finite)
    {
        float values[3]={ax,ay,az};
        if(owner->UpdateCount==0u) memcpy(owner->Accel,values,sizeof(values));
        for(unsigned i=0;i<3u;++i)
            owner->Accel[i]=owner->Accel[i]*owner->accLPFcoef/(dt+owner->accLPFcoef)+values[i]*dt/(dt+owner->accLPFcoef);
    }
    // 把加速度数据按向量长度缩放到长度为1，供滤波器比较方向。
    float accelNormSq = owner->Accel[0] * owner->Accel[0] +
                        owner->Accel[1] * owner->Accel[1] +
                        owner->Accel[2] * owner->Accel[2];
    uint16_t measurements=0u;
    // 无效加速度只表示本轮不能校正，不伪造重力向量。
    if(accel_finite && isfinite(accelNormSq) && accelNormSq>=1e-6f)
    {
        accelInvNorm=invSqrt(accelNormSq);
        for(unsigned i=0;i<3u;++i) owner->IMU_QuaternionEKF.MeasuredVector[i]=owner->Accel[i]*accelInvNorm;
        owner->accl_norm=1.0f/accelInvNorm; measurements=3u;
    }
    else owner->accl_norm=0.0f;
    // get body state
    owner->gyro_norm = 1.0f / invSqrt(owner->Gyro[0] * owner->Gyro[0] +
                                        owner->Gyro[1] * owner->Gyro[1] +
                                        owner->Gyro[2] * owner->Gyro[2]);
    // 如果角速度小于阈值且加速度处于设定范围内,认为运动稳定,加速度可以用于修正角速度
    // 稍后在最后的姿态更新部分会利用StableFlag来确定
    if (owner->gyro_norm < 0.3f && owner->accl_norm > 9.8f - 0.5f && owner->accl_norm < 9.8f + 0.5f)
    {
        owner->StableFlag = 1;
    }
    else
    {
        owner->StableFlag = 0;
    }

    // set Q R,过程噪声和观测噪声矩阵
    owner->IMU_QuaternionEKF.Q_data[0] = owner->Q1 * owner->dt;
    owner->IMU_QuaternionEKF.Q_data[7] = owner->Q1 * owner->dt;
    owner->IMU_QuaternionEKF.Q_data[14] = owner->Q1 * owner->dt;
    owner->IMU_QuaternionEKF.Q_data[21] = owner->Q1 * owner->dt;
    owner->IMU_QuaternionEKF.Q_data[28] = owner->Q2 * owner->dt;
    owner->IMU_QuaternionEKF.Q_data[35] = owner->Q2 * owner->dt;
    owner->IMU_QuaternionEKF.R_data[0] = owner->R;
    owner->IMU_QuaternionEKF.R_data[4] = owner->R;
    owner->IMU_QuaternionEKF.R_data[8] = owner->R;


    KFStep_t step={dt,now_ms,measurement_ms,sequence,owner->IMU_QuaternionEKF.MeasuredVector,NULL,measurements,owner};
    KFStatus_t status=KFUpdate(&QEKF_INS.IMU_QuaternionEKF,&step);
    owner->IMU_QuaternionEKF=QEKF_INS.IMU_QuaternionEKF;
    if(status==KF_OK || status==KF_PREDICTED)
    {
    // 获取融合后的数据,包括四元数和xy零飘值
    owner->q[0] = owner->IMU_QuaternionEKF.FilteredValue[0];
    owner->q[1] = owner->IMU_QuaternionEKF.FilteredValue[1];
    owner->q[2] = owner->IMU_QuaternionEKF.FilteredValue[2];
    owner->q[3] = owner->IMU_QuaternionEKF.FilteredValue[3];
    owner->GyroBias[0] = owner->IMU_QuaternionEKF.FilteredValue[4];
    owner->GyroBias[1] = owner->IMU_QuaternionEKF.FilteredValue[5];
    owner->GyroBias[2] = 0; // 大部分时候z轴通天,无法观测yaw的漂移

    // 利用四元数反解欧拉角
    owner->Yaw = atan2f(2.0f * (owner->q[0] * owner->q[3] + owner->q[1] * owner->q[2]), 2.0f * (owner->q[0] * owner->q[0] + owner->q[1] * owner->q[1]) - 1.0f) * 57.295779513f;
    owner->Pitch = atan2f(2.0f * (owner->q[0] * owner->q[1] + owner->q[2] * owner->q[3]), 2.0f * (owner->q[0] * owner->q[0] + owner->q[3] * owner->q[3]) - 1.0f) * 57.295779513f;
    // 浮点舍入可能略过±1；输入四元数已在候选步骤检查长度。
    owner->Roll = asinf(fmaxf(-1.0f,fminf(1.0f,-2.0f * (owner->q[1] * owner->q[3] - owner->q[0] * owner->q[2])))) * 57.295779513f;

    // 角度跨过 ±180° 时调整圈数，保留连续累计的 yaw 角。
    if (owner->Yaw - owner->YawAngleLast > 180.0f)
    {
        owner->YawRoundCount--;
    }
    else if (owner->Yaw - owner->YawAngleLast < -180.0f)
    {
        owner->YawRoundCount++;
    }
    owner->YawTotalAngle = 360.0f * owner->YawRoundCount + owner->Yaw;
    owner->YawAngleLast = owner->Yaw;


        if(!isfinite(owner->Yaw) || !isfinite(owner->Pitch) || !isfinite(owner->Roll))
        {
            QEKF_INS.IMU_QuaternionEKF.fatal=1u;
            QEKF_INS.IMU_QuaternionEKF.result.valid=0u;
            QEKF_INS.IMU_QuaternionEKF.result.mode=KF_INVALID;
            QEKF_INS.IMU_QuaternionEKF.result.cause=KF_ERROR_NUMERIC;
            return KF_ERROR_NUMERIC;
        }
    }
    if(isfinite(owner->dt) && owner->dt>0.0f) owner->UpdateCount++;
    QEKF_INS=*owner; QEKF_INS.ChiSquare.pData=QEKF_INS.ChiSquare_Data;
    memcpy(IMU_QuaternionEKF_P,QEKF_INS.IMU_QuaternionEKF.published_P,sizeof(IMU_QuaternionEKF_P));
    memcpy(IMU_QuaternionEKF_K,QEKF_INS.IMU_QuaternionEKF.K_data,sizeof(IMU_QuaternionEKF_K));
    memcpy(IMU_QuaternionEKF_H,QEKF_INS.IMU_QuaternionEKF.H_data,sizeof(IMU_QuaternionEKF_H));
    return status;
}
static KFStatus_t IMU_QuaternionEKF_F_Linearization_P_Fading(KalmanFilter_t *kf,void *context)
{
    QEKF_INS_t *owner=(QEKF_INS_t *)context;

    float q0, q1, q2, q3;
    float qInvNorm;

    q0 = kf->xhatminus_data[0];
    q1 = kf->xhatminus_data[1];
    q2 = kf->xhatminus_data[2];
    q3 = kf->xhatminus_data[3];

    // 把预测四元数按长度缩放到长度为1，避免计算中长度逐渐偏离。
    float norm=q0*q0+q1*q1+q2*q2+q3*q3;
    if(!isfinite(norm) || norm<1e-6f) return KF_ERROR_NUMERIC;
    qInvNorm = invSqrt(norm);
    for (uint8_t i = 0; i < 4; ++i)
    {
        kf->xhatminus_data[i] *= qInvNorm;
    }
    /*  F, number with * represent vals to be set
     0     1     2     3     4*     5*
     6     7     8     9    10*    11*
    12    13    14    15    16*    17*
    18    19    20    21    22*    23*
    24    25    26    27    28     29
    30    31    32    33    34     35
    */
    // set F
    // 把四元数与 x、y 轴零偏之间的变化关系写进 F 矩阵右上角。
    kf->F_data[4] = q1 * owner->dt / 2;
    kf->F_data[5] = q2 * owner->dt / 2;

    kf->F_data[10] = -q0 * owner->dt / 2;
    kf->F_data[11] = q3 * owner->dt / 2;

    kf->F_data[16] = -q3 * owner->dt / 2;
    kf->F_data[17] = -q0 * owner->dt / 2;

    kf->F_data[22] = q2 * owner->dt / 2;
    kf->F_data[23] = -q1 * owner->dt / 2;

    // 放大零偏估计的方差，避免滤波器长期过度相信旧估计。
    kf->P_data[28] /= owner->lambda;
    kf->P_data[35] /= owner->lambda;

    // 限制两个零偏方差的最大值，避免它们无限增大。
    if (kf->P_data[28] > 10000)
    {
        kf->P_data[28] = 10000;
    }
    if (kf->P_data[35] > 10000)
    {
        kf->P_data[35] = 10000;
    }

    return KF_OK;
}

/**
* @brief 根据预测姿态计算 H 矩阵，说明姿态变化怎样影响重力方向测量。
* @param kf 当前滤波器及其预测结果。
 */
static KFStatus_t IMU_QuaternionEKF_SetH(KalmanFilter_t *kf,void *context)
{
    (void)context;

    float doubleq0, doubleq1, doubleq2, doubleq3;
    /* H
     0     1     2     3     4     5
     6     7     8     9    10    11
    12    13    14    15    16    17
    last two cols are zero
    */
    // set H
    doubleq0 = 2 * kf->xhatminus_data[0];
    doubleq1 = 2 * kf->xhatminus_data[1];
    doubleq2 = 2 * kf->xhatminus_data[2];
    doubleq3 = 2 * kf->xhatminus_data[3];

    memset(kf->H_data, 0, sizeof(float) * kf->zSize * kf->xhatSize);

    kf->H_data[0] = -doubleq2;
    kf->H_data[1] = doubleq3;
    kf->H_data[2] = -doubleq0;
    kf->H_data[3] = doubleq1;

    kf->H_data[6] = doubleq1;
    kf->H_data[7] = doubleq0;
    kf->H_data[8] = doubleq3;
    kf->H_data[9] = doubleq2;

    kf->H_data[12] = doubleq0;
    kf->H_data[13] = -doubleq1;
    kf->H_data[14] = -doubleq2;
    kf->H_data[15] = doubleq3;

    return KF_OK;
}

/**
* @brief 比较预测的重力方向与加速度计数据，再修正姿态和零偏。
* @param kf 当前滤波器及其预测结果、测量数据。
 */
static KFStatus_t IMU_QuaternionEKF_xhatUpdate(KalmanFilter_t *kf,void *context)
{
    QEKF_INS_t *owner=(QEKF_INS_t *)context;

    float q0, q1, q2, q3;

    Q_MATRIX(arm_mat_trans_f32(&kf->H, &kf->HT),&kf->HT); // z|x => x|z
    kf->temp_matrix.numRows = kf->H.numRows;
    kf->temp_matrix.numCols = kf->Pminus.numCols;
    Q_MATRIX(arm_mat_mult_f32(&kf->H, &kf->Pminus, &kf->temp_matrix),&kf->temp_matrix); // temp_matrix = H·P'(k)
    kf->temp_matrix1.numRows = kf->temp_matrix.numRows;
    kf->temp_matrix1.numCols = kf->HT.numCols;
    Q_MATRIX(arm_mat_mult_f32(&kf->temp_matrix, &kf->HT, &kf->temp_matrix1),&kf->temp_matrix1); // temp_matrix1 = H·P'(k)·HT
    kf->S.numRows = kf->R.numRows;
    kf->S.numCols = kf->R.numCols;
    Q_MATRIX(arm_mat_add_f32(&kf->temp_matrix1, &kf->R, &kf->S),&kf->S); // S = H P'(k) HT + R
    Q_MATRIX(DSPMatrixInverseF32(&kf->S,&kf->temp_matrix1,kf->inverse_workspace,18u),&kf->temp_matrix1);     // temp_matrix1 = inv(H·P'(k)·HT + R)

    q0 = kf->xhatminus_data[0];
    q1 = kf->xhatminus_data[1];
    q2 = kf->xhatminus_data[2];
    q3 = kf->xhatminus_data[3];

    kf->temp_vector.numRows = kf->H.numRows;
    kf->temp_vector.numCols = 1;
    // 计算预测得到的重力加速度方向(通过姿态获取的)
    kf->temp_vector_data[0] = 2 * (q1 * q3 - q0 * q2);
    kf->temp_vector_data[1] = 2 * (q0 * q1 + q2 * q3);
    kf->temp_vector_data[2] = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3; // temp_vector = h(xhat'(k))

    // 求预测重力方向与各坐标轴的夹角，后面用它调整零偏修正力度。
    for (uint8_t i = 0; i < 3; ++i)
    {
        owner->OrientationCosine[i] = acosf(fminf(1.0f,fabsf(kf->temp_vector_data[i])));
    }

    // 用加速度计数据修正预测姿态。
    kf->temp_vector1.numRows = kf->z.numRows;
    kf->temp_vector1.numCols = 1;
    Q_MATRIX(arm_mat_sub_f32(&kf->z, &kf->temp_vector, &kf->temp_vector1),&kf->temp_vector1); // temp_vector1 = z(k) - h(xhat'(k))

    // 计算测量方向与预测方向的差距，并结合噪声大小判断本轮测量是否可信。
    kf->temp_matrix.numRows = kf->temp_vector1.numRows;
    kf->temp_matrix.numCols = 1;
    Q_MATRIX(arm_mat_mult_f32(&kf->temp_matrix1, &kf->temp_vector1, &kf->temp_matrix),&kf->temp_matrix); // 把测量方向的差距按当前估计和噪声大小换算，作为后面的差距检查值。
    kf->temp_vector.numRows = 1;
    kf->temp_vector.numCols = kf->temp_vector1.numRows;
    Q_MATRIX(arm_mat_trans_f32(&kf->temp_vector1, &kf->temp_vector),&kf->temp_vector); // temp_vector = z(k) - h(xhat'(k))'
    Q_MATRIX(arm_mat_mult_f32(&kf->temp_vector, &kf->temp_matrix, &owner->ChiSquare),&owner->ChiSquare);
    // 预测方向和加速度计方向接近时，记录滤波器已进入收敛状态。
    if (owner->ChiSquare_Data[0] < 0.5f * owner->ChiSquareTestThreshold)
    {
        owner->ConvergeFlag = 1;
    }
    // 已拒绝的方向不能因计数达到阈值而变成可信量测。
    if (owner->ChiSquare_Data[0] > owner->ChiSquareTestThreshold && owner->ConvergeFlag)
    {
        if(owner->StableFlag) owner->ErrorCount++;
        else owner->ErrorCount=0u;
        return KF_CORRECTION_REJECTED;
    }
    else // 误差还可接受或滤波器尚未收敛时，按方向差距调整加速度修正力度。
    {
        // 方向差距越大，修正力度越小；差距较小时保留完整修正。
        if (owner->ChiSquare_Data[0] > 0.1f * owner->ChiSquareTestThreshold && owner->ConvergeFlag)
        {
            owner->AdaptiveGainScale = (owner->ChiSquareTestThreshold - owner->ChiSquare_Data[0]) / (0.9f * owner->ChiSquareTestThreshold);
        }
        else
        {
            owner->AdaptiveGainScale = 1;
        }
        owner->ErrorCount = 0;
    }

    // cal kf-gain K
    kf->temp_matrix.numRows = kf->Pminus.numRows;
    kf->temp_matrix.numCols = kf->HT.numCols;
    Q_MATRIX(arm_mat_mult_f32(&kf->Pminus, &kf->HT, &kf->temp_matrix),&kf->temp_matrix); // temp_matrix = P'(k)·HT
    Q_MATRIX(arm_mat_mult_f32(&kf->temp_matrix, &kf->temp_matrix1, &kf->K),&kf->K);

    // 对所有增益元素应用上面算出的修正力度。
    for (uint8_t i = 0; i < kf->K.numRows * kf->K.numCols; ++i)
    {
        kf->K_data[i] *= owner->AdaptiveGainScale;
    }
    for (uint8_t i = 4; i < 6; ++i)
    {
        for (uint8_t j = 0; j < 3; ++j)
        {
            kf->K_data[i * 3 + j] *= owner->OrientationCosine[i - 4] / 1.5707963f; // 1 rad
        }
    }

    kf->temp_vector.numRows = kf->K.numRows;
    kf->temp_vector.numCols = 1;
    Q_MATRIX(arm_mat_mult_f32(&kf->K, &kf->temp_vector1, &kf->temp_vector),&kf->temp_vector); // temp_vector = K(k)·(z(k) - H·xhat'(k))

    // 收敛后限制每次零偏改变量，避免单次测量让估计跳得太远。
    if (owner->ConvergeFlag)
    {
        for (uint8_t i = 4; i < 6; ++i)
        {
            if (kf->temp_vector.pData[i] > 1e-2f * owner->dt)
            {
                kf->temp_vector.pData[i] = 1e-2f * owner->dt;
            }
            if (kf->temp_vector.pData[i] < -1e-2f * owner->dt)
            {
                kf->temp_vector.pData[i] = -1e-2f * owner->dt;
            }
        }
    }

    // 加速度计只能提供重力方向，无法单独判断绕重力方向的转动，因此不修正 yaw。
    // 加速度计看不到绕重力方向的转动，这一分量不由它修正。
    kf->temp_vector.pData[3] = 0;
    Q_MATRIX(arm_mat_add_f32(&kf->xhatminus, &kf->temp_vector, &kf->xhat),&kf->xhat);
    float norm=0.0f;
    for(unsigned i=0;i<4u;++i) norm+=kf->xhat_data[i]*kf->xhat_data[i];
    // 校正产生退化或过大四元数时使用已检查的预测，不能发布异常姿态。
    if(!isfinite(norm) || norm<1e-6f || norm>4.0f) return KF_ERROR_NUMERIC;
    return KF_OK;
}

/**
 * @brief 自定义1/sqrt(x),速度更快
 *
 * @param x x
 * @return float
 */
static float invSqrt(float x)
{
    float halfx = 0.5f * x;
    float y = x;
    uint32_t bits;
    memcpy(&bits, &y, sizeof(bits));
    // 用浮点数的位模式给倒平方根一个初值，再用一次牛顿迭代细化。
    bits = UINT32_C(0x5f375a86) - (bits >> 1u);
    memcpy(&y, &bits, sizeof(y));
    y = y * (1.5f - (halfx * y * y));
    return y;
}
