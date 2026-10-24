/**
 * @file attitude_filter.c
 * @brief 用统一结果和调用者工作区切换 QuaternionEKF、Fusion 姿态算法。
 * @author MC-02 contributors
 */
#include "attitude_filter.h"
#include "QuaternionEKF.h"
#if (defined(MC02_IMU_BACKEND_FUSION) && MC02_IMU_BACKEND_FUSION) || \
    (defined(MC02_ATTITUDE_FILTER_ENABLE_BOTH_BACKENDS) && MC02_ATTITUDE_FILTER_ENABLE_BOTH_BACKENDS)
#include "FusionAhrs.h"
#define ATTITUDE_FILTER_HAS_FUSION 1
#else
#define ATTITUDE_FILTER_HAS_FUSION 0
#endif
#include <float.h>
#include <math.h>
#include <string.h>

#define ATTITUDE_GRAVITY_M_S2 9.80665f
#define ATTITUDE_RAD_TO_DEG 57.29577951308232f
#define ATTITUDE_FILTER_MAGIC 0x41545446u

static size_t align_size(size_t value,size_t alignment)
{
    return (value + alignment - 1u) / alignment * alignment;
}

static int ranges_overlap(uintptr_t a,size_t a_bytes,uintptr_t b,size_t b_bytes)
{
    if(a_bytes==0u || b_bytes==0u) return 0;
    if(a>UINTPTR_MAX-a_bytes || b>UINTPTR_MAX-b_bytes) return 1;
    return a<b+b_bytes && b<a+a_bytes;
}

static int valid_qekf_config(const AttitudeFilterConfig_t *config)
{
    const AttitudeQekfConfig_t *q=&config->qekf;
    return isfinite(q->process_noise1) && q->process_noise1>=0.0f &&
        isfinite(q->process_noise2) && q->process_noise2>=0.0f &&
        isfinite(q->measure_noise) && q->measure_noise>0.0f &&
        isfinite(q->lambda) && q->lambda>0.0f && q->lambda<=1.0f &&
        isfinite(q->lpf) && q->lpf>=0.0f &&
        isfinite(q->variance_ratio) && q->variance_ratio>=1.0f;
}

#if ATTITUDE_FILTER_HAS_FUSION
static int valid_fusion_config(const AttitudeFilterConfig_t *config)
{
    const AttitudeFusionConfig_t *f=&config->fusion;
    const double recovery_samples=(double)f->sample_rate_hz*(double)f->rejection_timeout_s;
    return isfinite(f->sample_rate_hz) && f->sample_rate_hz>0.0f &&
        f->sample_rate_hz<=10000.0f &&
        isfinite(f->gain) && f->gain>=0.0f && f->gain<=10.0f &&
        isfinite(f->gyroscope_range_deg_s) && f->gyroscope_range_deg_s>=0.0f &&
        isfinite(f->acceleration_rejection_deg) && f->acceleration_rejection_deg>=0.0f &&
        f->acceleration_rejection_deg<180.0f &&
        isfinite(f->rejection_timeout_s) && f->rejection_timeout_s>=0.0f &&
        isfinite(recovery_samples) && recovery_samples<=(double)INT32_MAX-1024.0 &&
        (f->acceleration_rejection_deg==0.0f || recovery_samples>=1.0) &&
        isfinite(f->accel_min_m_s2) && f->accel_min_m_s2>=0.0f &&
        isfinite(f->accel_max_m_s2) && f->accel_max_m_s2>f->accel_min_m_s2;
}
#endif

static uint32_t increment_nonzero(uint32_t value)
{
    value++;
    return value==0u ? 1u : value;
}

static void set_invalid(AttitudeFilter_t *filter,KFStatus_t cause,uint32_t now_ms)
{
    filter->state.filter.mode=KF_INVALID;
    filter->state.filter.cause=cause;
    filter->state.filter.valid=0u;
    filter->state.filter.estimate_ms=now_ms;
    if(filter->state.filter.has_measurement)
        filter->state.filter.prediction_age_ms=now_ms-filter->state.filter.measurement_ms;
}

static void copy_qekf_state(AttitudeFilter_t *filter,const QEKF_INS_t *qekf,KFStatus_t status,uint32_t now_ms)
{
    const KFResult_t *result=&qekf->IMU_QuaternionEKF.result;
    filter->state.filter=*result;
    filter->state.filter.estimate_ms=now_ms;
    filter->state.reference_version=filter->reference_version;
    if(status!=KF_OK && status!=KF_PREDICTED) {
        filter->state.filter.mode=KF_INVALID;
        filter->state.filter.valid=0u;
        filter->state.filter.cause=status;
    }
    if(filter->state.filter.valid) {
        memcpy(filter->state.quaternion,qekf->q,sizeof(filter->state.quaternion));
        filter->state.roll_deg=qekf->Roll;
        filter->state.pitch_deg=qekf->Pitch;
        filter->state.yaw_deg=qekf->Yaw;
        filter->state.yaw_total_deg=qekf->YawTotalAngle;
    }
}

KFStatus_t AttitudeFilterGetWorkspaceRequirements(AttitudeBackend_t backend,size_t *bytes,size_t *alignment)
{
    if(bytes==NULL || alignment==NULL) return KF_ERROR_ARGUMENT;
    if(ranges_overlap((uintptr_t)bytes,sizeof(*bytes),(uintptr_t)alignment,sizeof(*alignment)))
        return KF_ERROR_ARGUMENT;
    size_t required;
    size_t required_alignment=_Alignof(max_align_t);
    if(backend==ATTITUDE_BACKEND_QUATERNION_EKF) {
        const size_t offset=align_size(sizeof(QEKF_INS_t),required_alignment);
        const size_t matrix_bytes=KF_WORKSPACE_BYTES(6u,0u,3u);
        if(offset>SIZE_MAX-matrix_bytes) return KF_ERROR_WORKSPACE;
        required=offset+matrix_bytes;
    }
#if ATTITUDE_FILTER_HAS_FUSION
    else if(backend==ATTITUDE_BACKEND_FUSION) {
        required=sizeof(FusionAhrs);
        if(required_alignment<_Alignof(FusionAhrs)) required_alignment=_Alignof(FusionAhrs);
    }
#endif
    else {
        return KF_ERROR_CONFIG;
    }
    if(required>ATTITUDE_FILTER_STORAGE_BYTES) return KF_ERROR_WORKSPACE;
    *bytes=required;
    *alignment=required_alignment;
    return KF_OK;
}

KFStatus_t AttitudeFilterInit(AttitudeFilter_t *filter,const AttitudeFilterConfig_t *config,
    const float initial_quaternion[4],void *workspace,size_t workspace_bytes)
{
    if(filter==NULL || config==NULL || initial_quaternion==NULL || workspace==NULL)
        return KF_ERROR_ARGUMENT;
    size_t required=0u,alignment=0u;
    KFStatus_t status=AttitudeFilterGetWorkspaceRequirements(config->backend,&required,&alignment);
    if(status!=KF_OK) return status;
    if(workspace_bytes<required || workspace_bytes>ATTITUDE_FILTER_STORAGE_BYTES ||
       (uintptr_t)workspace%alignment!=0u) return KF_ERROR_WORKSPACE;
    if(config->max_prediction_ms==0u || config->max_prediction_ms>INT32_MAX ||
       config->max_measurement_age_ms==0u || config->max_measurement_age_ms>INT32_MAX ||
       !isfinite(config->max_dt_s) || config->max_dt_s<=0.0f ||
       (config->backend==ATTITUDE_BACKEND_QUATERNION_EKF && !valid_qekf_config(config)) ||
        (config->backend==ATTITUDE_BACKEND_FUSION &&
#if ATTITUDE_FILTER_HAS_FUSION
         !valid_fusion_config(config)))
#else
         1))
#endif
        return KF_ERROR_CONFIG;

    float initial[4];
    double norm_sq=0.0;
    for(unsigned i=0u;i<4u;++i) {
        if(!isfinite(initial_quaternion[i])) return KF_ERROR_NUMERIC;
        norm_sq+=(double)initial_quaternion[i]*(double)initial_quaternion[i];
    }
    if(!isfinite(norm_sq) || norm_sq<1.0e-12) return KF_ERROR_NUMERIC;
    const float inverse_norm=(float)(1.0/sqrt(norm_sq));
    for(unsigned i=0u;i<4u;++i) initial[i]=initial_quaternion[i]*inverse_norm;

    const uintptr_t filter_address=(uintptr_t)filter;
    const uintptr_t workspace_address=(uintptr_t)workspace;
    const uintptr_t config_address=(uintptr_t)config;
    const uintptr_t initial_address=(uintptr_t)initial_quaternion;
    if(ranges_overlap(filter_address,sizeof(*filter),workspace_address,workspace_bytes) ||
       ranges_overlap(config_address,sizeof(*config),workspace_address,workspace_bytes) ||
       ranges_overlap(initial_address,4u*sizeof(float),workspace_address,workspace_bytes) ||
       ranges_overlap(filter_address,sizeof(*filter),config_address,sizeof(*config)) ||
       ranges_overlap(filter_address,sizeof(*filter),initial_address,4u*sizeof(float)))
        return KF_ERROR_WORKSPACE;

    AttitudeFilter_t prepared={0};
    prepared.initialized=ATTITUDE_FILTER_MAGIC;
    prepared.backend=config->backend;
    prepared.config=*config;
    prepared.workspace=workspace;
    prepared.workspace_bytes=workspace_bytes;
    prepared.reference_version=(filter->initialized==ATTITUDE_FILTER_MAGIC)
        ? increment_nonzero(filter->reference_version) : 1u;
    prepared.state.reference_version=prepared.reference_version;
    prepared.state.filter.mode=KF_INVALID;
    prepared.state.filter.cause=KF_NO_MEASUREMENT;
    memcpy(prepared.state.quaternion,initial,sizeof(initial));

    if(config->backend==ATTITUDE_BACKEND_QUATERNION_EKF) {
        const size_t offset=align_size(sizeof(QEKF_INS_t),_Alignof(max_align_t));
        QEKF_INS_t *qekf=(QEKF_INS_t *)workspace;
        const QEKFConfig_t qekf_config={
            .process_noise1=config->qekf.process_noise1,
            .process_noise2=config->qekf.process_noise2,
            .measure_noise=config->qekf.measure_noise,
            .lambda=config->qekf.lambda,
            .lpf=config->qekf.lpf,
            .max_prediction_ms=config->max_prediction_ms,
            .max_measurement_age_ms=config->max_measurement_age_ms,
            .variance_ratio=config->qekf.variance_ratio,
            .max_dt_s=config->max_dt_s,
        };
        status=IMU_QuaternionEKF_InstanceInit(qekf,initial,&qekf_config,
            (uint8_t *)workspace+offset,workspace_bytes-offset);
        if(status!=KF_OK) return status;
    }
#if ATTITUDE_FILTER_HAS_FUSION
    else {
        FusionAhrs candidate;
        FusionAhrsInitialise(&candidate);
        const AttitudeFusionConfig_t *f=&config->fusion;
        const FusionAhrsSettings settings={
            .sampleRate=f->sample_rate_hz,
            .convention=FusionConventionNwu,
            .gain=f->gain,
            .gyroscopeRange=f->gyroscope_range_deg_s,
            .accelerationRejection=f->acceleration_rejection_deg,
            .magneticRejection=0.0f,
            .rejectionTimeout=f->rejection_timeout_s,
        };
        FusionAhrsSetSettings(&candidate,&settings);
        // 起始四元数已由初始静止加速度建立，跳过库内启动阶段，避免暂时关闭加速度拒绝。
        FusionAhrsSkipStartup(&candidate);
        FusionAhrsSetQuaternion(&candidate,(FusionQuaternion){.array={initial[0],initial[1],initial[2],initial[3]}});
        memcpy(workspace,&candidate,sizeof(candidate));
    }
#endif
    *filter=prepared;
    return KF_OK;
}

static int is_newer_sample(const AttitudeFilter_t *filter,const AttitudeFilterInput_t *input)
{
    if(!filter->has_seen_sample) return 1;
    return (int32_t)(input->sequence-filter->last_seen_sequence)>0 &&
        (int32_t)(input->sample_time_ms-filter->last_seen_time_ms)>=0;
}

static void remember_sample(AttitudeFilter_t *filter,const AttitudeFilterInput_t *input)
{
    filter->last_seen_sequence=input->sequence;
    filter->last_seen_time_ms=input->sample_time_ms;
    filter->has_seen_sample=1u;
}

#if ATTITUDE_FILTER_HAS_FUSION
static int fusion_accel_usable(const AttitudeFilterConfig_t *config,const AttitudeFilterInput_t *input)
{
    if(input->accelerometer_valid==0u) return 0;
    double norm_sq=0.0;
    for(unsigned i=0u;i<3u;++i) {
        if(!isfinite(input->accelerometer_m_s2[i])) return 0;
        norm_sq+=(double)input->accelerometer_m_s2[i]*(double)input->accelerometer_m_s2[i];
    }
    const double magnitude=sqrt(norm_sq);
    const AttitudeFusionConfig_t *f=&config->fusion;
    if(!isfinite(magnitude) || magnitude<f->accel_min_m_s2 || magnitude>f->accel_max_m_s2) return 0;
    const uint32_t age=input->now_ms-input->sample_time_ms;
    if(age>config->max_measurement_age_ms || age>INT32_MAX) return 0;
    return 1;
}

static int fusion_accel_direction_accepted(const AttitudeFusionConfig_t *config,
    const FusionAhrs *ahrs,const AttitudeFilterInput_t *input)
{
    if(config->acceleration_rejection_deg==0.0f) return 1;
    const FusionVector gravity=FusionAhrsGetGravity(ahrs);
    double accel_norm_sq=0.0;
    double dot=0.0;
    for(unsigned i=0u;i<3u;++i) {
        const double acceleration=(double)input->accelerometer_m_s2[i];
        accel_norm_sq+=acceleration*acceleration;
        dot+=(double)gravity.array[i]*acceleration;
    }
    if(!isfinite(accel_norm_sq) || accel_norm_sq<=0.0 || !isfinite(dot)) return 0;
    const double cosine=dot/sqrt(accel_norm_sq);
    const double minimum_cosine=cos((double)config->acceleration_rejection_deg/ATTITUDE_RAD_TO_DEG);
    // 上游残差由叉积得到，完全反向时叉积为零；点积门控可拒绝这一退化方向。
    return isfinite(cosine) && cosine>=minimum_cosine;
}

static KFStatus_t update_fusion(AttitudeFilter_t *filter,const AttitudeFilterConfig_t *config,
    const AttitudeFilterInput_t *input,AttitudeFilterState_t *candidate_state)
{
    FusionAhrs *stored=(FusionAhrs *)filter->workspace;
    FusionAhrs candidate=*stored;
    FusionAhrsSetSamplePeriod(&candidate,input->dt_s);
    const FusionVector gyroscope={.array={
        input->gyroscope_rad_s[0]*ATTITUDE_RAD_TO_DEG,
        input->gyroscope_rad_s[1]*ATTITUDE_RAD_TO_DEG,
        input->gyroscope_rad_s[2]*ATTITUDE_RAD_TO_DEG}};
    if(!isfinite(gyroscope.array[0]) || !isfinite(gyroscope.array[1]) || !isfinite(gyroscope.array[2]))
        return KF_ERROR_NUMERIC;

    int accel_usable=fusion_accel_usable(config,input);
    if(accel_usable && !fusion_accel_direction_accepted(&config->fusion,&candidate,input))
        accel_usable=0;
    FusionVector accelerometer=FUSION_VECTOR_ZERO;
    if(accel_usable) {
        for(unsigned i=0u;i<3u;++i)
            accelerometer.array[i]=input->accelerometer_m_s2[i]/ATTITUDE_GRAVITY_M_S2;
    }
    FusionAhrsUpdateNoMagnetometer(&candidate,gyroscope,accelerometer);

    FusionAhrsInternalStates internal=FusionAhrsGetInternalStates(&candidate);
    FusionAhrsFlags flags=FusionAhrsGetFlags(&candidate);
    int corrected=accel_usable && !internal.accelerometerIgnored && !flags.accelerationRecovery;
    if(accel_usable && !corrected) {
        // 上游库会在连续拒绝后强行恢复加速度；回滚该候选，只提交本轮陀螺预测。
        candidate=*stored;
        FusionAhrsSetSamplePeriod(&candidate,input->dt_s);
        FusionAhrsUpdateNoMagnetometer(&candidate,gyroscope,FUSION_VECTOR_ZERO);
    }

    FusionQuaternion quaternion=FusionAhrsGetQuaternion(&candidate);
    double norm_sq=0.0;
    for(unsigned i=0u;i<4u;++i) {
        if(!isfinite(quaternion.array[i])) return KF_ERROR_NUMERIC;
        norm_sq+=(double)quaternion.array[i]*(double)quaternion.array[i];
    }
    if(!isfinite(norm_sq) || norm_sq<1.0e-12) return KF_ERROR_NUMERIC;
    const float inverse_norm=(float)(1.0/sqrt(norm_sq));
    for(unsigned i=0u;i<4u;++i) quaternion.array[i]*=inverse_norm;
    FusionAhrsSetQuaternion(&candidate,quaternion);

    const FusionEuler euler=FusionQuaternionToEuler(quaternion);
    for(unsigned i=0u;i<3u;++i)
        if(!isfinite(euler.array[i])) return KF_ERROR_NUMERIC;
    *stored=candidate;
    if(corrected) {
        filter->has_corrected_measurement=1u;
        filter->state.filter.has_measurement=1u;
        filter->state.filter.measurement_ms=input->sample_time_ms;
        filter->state.filter.prediction_age_ms=input->now_ms-input->sample_time_ms;
    }
    const uint32_t measurement_age=filter->has_corrected_measurement
        ? input->now_ms-filter->state.filter.measurement_ms : UINT32_MAX;
    if(!filter->has_corrected_measurement || measurement_age>config->max_prediction_ms || measurement_age>INT32_MAX) {
        candidate_state->filter=filter->state.filter;
        candidate_state->filter.mode=KF_INVALID;
        candidate_state->filter.cause=filter->has_corrected_measurement ? KF_PREDICTION_LIMIT : KF_NO_MEASUREMENT;
        candidate_state->filter.valid=0u;
        candidate_state->filter.estimate_ms=input->now_ms;
        candidate_state->filter.prediction_age_ms=filter->has_corrected_measurement ? measurement_age : 0u;
        candidate_state->reference_version=filter->reference_version;
        return candidate_state->filter.cause;
    }
    const float delta_yaw=euler.angle.yaw-filter->last_yaw_deg;
    float yaw_total=filter->state.yaw_total_deg;
    if(filter->has_published_orientation) {
        if(delta_yaw>180.0f) yaw_total-=360.0f;
        else if(delta_yaw< -180.0f) yaw_total+=360.0f;
    } else {
        yaw_total=euler.angle.yaw;
    }
    memcpy(candidate_state->quaternion,quaternion.array,sizeof(candidate_state->quaternion));
    candidate_state->roll_deg=euler.angle.roll;
    candidate_state->pitch_deg=euler.angle.pitch;
    candidate_state->yaw_deg=euler.angle.yaw;
    candidate_state->yaw_total_deg=yaw_total;
    candidate_state->filter=filter->state.filter;
    candidate_state->filter.mode=corrected ? KF_CORRECTED : KF_PREDICTION;
    candidate_state->filter.cause=corrected ? KF_OK : KF_PREDICTED;
    candidate_state->filter.valid=1u;
    candidate_state->filter.estimate_ms=input->now_ms;
    candidate_state->filter.prediction_age_ms=measurement_age;
    candidate_state->reference_version=filter->reference_version;
    filter->last_yaw_deg=euler.angle.yaw;
    filter->has_published_orientation=1u;
    return corrected ? KF_OK : KF_PREDICTED;
}
#endif

KFStatus_t AttitudeFilterUpdate(AttitudeFilter_t *filter,const AttitudeFilterInput_t *input,
    AttitudeFilterState_t *state)
{
    if(filter==NULL || input==NULL || state==NULL) return KF_ERROR_ARGUMENT;
    if(filter->initialized!=ATTITUDE_FILTER_MAGIC) return KF_ERROR_STATE;
    if(ranges_overlap((uintptr_t)state,sizeof(*state),(uintptr_t)filter,sizeof(*filter)) ||
       ranges_overlap((uintptr_t)state,sizeof(*state),(uintptr_t)filter->workspace,filter->workspace_bytes) ||
       ranges_overlap((uintptr_t)input,sizeof(*input),(uintptr_t)filter,sizeof(*filter)) ||
       ranges_overlap((uintptr_t)input,sizeof(*input),(uintptr_t)filter->workspace,filter->workspace_bytes))
        return KF_ERROR_WORKSPACE;
    if(filter->fatal!=0u) {
        filter->state.filter.mode=KF_INVALID;
        filter->state.filter.valid=0u;
        filter->state.filter.estimate_ms=input->now_ms;
        *state=filter->state;
        return KF_ERROR_STATE;
    }
    AttitudeFilterState_t result=filter->state;
    if(!isfinite(input->dt_s) || input->dt_s<=0.0f || input->dt_s>filter->config.max_dt_s) {
        set_invalid(filter,KF_ERROR_TIME,input->now_ms);
        *state=filter->state;
        return KF_ERROR_TIME;
    }
    // dt上限按初始化配置保存在后端；两种实现都拒绝超过本轮配置的间隔。
    for(unsigned i=0u;i<3u;++i) {
        if(!isfinite(input->gyroscope_rad_s[i])) {
            set_invalid(filter,KF_ERROR_NUMERIC,input->now_ms);
            *state=filter->state;
            return KF_ERROR_NUMERIC;
        }
    }
    if(!is_newer_sample(filter,input)) {
        set_invalid(filter,(input->sequence==filter->last_seen_sequence)
            ? KF_DUPLICATE_MEASUREMENT : KF_STALE_MEASUREMENT,input->now_ms);
        *state=filter->state;
        return filter->state.filter.cause;
    }

    KFStatus_t status;
    if(filter->backend==ATTITUDE_BACKEND_QUATERNION_EKF) {
        QEKF_INS_t *qekf=(QEKF_INS_t *)filter->workspace;
        float acceleration[3];
        int measurement_ok=input->accelerometer_valid!=0u;
        for(unsigned i=0u;i<3u;++i) {
            acceleration[i]=input->accelerometer_m_s2[i];
            if(!isfinite(acceleration[i])) measurement_ok=0;
        }
        const uint32_t sample_age=input->now_ms-input->sample_time_ms;
        if(sample_age>filter->config.max_measurement_age_ms || sample_age>INT32_MAX) measurement_ok=0;
        if(measurement_ok==0) acceleration[0]=acceleration[1]=acceleration[2]=NAN;
        status=IMU_QuaternionEKF_InstanceUpdate(qekf,input->gyroscope_rad_s[0],input->gyroscope_rad_s[1],
            input->gyroscope_rad_s[2],acceleration[0],acceleration[1],acceleration[2],input->dt_s,
            input->now_ms,input->sample_time_ms,input->sequence);
        copy_qekf_state(filter,qekf,status,input->now_ms);
        if(filter->state.filter.has_measurement) filter->has_corrected_measurement=1u;
        filter->last_yaw_deg=filter->state.yaw_deg;
    }
#if ATTITUDE_FILTER_HAS_FUSION
    else {
        // 上限不保存在调用者输入中；运行时按Fusion采样率可接受的最大周期核对。
        status=update_fusion(filter,&filter->config,input,&result);
    }
#else
    else {
        return KF_ERROR_CONFIG;
    }
#endif
    if(status==KF_ERROR_NUMERIC || status==KF_ERROR_MATRIX)
        filter->fatal=1u;
    if(status!=KF_OK && status!=KF_PREDICTED && status!=KF_PREDICTION_LIMIT &&
       status!=KF_NO_MEASUREMENT && filter->backend==ATTITUDE_BACKEND_FUSION) {
        result=filter->state;
        result.filter.mode=KF_INVALID;
        result.filter.cause=status;
        result.filter.valid=0u;
        result.filter.estimate_ms=input->now_ms;
    }
    remember_sample(filter,input);
    if(filter->backend==ATTITUDE_BACKEND_QUATERNION_EKF) result=filter->state;
    else filter->state=result;
    *state=result;
    return status;
}

KFStatus_t AttitudeFilterGetState(const AttitudeFilter_t *filter,AttitudeFilterState_t *state)
{
    if(filter==NULL || state==NULL) return KF_ERROR_ARGUMENT;
    if(filter->initialized!=ATTITUDE_FILTER_MAGIC) return KF_ERROR_STATE;
    if(ranges_overlap((uintptr_t)state,sizeof(*state),(uintptr_t)filter,sizeof(*filter)) ||
       ranges_overlap((uintptr_t)state,sizeof(*state),(uintptr_t)filter->workspace,filter->workspace_bytes))
        return KF_ERROR_WORKSPACE;
    *state=filter->state;
    return KF_OK;
}
