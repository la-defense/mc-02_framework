/** @file kalman_filter.c
 * @brief 用固定工作区完成KF预测、校正和结果发布；公开用法见头文件。
 * @author Wang Hongxi（原公式）；本轮将分配、临时数据和发布步骤分开。
 */
#include "kalman_filter.h"
#include "dsp_matrix.h"
#include <math.h>
#include <string.h>
#include <limits.h>
#define KF_INITIALIZED 0x4b463031u

// 地址和实际使用字节数都先检查，防止地址加法回到零后误判不重叠。
static int overlaps(const void *a,size_t an,const void *b,size_t bn)
{
    uintptr_t ap=(uintptr_t)a,bp=(uintptr_t)b;
    if(an>UINTPTR_MAX-ap || bn>UINTPTR_MAX-bp) return 1;
    return an!=0u && bn!=0u && ap<bp+bn && bp<ap+an;
}
static int finite_values(const float *data,size_t count)
{
    if(data==NULL && count!=0u) return 0;
    for(size_t i=0;i<count;++i) if(!isfinite(data[i])) return 0;
    return 1;
}
static int covariance_valid(const float *data,size_t n)
{
    if(!finite_values(data,n*n)) return 0;
    for(size_t i=0;i<n;++i) if(data[i*n+i]<0.0f) return 0;
    return 1;
}
static int configured(const KFConfig_t *c)
{
    return c!=NULL && c->state_count>0u && c->state_count<=255u && c->control_count<=255u &&
        c->measurement_capacity>0u && c->measurement_capacity<=255u &&
        c->max_prediction_ms<=INT32_MAX && c->max_measurement_age_ms<=INT32_MAX &&
        isfinite(c->max_dt_s) && c->max_dt_s>0.0f &&
        isfinite(c->max_prediction_variance_ratio) && c->max_prediction_variance_ratio>=1.0f;
}
static int initialized(const KalmanFilter_t *f)
{ return f!=NULL && f->initialized==KF_INITIALIZED; }
KFStatus_t KFGetWorkspaceRequirements(const KFConfig_t *c,size_t *bytes,size_t *alignment)
{
    if(c==NULL || bytes==NULL || alignment==NULL) return KF_ERROR_ARGUMENT;
    if(overlaps(bytes,sizeof(*bytes),alignment,sizeof(*alignment)) ||
       overlaps(bytes,sizeof(*bytes),c,sizeof(*c)) || overlaps(alignment,sizeof(*alignment),c,sizeof(*c)))
        return KF_ERROR_ARGUMENT;
    if(!configured(c)) return KF_ERROR_CONFIG;
    // 维度先限制到255，再按size_t计算，32位平台同样不会乘法溢出。
    *bytes=KF_WORKSPACE_BYTES(c->state_count,c->control_count,c->measurement_capacity);
    *alignment=KF_WORKSPACE_ALIGNMENT;
    return KF_OK;
}
static void matrix(arm_matrix_instance_f32 *m,uint16_t rows,uint16_t cols,float *data)
{ arm_mat_init_f32(m,rows,cols,data); }
static void bind_matrices(KalmanFilter_t *f)
{
    uint16_t n=f->xhatSize,u=f->uSize,m=f->zSize,d=(uint16_t)KF_MAX_DIM(n,m);
    matrix(&f->xhat,n,1,f->xhat_data); matrix(&f->P,n,n,f->P_data);
    matrix(&f->xhatminus,n,1,f->xhatminus_data); matrix(&f->Pminus,n,n,f->Pminus_data);
    matrix(&f->F,n,n,f->F_data); matrix(&f->FT,n,n,f->FT_data); matrix(&f->Q,n,n,f->Q_data);
    matrix(&f->B,n,u,f->B_data); matrix(&f->u,u,1,f->u_data);
    matrix(&f->H,m,n,f->H_data); matrix(&f->HT,n,m,f->HT_data); matrix(&f->K,n,m,f->K_data);
    matrix(&f->R,m,m,f->R_data); matrix(&f->S,m,m,f->S_data); matrix(&f->z,m,1,f->z_data);
    matrix(&f->temp_matrix,d,d,f->temp_matrix_data); matrix(&f->temp_matrix1,d,d,f->temp_matrix_data1);
    matrix(&f->temp_vector,d,1,f->temp_vector_data); matrix(&f->temp_vector1,d,1,f->temp_vector_data1);
}
KFStatus_t KFInit(KalmanFilter_t *f,const KFConfig_t *c,void *workspace,size_t bytes,
    const float *state,const float *covariance)
{
    size_t required,alignment;
    if(f==NULL || workspace==NULL || state==NULL || covariance==NULL) return KF_ERROR_ARGUMENT;
    KFStatus_t status=KFGetWorkspaceRequirements(c,&required,&alignment);
    if(status!=KF_OK) return status;
    size_t n=c->state_count,m=c->measurement_capacity,u=c->control_count,d=KF_MAX_DIM(n,m);
    if(bytes<required || (uintptr_t)workspace%alignment!=0u ||
       overlaps(workspace,required,f,sizeof(*f)) || overlaps(workspace,required,c,sizeof(*c)) ||
       overlaps(workspace,required,state,n*sizeof(float)) ||
       overlaps(workspace,required,covariance,n*n*sizeof(float)) ||
       overlaps(f,sizeof(*f),state,n*sizeof(float)) || overlaps(f,sizeof(*f),covariance,n*n*sizeof(float)))
        return KF_ERROR_WORKSPACE;
    if(!finite_values(state,n) || !covariance_valid(covariance,n)) return KF_ERROR_NUMERIC;
    KalmanFilter_t prepared={0};
    prepared.config=*c; prepared.xhatSize=(uint16_t)n; prepared.uSize=(uint16_t)u; prepared.zSize=(uint16_t)m;
    prepared.workspace=workspace; prepared.workspace_bytes=required;
    float *cursor=(float *)workspace;
#define TAKE(field,count) do { prepared.field=cursor; cursor+=(count); } while(0)
    TAKE(xhat_data,n); TAKE(P_data,n*n); TAKE(trial_x,n); TAKE(trial_P,n*n);
    TAKE(xhatminus_data,n); TAKE(Pminus_data,n*n); TAKE(FilteredValue,n); TAKE(published_P,n*n);
    TAKE(corrected_variance,n); TAKE(F_data,n*n); TAKE(FT_data,n*n); TAKE(Q_data,n*n);
    TAKE(B_data,n*u); TAKE(H_data,m*n); TAKE(HT_data,n*m); TAKE(K_data,n*m);
    TAKE(R_data,m*m); TAKE(S_data,m*m); TAKE(selected_H,m*n); TAKE(selected_R,m*m);
    TAKE(temp_matrix_data,d*d); TAKE(temp_matrix_data1,d*d);
    TAKE(temp_vector_data,d); TAKE(temp_vector_data1,d); TAKE(MeasuredVector,m); TAKE(z_data,m);
    TAKE(ControlVector,u); TAKE(u_data,u); TAKE(StateMinVariance,n); TAKE(inverse_workspace,2u*m*m);
#undef TAKE
    prepared.selected_rows=(uint8_t *)cursor;
    // 所有输入和容量通过检查后才清工作区、绑定实例，不留下半套初始化数据。
    memset(workspace,0,required);
    memcpy(prepared.xhat_data,state,n*sizeof(float)); memcpy(prepared.FilteredValue,state,n*sizeof(float));
    memcpy(prepared.P_data,covariance,n*n*sizeof(float)); memcpy(prepared.published_P,covariance,n*n*sizeof(float));
    prepared.initialized=KF_INITIALIZED; bind_matrices(&prepared); *f=prepared;
    return KF_OK;
}
KFStatus_t KFReset(KalmanFilter_t *f,const float *state,const float *covariance)
{
    if(!initialized(f)) return KF_ERROR_STATE;
    size_t n=f->xhatSize;
    if(state==NULL || covariance==NULL) return KF_ERROR_ARGUMENT;
    if(overlaps(f->workspace,f->workspace_bytes,state,n*sizeof(float)) ||
       overlaps(f->workspace,f->workspace_bytes,covariance,n*n*sizeof(float)) ||
       overlaps(f,sizeof(*f),state,n*sizeof(float)) || overlaps(f,sizeof(*f),covariance,n*n*sizeof(float)))
        return KF_ERROR_WORKSPACE;
    if(!finite_values(state,n) || !covariance_valid(covariance,n)) return KF_ERROR_NUMERIC;
    memcpy(f->xhat_data,state,n*sizeof(float)); memcpy(f->FilteredValue,state,n*sizeof(float));
    memcpy(f->P_data,covariance,n*n*sizeof(float)); memcpy(f->published_P,covariance,n*n*sizeof(float));
    memset(f->corrected_variance,0,n*sizeof(float));
    // 模型F/B/H/Q/R和最小方差保留；计算用数组清零，旧增益及输入不带入新起点。
    size_t m=f->config.measurement_capacity,u=f->uSize,d=KF_MAX_DIM(n,m);
#define CLEAR(field,count) memset(f->field,0,(count)*sizeof(float))
    CLEAR(trial_x,n); CLEAR(trial_P,n*n); CLEAR(xhatminus_data,n); CLEAR(Pminus_data,n*n);
    CLEAR(FT_data,n*n); CLEAR(HT_data,n*m); CLEAR(K_data,n*m); CLEAR(S_data,m*m);
    CLEAR(selected_H,n*m); CLEAR(selected_R,m*m);
    CLEAR(temp_matrix_data,d*d); CLEAR(temp_matrix_data1,d*d);
    CLEAR(temp_vector_data,d); CLEAR(temp_vector_data1,d); CLEAR(MeasuredVector,m); CLEAR(z_data,m);
    CLEAR(ControlVector,u); CLEAR(u_data,u); CLEAR(inverse_workspace,2u*m*m);
#undef CLEAR
    memset(f->selected_rows,0,m);
    f->result=(KFResult_t){0}; f->has_time=0u; f->has_seen=0u; f->fatal=0u;
    f->last_step_ms=0u; f->last_seen_ms=0u; f->last_seen_sequence=0u;
    return KF_OK;
}
static KFStatus_t invalid(KalmanFilter_t *f,KFStatus_t cause,int fatal)
{
    f->result.valid=0u; f->result.mode=KF_INVALID; f->result.cause=cause;
    if(fatal) f->fatal=1u;
    return cause;
}
static KFStatus_t checked(arm_status status,const arm_matrix_instance_f32 *out)
{
    if(status!=ARM_MATH_SUCCESS) return KF_ERROR_MATRIX;
    return finite_values(out->pData,(size_t)out->numRows*out->numCols)?KF_OK:KF_ERROR_NUMERIC;
}
// 扩展步骤只能成功或返回失败；正数不能冒充已发布的有限预测。
static KFStatus_t stage_status(KFStatus_t status)
{ return status>KF_OK?KF_ERROR_STATE:status; }
#define MATRIX(expression,out) do { KFStatus_t s=checked((expression),(out)); if(s!=KF_OK) return s; } while(0)
static KFStatus_t predict(KalmanFilter_t *f,const KFStep_t *step)
{
    size_t n=f->xhatSize,u=f->uSize;
    if(!finite_values(f->F_data,n*n) || !finite_values(f->Q_data,n*n) ||
       !finite_values(f->ControlVector,u) || !finite_values(f->B_data,n*u)) return KF_ERROR_NUMERIC;
    MATRIX(arm_mat_mult_f32(&f->F,&f->xhat,&f->xhatminus),&f->xhatminus);
    if(u!=0u)
    {
        memcpy(f->u_data,f->ControlVector,u*sizeof(float));
        matrix(&f->temp_vector,(uint16_t)n,1,f->temp_vector_data);
        MATRIX(arm_mat_mult_f32(&f->B,&f->u,&f->temp_vector),&f->temp_vector);
        MATRIX(arm_mat_add_f32(&f->xhatminus,&f->temp_vector,&f->xhatminus),&f->xhatminus);
    }
    if(f->config.after_state_prediction!=NULL)
    {
        KFStatus_t status=stage_status(f->config.after_state_prediction(f,step->callback_context));
        if(status!=KF_OK) return status;
    }
    if(!finite_values(f->xhatminus_data,n) || !covariance_valid(f->P_data,n) ||
       !finite_values(f->F_data,n*n)) return KF_ERROR_NUMERIC;
    MATRIX(arm_mat_trans_f32(&f->F,&f->FT),&f->FT);
    matrix(&f->temp_matrix,(uint16_t)n,(uint16_t)n,f->temp_matrix_data);
    MATRIX(arm_mat_mult_f32(&f->F,&f->P,&f->temp_matrix),&f->temp_matrix);
    MATRIX(arm_mat_mult_f32(&f->temp_matrix,&f->FT,&f->Pminus),&f->Pminus);
    MATRIX(arm_mat_add_f32(&f->Pminus,&f->Q,&f->Pminus),&f->Pminus);
    return covariance_valid(f->Pminus_data,n)?KF_OK:KF_ERROR_NUMERIC;
}
static KFStatus_t linear_correction(KalmanFilter_t *f)
{
    uint16_t n=f->xhatSize,m=f->z.numRows;
    MATRIX(arm_mat_trans_f32(&f->H,&f->HT),&f->HT);
    matrix(&f->temp_matrix,m,n,f->temp_matrix_data);
    MATRIX(arm_mat_mult_f32(&f->H,&f->Pminus,&f->temp_matrix),&f->temp_matrix);
    matrix(&f->temp_matrix1,m,m,f->temp_matrix_data1);
    MATRIX(arm_mat_mult_f32(&f->temp_matrix,&f->HT,&f->temp_matrix1),&f->temp_matrix1);
    MATRIX(arm_mat_add_f32(&f->temp_matrix1,&f->R,&f->S),&f->S);
    MATRIX(DSPMatrixInverseF32(&f->S,&f->temp_matrix1,f->inverse_workspace,
                              2u*(size_t)f->config.measurement_capacity*f->config.measurement_capacity),&f->temp_matrix1);
    matrix(&f->temp_matrix,n,m,f->temp_matrix_data);
    MATRIX(arm_mat_mult_f32(&f->Pminus,&f->HT,&f->temp_matrix),&f->temp_matrix);
    MATRIX(arm_mat_mult_f32(&f->temp_matrix,&f->temp_matrix1,&f->K),&f->K);
    matrix(&f->temp_vector,m,1,f->temp_vector_data); matrix(&f->temp_vector1,m,1,f->temp_vector_data1);
    MATRIX(arm_mat_mult_f32(&f->H,&f->xhatminus,&f->temp_vector),&f->temp_vector);
    MATRIX(arm_mat_sub_f32(&f->z,&f->temp_vector,&f->temp_vector1),&f->temp_vector1);
    matrix(&f->temp_vector,n,1,f->temp_vector_data);
    MATRIX(arm_mat_mult_f32(&f->K,&f->temp_vector1,&f->temp_vector),&f->temp_vector);
    MATRIX(arm_mat_add_f32(&f->xhatminus,&f->temp_vector,&f->xhat),&f->xhat);
    return KF_OK;
}
static KFStatus_t correct(KalmanFilter_t *f,const KFStep_t *step,uint16_t active)
{
    size_t n=f->xhatSize,maximum=f->config.measurement_capacity;
    if(f->config.prepare_measurement!=NULL)
    {
        KFStatus_t status=stage_status(f->config.prepare_measurement(f,step->callback_context));
        if(status!=KF_OK) return status;
    }
    // 把存在的量测行及其R子矩阵复制到独立数组，合法零值不会被丢弃。
    for(size_t i=0;i<active;++i)
    {
        size_t row=f->selected_rows[i];
        for(size_t j=0;j<n;++j) f->selected_H[i*n+j]=f->H_data[row*n+j];
        for(size_t j=0;j<active;++j) f->selected_R[i*active+j]=f->R_data[row*maximum+f->selected_rows[j]];
    }
    if(!finite_values(f->selected_H,(size_t)active*n) || !finite_values(f->selected_R,(size_t)active*active))
        return KF_ERROR_NUMERIC;
    f->H_data=f->selected_H; f->R_data=f->selected_R;
    matrix(&f->H,active,(uint16_t)n,f->H_data); matrix(&f->R,active,active,f->R_data);
    matrix(&f->HT,(uint16_t)n,active,f->HT_data); matrix(&f->K,(uint16_t)n,active,f->K_data);
    matrix(&f->S,active,active,f->S_data); matrix(&f->z,active,1,f->z_data); f->zSize=active;
    KFStatus_t status=f->config.correction!=NULL?stage_status(f->config.correction(f,step->callback_context)):linear_correction(f);
    if(status!=KF_OK) return status;
    matrix(&f->temp_matrix,(uint16_t)n,(uint16_t)n,f->temp_matrix_data);
    matrix(&f->temp_matrix1,(uint16_t)n,(uint16_t)n,f->temp_matrix_data1);
    MATRIX(arm_mat_mult_f32(&f->K,&f->H,&f->temp_matrix),&f->temp_matrix);
    MATRIX(arm_mat_mult_f32(&f->temp_matrix,&f->Pminus,&f->temp_matrix1),&f->temp_matrix1);
    MATRIX(arm_mat_sub_f32(&f->Pminus,&f->temp_matrix1,&f->P),&f->P);
    for(size_t i=0;i<n;++i)
    {
        if(!isfinite(f->StateMinVariance[i]) || f->StateMinVariance[i]<0.0f) return KF_ERROR_CONFIG;
        if(f->P_data[i*n+i]<f->StateMinVariance[i]) f->P_data[i*n+i]=f->StateMinVariance[i];
    }
    return finite_values(f->xhat_data,n) && covariance_valid(f->P_data,n)?KF_OK:KF_ERROR_NUMERIC;
}
#undef MATRIX
KFStatus_t KFUpdate(KalmanFilter_t *f,const KFStep_t *step)
{
    if(!initialized(f)) return KF_ERROR_STATE;
    if(f->fatal) return KF_ERROR_STATE; // 保留导致停用的原始原因，方便诊断。
    if(step==NULL) return invalid(f,KF_ERROR_ARGUMENT,0);
    if(overlaps(step,sizeof(*step),f,sizeof(*f)) || overlaps(step,sizeof(*step),f->workspace,f->workspace_bytes))
        return invalid(f,KF_ERROR_ARGUMENT,0);
    if(!isfinite(step->dt_s) || step->dt_s<=0.0f || step->dt_s>f->config.max_dt_s ||
       (f->has_time && (uint32_t)(step->now_ms-f->last_step_ms)>INT32_MAX))
        return invalid(f,KF_ERROR_TIME,1);
    if(step->measurement_count>f->config.measurement_capacity ||
       (step->measurement_count!=0u && step->measurements==NULL)) return invalid(f,KF_ERROR_ARGUMENT,0);
    size_t n=f->xhatSize;
    KalmanFilter_t candidate=*f;
    memcpy(f->trial_x,f->xhat_data,n*sizeof(float)); memcpy(f->trial_P,f->P_data,n*n*sizeof(float));
    candidate.xhat_data=f->trial_x; candidate.P_data=f->trial_P; bind_matrices(&candidate);
    KFStatus_t cause=KF_NO_MEASUREMENT;
    uint16_t active=0u;
    for(uint16_t i=0;i<step->measurement_count;++i)
    {
        if(step->measurement_present!=NULL && step->measurement_present[i]>1u)
            return invalid(f,KF_ERROR_ARGUMENT,0);
        if(step->measurement_present==NULL || step->measurement_present[i]!=0u)
        { candidate.selected_rows[active]=(uint8_t)i; candidate.z_data[active++]=step->measurements[i]; }
    }
    // 先接收量测，再使用临时矩阵，避免量测位于模型存储内时被预测覆盖。
    KFStatus_t status=predict(&candidate,step);
    if(status!=KF_OK) return invalid(f,status,1);
    if(active!=0u)
    {
        uint32_t age=step->now_ms-step->measurement_ms;
        if(age>f->config.max_measurement_age_ms ||
           (f->has_seen && (uint32_t)(step->measurement_ms-f->last_seen_ms)>INT32_MAX)) cause=KF_STALE_MEASUREMENT;
        else if(f->has_seen && step->measurement_sequence==f->last_seen_sequence) cause=KF_DUPLICATE_MEASUREMENT;
        else if(f->has_seen && (uint32_t)(step->measurement_sequence-f->last_seen_sequence)>INT32_MAX)
            cause=KF_STALE_MEASUREMENT;
        else
        {
            f->last_seen_ms=step->measurement_ms; f->last_seen_sequence=step->measurement_sequence; f->has_seen=1u;
            cause=finite_values(candidate.z_data,active)?correct(&candidate,step,active):KF_ERROR_NUMERIC;
        }
    }
    int corrected=cause==KF_OK;
    // 校正失败时只接受本轮已经验证过的预测，不能拿部分校正矩阵冒充预测。
    const float *next_x=corrected?candidate.xhat_data:candidate.xhatminus_data;
    const float *next_P=corrected?candidate.P_data:candidate.Pminus_data;
    // 扩展回调失败后仍检查预测，避免回调误改预测数组后发布非有限值。
    if(!finite_values(next_x,n) || !covariance_valid(next_P,n)) return invalid(f,KF_ERROR_NUMERIC,1);
    memcpy(f->xhat_data,next_x,n*sizeof(float)); memcpy(f->P_data,next_P,n*n*sizeof(float));
    uint32_t elapsed_ms=f->has_time?step->now_ms-f->last_step_ms:0u;
    f->last_step_ms=step->now_ms; f->has_time=1u;
    if(corrected)
    {
        f->result.has_measurement=1u; f->result.measurement_ms=step->measurement_ms;
        for(size_t i=0;i<n;++i) f->corrected_variance[i]=next_P[i*n+i];
    }
    if(corrected) f->result.prediction_age_ms=step->now_ms-step->measurement_ms;
    else if(f->result.has_measurement)
    {
        // 年龄累加到最大值后保持，防止长期无校正跨过整轮毫秒计数后重新变有效。
        uint64_t age=(uint64_t)f->result.prediction_age_ms+elapsed_ms;
        f->result.prediction_age_ms=age>UINT32_MAX?UINT32_MAX:(uint32_t)age;
    }
    else f->result.prediction_age_ms=UINT32_MAX;
    int usable=corrected || (f->result.has_measurement && f->result.prediction_age_ms<=f->config.max_prediction_ms);
    // 比值以上次成功校正为固定基准，不能被每轮预测逐步抬高。
    if(!corrected && usable)
        for(size_t i=0;i<n;++i)
            if((double)next_P[i*n+i]>(double)f->corrected_variance[i]*f->config.max_prediction_variance_ratio) usable=0;
    if(!usable) return invalid(f,KF_PREDICTION_LIMIT,0);
    memcpy(f->FilteredValue,next_x,n*sizeof(float)); memcpy(f->published_P,next_P,n*n*sizeof(float));
    f->result.valid=1u; f->result.mode=corrected?KF_CORRECTED:KF_PREDICTION;
    f->result.cause=cause; f->result.estimate_ms=step->now_ms;
    return corrected?KF_OK:KF_PREDICTED;
}
KFStatus_t KFGetState(const KalmanFilter_t *f,float *state,size_t state_capacity,
    float *covariance,size_t covariance_capacity,KFResult_t *result)
{
    if(!initialized(f)) return KF_ERROR_STATE;
    if(state==NULL || covariance==NULL || result==NULL) return KF_ERROR_ARGUMENT;
    size_t n=f->xhatSize,sb=n*sizeof(float),pb=n*n*sizeof(float);
    if(state_capacity<n || covariance_capacity<n*n) return KF_ERROR_WORKSPACE;
    if(overlaps(state,sb,covariance,pb) || overlaps(state,sb,result,sizeof(*result)) ||
       overlaps(covariance,pb,result,sizeof(*result)) ||
       overlaps(state,sb,f->workspace,f->workspace_bytes) ||
       overlaps(covariance,pb,f->workspace,f->workspace_bytes) || overlaps(result,sizeof(*result),f->workspace,f->workspace_bytes) ||
       overlaps(state,sb,f,sizeof(*f)) || overlaps(covariance,pb,f,sizeof(*f)) || overlaps(result,sizeof(*result),f,sizeof(*f)))
        return KF_ERROR_ARGUMENT;
    memcpy(state,f->FilteredValue,sb); memcpy(covariance,f->published_P,pb); *result=f->result;
    return KF_OK;
}
