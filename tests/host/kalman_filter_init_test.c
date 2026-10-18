/** @file kalman_filter_init_test.c
 * @brief 主机用真实DSP检查工作区、标量参考、有限预测和失败；TEST_CHECK在Release仍执行。
 */
#include "test_check.h"
#include "kalman_filter.h"
#include <math.h>
#include <float.h>
#include <string.h>
#ifdef KF_TEST_THREADS
#include <pthread.h>
#endif
#ifdef KF_TEST_ALLOC_WRAP
#include <stdlib.h>
static int forbidden_allocations;
void *__wrap_malloc(size_t n) { (void)n; forbidden_allocations++; return NULL; }
void *__wrap_calloc(size_t n,size_t s) { (void)n;(void)s; forbidden_allocations++; return NULL; }
void __wrap_free(void *p) { (void)p; forbidden_allocations++; }
#endif
static KFConfig_t config(unsigned n,unsigned m)
{
    KFConfig_t c={n,0,m,20u,20u,2.0f,0.1f,NULL,NULL,NULL}; return c;
}
typedef struct {
    KalmanFilter_t k;
    _Alignas(KF_WORKSPACE_ALIGNMENT) unsigned char w[KF_WORKSPACE_BYTES(2,1,3)];
} Fixture;
static void init(Fixture *f,unsigned n,unsigned m)
{
    KFConfig_t c=config(n,m);
    float x[2]={0,0},p[4]={1,0,0,1}; if(n==1) p[0]=1;
    TEST_CHECK(KFInit(&f->k,&c,f->w,KF_WORKSPACE_BYTES(n,0,m),x,p)==KF_OK);
    for(unsigned i=0;i<n;i++) f->k.F_data[i*n+i]=1;
    for(unsigned i=0;i<m;i++) { f->k.H_data[i*n+i%n]=1; f->k.R_data[i*m+i]=1; }
}
static KFStatus_t step(Fixture *f,uint32_t now,uint32_t source,uint32_t seq,float *z,unsigned m)
{ KFStep_t s={.001f,now,source,seq,z,NULL,m,NULL}; return KFUpdate(&f->k,&s); }
static void workspace(void)
{
    Fixture f={0}; KFConfig_t c=config(1,3); float x=0,p=1; size_t bytes=7,align=9;
    TEST_CHECK(KFGetWorkspaceRequirements(&c,&bytes,&align)==KF_OK);
    TEST_CHECK(bytes==KF_WORKSPACE_BYTES(1,0,3) && align==KF_WORKSPACE_ALIGNMENT);
    memset(f.w,0xa5,sizeof(f.w));
    TEST_CHECK(KFInit(&f.k,&c,f.w,bytes-1,&x,&p)==KF_ERROR_WORKSPACE);
    TEST_CHECK(f.w[0]==0xa5);
    TEST_CHECK(KFInit(&f.k,&c,f.w+1,bytes,&x,&p)==KF_ERROR_WORKSPACE);
    TEST_CHECK(KFInit(&f.k,&c,f.w,bytes,(float *)f.w,&p)==KF_ERROR_WORKSPACE);
    TEST_CHECK(KFInit((KalmanFilter_t *)f.w,&c,f.w,bytes,&x,&p)==KF_ERROR_WORKSPACE);
    TEST_CHECK(KFInit(&f.k,&c,f.w,bytes,&x,&p)==KF_OK);
    TEST_CHECK(f.w[bytes]==0xa5);
    KalmanFilter_t before=f.k; unsigned char saved[sizeof(f.w)]; memcpy(saved,f.w,sizeof(saved));
    for(unsigned i=0;i<4;i++) {
        KFConfig_t bad=c;
        if(i==0)bad.state_count=0; if(i==1)bad.measurement_capacity=256;
        if(i==2)bad.control_count=256; if(i==3)bad.max_prediction_variance_ratio=NAN;
        TEST_CHECK(KFInit(&f.k,&bad,f.w,bytes,&x,&p)==KF_ERROR_CONFIG);
        TEST_CHECK(memcmp(&before,&f.k,sizeof(before))==0 && memcmp(saved,f.w,sizeof(saved))==0);
    }
    c=config(255,255); c.control_count=255;
    TEST_CHECK(KFGetWorkspaceRequirements(&c,&bytes,&align)==KF_OK);
    TEST_CHECK(bytes==KF_WORKSPACE_BYTES(255,255,255));
    TEST_CHECK(KFGetWorkspaceRequirements(NULL,&bytes,&align)==KF_ERROR_ARGUMENT);
    init(&f,1,3); float z[3]={0,0,0};
    TEST_CHECK(step(&f,1,1,1,z,3)==KF_OK);
    TEST_CHECK(fabsf(f.k.published_P[0]-.25f)<1e-6f);
    float out=17,pc=19; KFResult_t r;
    TEST_CHECK(KFGetState(&f.k,&out,0,&pc,1,&r)==KF_ERROR_WORKSPACE && out==17 && pc==19);
    TEST_CHECK(KFGetState(&f.k,&out,1,&pc,1,&r)==KF_OK && r.valid && out==0);
    TEST_CHECK(KFGetState(&f.k,f.k.FilteredValue,1,&pc,1,&r)==KF_ERROR_ARGUMENT);
    float invalid_p=-1,invalid_x=NAN;
    TEST_CHECK(KFReset(&f.k,&invalid_x,&p)==KF_ERROR_NUMERIC);
    TEST_CHECK(KFReset(&f.k,&x,&invalid_p)==KF_ERROR_NUMERIC);
    TEST_CHECK(f.k.result.valid && f.k.FilteredValue[0]==0);
    // 带控制输入的预测使用同一个显式dt；F/B的物理含义由调用者填写。
    c=config(1,1); c.control_count=1;
    TEST_CHECK(KFInit(&f.k,&c,f.w,KF_WORKSPACE_BYTES(1,1,1),&x,&p)==KF_OK);
    f.k.F_data[0]=1;f.k.B_data[0]=.1f;f.k.ControlVector[0]=2;f.k.H_data[0]=1;f.k.R_data[0]=1;
    float zero=0;TEST_CHECK(step(&f,1,1,1,&zero,1)==KF_OK);
    TEST_CHECK(fabsf(f.k.FilteredValue[0]-.1f)<1e-6f);
}
static void scalar_and_time(void)
{
    Fixture f={0}; init(&f,1,1); float z=2;
    TEST_CHECK(step(&f,0,0,1,NULL,0)==KF_PREDICTION_LIMIT && !f.k.result.valid);
    TEST_CHECK(step(&f,1,1,1,&z,1)==KF_OK);
    TEST_CHECK(fabsf(f.k.FilteredValue[0]-1)<1e-6f && fabsf(f.k.published_P[0]-.5f)<1e-6f);
    z=0; TEST_CHECK(step(&f,2,2,2,&z,1)==KF_OK);
    TEST_CHECK(fabsf(f.k.FilteredValue[0]-2.0f/3)<1e-6f);
    TEST_CHECK(step(&f,3,2,2,&z,1)==KF_PREDICTED && f.k.result.cause==KF_DUPLICATE_MEASUREMENT);
    TEST_CHECK(step(&f,22,2,2,NULL,0)==KF_PREDICTED && f.k.result.prediction_age_ms==20);
    float saved=f.k.FilteredValue[0],ps=f.k.published_P[0];
    TEST_CHECK(step(&f,23,2,2,NULL,0)==KF_PREDICTION_LIMIT && !f.k.result.valid);
    TEST_CHECK(f.k.FilteredValue[0]==saved && f.k.published_P[0]==ps && f.k.result.measurement_ms==2);
    TEST_CHECK(step(&f,24,1,3,&z,1)<0 && !f.k.result.valid);
    TEST_CHECK(step(&f,25,25,4,&z,1)==KF_OK && f.k.result.measurement_ms==25);
    TEST_CHECK(step(&f,26,27,5,&z,1)==KF_PREDICTED && f.k.result.cause==KF_STALE_MEASUREMENT);
    z=NAN; TEST_CHECK(step(&f,27,27,6,&z,1)==KF_PREDICTED && f.k.result.cause==KF_ERROR_NUMERIC);
    TEST_CHECK(f.k.result.measurement_ms==25);
    z=0;TEST_CHECK(step(&f,28,28,5,&z,1)==KF_PREDICTED && f.k.result.cause==KF_STALE_MEASUREMENT);
    float initial=0,p=1; TEST_CHECK(KFReset(&f.k,&initial,&p)==KF_OK && !f.k.result.valid);
    z=0; TEST_CHECK(step(&f,UINT32_MAX-10,UINT32_MAX-10,UINT32_MAX,&z,1)==KF_OK);
    TEST_CHECK(step(&f,9,9,0,NULL,0)==KF_PREDICTED && f.k.result.prediction_age_ms==20);
    TEST_CHECK(step(&f,10,10,0,&z,1)==KF_OK);
    TEST_CHECK(step(&f,9,9,1,&z,1)==KF_ERROR_TIME && f.k.fatal);
    init(&f,1,1);TEST_CHECK(step(&f,0,0,1,&z,1)==KF_OK);
    TEST_CHECK(step(&f,INT32_MAX,0,1,NULL,0)==KF_PREDICTION_LIMIT);
    TEST_CHECK(step(&f,UINT32_MAX-1,0,1,NULL,0)==KF_PREDICTION_LIMIT);
    TEST_CHECK(step(&f,1,0,1,NULL,0)==KF_PREDICTION_LIMIT && f.k.result.prediction_age_ms==UINT32_MAX);
}
static KFStatus_t failing(KalmanFilter_t *k,void *ctx)
{ k->xhat_data[0]=1234; return ctx?KF_PREDICTED:KF_ERROR_MATRIX; }
static void variable_dt(void)
{
    Fixture f={0};init(&f,2,1);f.k.H_data[0]=1;f.k.H_data[1]=0;
    double x[2]={0,0},p[4]={1,0,0,1};uint32_t now=0;
    const float intervals[]={.001f,.005f,.01f};
    // 两状态匀速模型的双精度参考，不调用DSP；F/Q由调用方按dt重新填写。
    for(unsigned sample=0;sample<60;sample++) {
        float dt=intervals[sample%3],z=sample%5==0?0:(float)sample*.01f;
        f.k.F_data[1]=dt;f.k.Q_data[0]=.1f*dt;f.k.Q_data[3]=.01f*dt;
        double xp[2]={x[0]+dt*x[1],x[1]};
        double pp[4]={p[0]+dt*(p[1]+p[2])+dt*dt*p[3]+f.k.Q_data[0],
            p[1]+dt*p[3],p[2]+dt*p[3],p[3]+f.k.Q_data[3]};
        double k[2]={pp[0]/(pp[0]+1),pp[2]/(pp[0]+1)};
        x[0]=xp[0]+k[0]*(z-xp[0]);x[1]=xp[1]+k[1]*(z-xp[0]);
        for(unsigned i=0;i<2;i++)for(unsigned j=0;j<2;j++)p[i*2+j]=pp[i*2+j]-k[i]*pp[j];
        now+=(uint32_t)lroundf(dt*1000);
        KFStep_t s={dt,now,now,sample,&z,NULL,1,NULL};
        TEST_CHECK(KFUpdate(&f.k,&s)==KF_OK);
        for(unsigned i=0;i<2;i++)TEST_CHECK(fabs(f.k.FilteredValue[i]-x[i])<2e-5);
        for(unsigned i=0;i<4;i++)TEST_CHECK(fabs(f.k.published_P[i]-p[i])<2e-5);
    }
}
static void failures_and_variance(void)
{
    Fixture f={0}; init(&f,2,2); float z[2]={0,0};
    TEST_CHECK(step(&f,0,0,1,z,2)==KF_OK);
    f.k.Q_data[0]=.5f;
    TEST_CHECK(step(&f,1,0,1,NULL,0)==KF_PREDICTED && f.k.published_P[0]==1);
    TEST_CHECK(step(&f,2,0,1,NULL,0)==KF_PREDICTION_LIMIT);
    TEST_CHECK(f.k.published_P[0]==1 && f.k.xhat_data[0]==0);
    TEST_CHECK(step(&f,3,3,2,z,2)==KF_OK);
    f.k.Q_data[0]=0; f.k.Q_data[3]=2;
    TEST_CHECK(step(&f,4,3,2,NULL,0)==KF_PREDICTION_LIMIT); // 第二个状态单独超限也必须失效。
    init(&f,1,1); f.k.P_data[0]=0;
    TEST_CHECK(step(&f,0,0,1,z,1)==KF_OK && f.k.corrected_variance[0]==0);
    TEST_CHECK(step(&f,1,0,1,NULL,0)==KF_PREDICTED);
    f.k.Q_data[0]=FLT_MIN;
    TEST_CHECK(step(&f,2,0,1,NULL,0)==KF_PREDICTION_LIMIT);
    init(&f,1,1); TEST_CHECK(step(&f,0,0,1,z,1)==KF_OK);
    f.k.config.correction=failing;
    TEST_CHECK(step(&f,1,1,2,z,1)==KF_PREDICTED && f.k.FilteredValue[0]==0 && f.k.result.measurement_ms==0);
    f.k.config.correction=NULL; f.k.config.prepare_measurement=failing;
    TEST_CHECK(step(&f,2,2,3,z,1)==KF_PREDICTED && f.k.FilteredValue[0]==0);
    f.k.config.prepare_measurement=NULL; f.k.R_data[0]=-.5f; // S为0，真实求逆失败。
    TEST_CHECK(step(&f,3,3,4,z,1)==KF_PREDICTED && f.k.result.cause==KF_ERROR_MATRIX);
    f.k.config.after_state_prediction=failing;
    TEST_CHECK(step(&f,4,4,5,z,1)==KF_ERROR_MATRIX && f.k.fatal && !f.k.result.valid);
    f.k.config.after_state_prediction=NULL; f.k.R_data[0]=1;
    TEST_CHECK(step(&f,5,5,6,z,1)==KF_ERROR_STATE);
    float initial=0,p=1; TEST_CHECK(KFReset(&f.k,&initial,&p)==KF_OK);
    TEST_CHECK(step(&f,6,6,7,z,1)==KF_OK);
    f.k.F_data[0]=FLT_MAX; f.k.xhat_data[0]=2;
    float old=f.k.FilteredValue[0]; TEST_CHECK(step(&f,7,7,8,z,1)==KF_ERROR_NUMERIC);
    TEST_CHECK(f.k.FilteredValue[0]==old);
    init(&f,1,1); KFStep_t s={NAN,0,0,0,NULL,NULL,0,NULL};
    TEST_CHECK(KFUpdate(&f.k,&s)==KF_ERROR_TIME && f.k.fatal);
    const float bad_dt[]={0,-1,INFINITY,-INFINITY,NAN,.101f};
    for(unsigned i=0;i<sizeof(bad_dt)/sizeof(bad_dt[0]);i++) {
        init(&f,1,1);s.dt_s=bad_dt[i];
        TEST_CHECK(KFUpdate(&f.k,&s)==KF_ERROR_TIME && !f.k.result.valid);
    }
    init(&f,1,1);uint8_t absent=0;float nonfinite=NAN;
    s=(KFStep_t){.001f,0,0,0,&nonfinite,&absent,1,NULL};
    TEST_CHECK(KFUpdate(&f.k,&s)==KF_PREDICTION_LIMIT); // 不存在的量测无需数值参与校正。
    absent=2;TEST_CHECK(KFUpdate(&f.k,&s)==KF_ERROR_ARGUMENT);
    // 扩展步骤误返回正数也必须报告失败，不能与valid=0的结果给出成功状态。
    init(&f,1,1);f.k.config.after_state_prediction=failing;
    s=(KFStep_t){.001f,0,0,0,NULL,NULL,0,(void *)1};
    TEST_CHECK(KFUpdate(&f.k,&s)==KF_ERROR_STATE && f.k.fatal && !f.k.result.valid);
}
static void *instances(void *arg)
{
    Fixture f={0}; init(&f,1,1); float z=arg?2.0f:-2.0f;
    for(unsigned i=0;i<1000;i++) TEST_CHECK(step(&f,i,i,i,&z,1)==KF_OK);
    TEST_CHECK(fabsf(f.k.FilteredValue[0]-z)<.003f); return NULL;
}
int main(void)
{
    workspace(); scalar_and_time(); variable_dt(); failures_and_variance(); instances(NULL); instances((void *)1);
    Fixture a={0},b={0}; init(&a,1,1); init(&b,1,1); float za=2,zb=-2;
    for(unsigned i=0;i<20;i++) { TEST_CHECK(step(&a,i,i,i,&za,1)==KF_OK); TEST_CHECK(step(&b,i,i,i,&zb,1)==KF_OK); }
    TEST_CHECK(a.k.FilteredValue[0]==-b.k.FilteredValue[0]);
#ifdef KF_TEST_ALLOC_WRAP
    TEST_CHECK(forbidden_allocations==0);
#endif
#ifdef KF_TEST_THREADS
    pthread_t a_thread,b_thread;
    TEST_CHECK(pthread_create(&a_thread,NULL,instances,NULL)==0);
    TEST_CHECK(pthread_create(&b_thread,NULL,instances,(void *)1)==0);
    TEST_CHECK(pthread_join(a_thread,NULL)==0 && pthread_join(b_thread,NULL)==0);
#endif
    return 0;
}
