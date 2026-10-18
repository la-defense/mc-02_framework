/** @file ins_filter_test.c
 * @brief 生产INS使用真实EKF/DSP，检查三类结果对有效许可和状态帧的影响；替换采样与传输。
 */
#include "test_check.h"
#include "ins_task.h"
#include "ins_hardware.h"
#include "imu_heater.h"
#include <math.h>
#include <string.h>
static uint32_t now,seq;
static int scenario,valid,mode,force_off;
static float frame[4];
#ifdef INS_TEST_INIT_WRAP
static int fail_init;
KFStatus_t __real_IMU_QuaternionEKF_Init(const float *,const QEKFConfig_t *,void *,size_t);
KFStatus_t __wrap_IMU_QuaternionEKF_Init(const float *q,const QEKFConfig_t *c,void *w,size_t n)
{ return fail_init?KF_ERROR_WORKSPACE:__real_IMU_QuaternionEKF_Init(q,c,w,n); }
#endif
int hspi2;
volatile uint32_t bmi088_init_error,bmi088_init_retry_count;
float DWT_GetDeltaT(uint32_t *p){ *p=now;return .001f; }
uint32_t DWT_ProbeStart(void){ return 0; }
void DWT_ProbeDone(volatile DWT_Probe_t *p,uint32_t s){(void)p;(void)s;}
uint32_t HAL_GetTick(void){return now;}
BMI088Instance *BMI088Register(const BMI088_Init_Config_s *c){static BMI088Instance i;(void)c;return &i;}
uint8_t BMI088Acquire(BMI088Instance *i,BMI088_Data_t *s){
    (void)i; *s=(BMI088_Data_t){.sequence=++seq,.timestamp_ms=now,.valid=1,.temperature=25};
    s->acc[2]=scenario==1?0:9.8f; if(scenario==2)s->gyro[0]=NAN;
    if(scenario==3){s->valid=0;return 0;} return BMI088_ACQUIRE_OK;
}
uint8_t BMI088CalibService(void){return 0;}
void RobotSafetySetImuValid(uint8_t v){valid=v;}
void RobotSafetySetCalibValid(uint8_t v){(void)v;}
Robot_Status_e RobotSafetyGetState(void){return ROBOT_READY;}
uint8_t RobotSafetyIsReady(void){return valid;}
void VisionUpdateTx(uint8_t m,float w,float x,float y,float z,float a,float b,float c,float d,float e,uint8_t f){
    (void)a;(void)b;(void)c;(void)d;(void)e;(void)f; mode=m;frame[0]=w;frame[1]=x;frame[2]=y;frame[3]=z;
}
void TaskMonitorDelayMs(int task,uint32_t ms){(void)task;now+=ms;}
void IMUHeaterForceOff(void){force_off=1;}
void IMUHeaterInit(void){}
void IMUHeaterUpdate(const BMI088_Data_t *s,uint8_t off){force_off=off || !s || !s->valid;}
float NormOf3d(float *v){return sqrtf(Dot3d(v,v));}
void Norm3d(float *v){float n=NormOf3d(v);for(int i=0;i<3;i++)v[i]/=n;}
float Dot3d(float *a,float *b){return a[0]*b[0]+a[1]*b[1]+a[2]*b[2];}
void Cross3d(float *a,float *b,float *o){o[0]=a[1]*b[2]-a[2]*b[1];o[1]=a[2]*b[0]-a[0]*b[2];o[2]=a[0]*b[1]-a[1]*b[0];}
static void tick(void){now++;INS_Task();}
static void invalid(void){TEST_CHECK(!valid && mode==0);for(int i=0;i<4;i++)TEST_CHECK(frame[i]==0);}
int main(int argc,char **argv)
{
    (void)argv;
#ifdef INS_TEST_INIT_WRAP
    if(argc>1){fail_init=1;TEST_CHECK(INS_Init()==NULL && !valid && force_off);return 0;}
#else
    (void)argc;
#endif
    TEST_CHECK(INS_Init()!=NULL);tick();TEST_CHECK(valid && mode==1);
    scenario=1;
    for(unsigned i=1;i<=20;i++){tick();TEST_CHECK(valid && QEKF_INS.IMU_QuaternionEKF.result.mode==KF_PREDICTION);}
    tick();invalid();TEST_CHECK(force_off);
    scenario=0;tick();TEST_CHECK(valid && mode==1 && QEKF_INS.IMU_QuaternionEKF.result.mode==KF_CORRECTED);
    scenario=3;tick();invalid();scenario=0;tick();TEST_CHECK(valid);
    scenario=2;tick();invalid();scenario=0;tick();invalid();
    TEST_CHECK(QEKF_INS.IMU_QuaternionEKF.fatal);
    TEST_CHECK(INS_Init()!=NULL);tick();invalid(); // 重复INS_Init不能偷偷重建姿态参考。
    return 0;
}
