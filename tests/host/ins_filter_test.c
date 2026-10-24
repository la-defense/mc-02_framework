/** @file ins_filter_test.c
 * @brief 生产INS使用真实EKF/DSP，检查三类结果对有效许可和状态帧的影响；替换采样与传输。
 */
#include "test_check.h"
#include "ins_task.h"
#include "ins_hardware.h"
#include "imu_heater.h"
#include "vector3.h"
#include <math.h>
#include <string.h>
static uint32_t now,seq;
static int scenario,valid,mode,force_off,calib_valid=1;
static float frame[4];
static float period_s=.001f;
#ifdef INS_TEST_INIT_WRAP
static int fail_init;
KFStatus_t __real_IMU_QuaternionEKF_Init(const float *,const QEKFConfig_t *,void *,size_t);
KFStatus_t __wrap_IMU_QuaternionEKF_Init(const float *q,const QEKFConfig_t *c,void *w,size_t n)
{ return fail_init?KF_ERROR_WORKSPACE:__real_IMU_QuaternionEKF_Init(q,c,w,n); }
#endif
int hspi2;
volatile uint32_t bmi088_init_error,bmi088_init_retry_count;
float DWT_GetDeltaT(uint32_t *p){ *p=now;return period_s; }
uint32_t DWT_ProbeStart(void){ return 0; }
void DWT_ProbeDone(volatile DWT_Probe_t *p,uint32_t s){(void)p;(void)s;}
uint32_t HAL_GetTick(void){return now;}
BMI088Instance *BMI088Register(const BMI088_Init_Config_s *c){static BMI088Instance i;(void)c;return &i;}
uint8_t BMI088Acquire(BMI088Instance *i,BMI088_Data_t *s){
    (void)i; *s=(BMI088_Data_t){.sequence=++seq,.timestamp_ms=now,.valid=1,.temperature=25};
    s->acc[2]=scenario==1?0:9.8f; if(scenario==2)s->gyro[0]=NAN;
    if(scenario==4)s->acc[0]=NAN;
    if(scenario==5){s->acc[0]=4;s->acc[2]=3;}
    if(scenario==6)s->acc[2]=-9.8f;
    if(scenario==3){s->valid=0;return 0;} return BMI088_ACQUIRE_OK;
}
uint8_t BMI088CalibService(void){return 0;}
void RobotSafetySetImuValid(uint8_t v){valid=v;}
void RobotSafetySetCalibValid(uint8_t v){calib_valid=v;}
Robot_Status_e RobotSafetyGetState(void){return ROBOT_READY;}
uint8_t RobotSafetyIsReady(void){return valid;}
void VisionUpdateTx(uint8_t m,float w,float x,float y,float z,float a,float b,float c,float d,float e,uint8_t f){
    (void)a;(void)b;(void)c;(void)d;(void)e;(void)f; mode=m;frame[0]=w;frame[1]=x;frame[2]=y;frame[3]=z;
}
void TaskMonitorDelayMs(int task,uint32_t ms){(void)task;now+=ms;}
void IMUHeaterForceOff(void){force_off=1;}
void IMUHeaterInit(void){}
void IMUHeaterUpdate(const BMI088_Data_t *s,uint8_t off){force_off=off || !s || !s->valid;}
static void tick(void){now+=(uint32_t)lroundf(period_s*1000);INS_Task();}
static void invalid(void){TEST_CHECK(!valid && mode==0);for(int i=0;i<4;i++)TEST_CHECK(frame[i]==0);}
int main(int argc,char **argv)
{
    if(argc>1 && strncmp(argv[1],"--initial-",10)==0)
    {
        const char *kind=argv[1]+10;
        if(strcmp(kind,"tilt")==0)scenario=5;
        else if(strcmp(kind,"inverted")==0)scenario=6;
        else if(strcmp(kind,"zero")==0)scenario=1;
        else {TEST_CHECK(strcmp(kind,"short")==0);scenario=3;}
        TEST_CHECK(INS_Init()!=NULL);
        // 独立轴角答案覆盖倾斜、倒置及原有两种水平回退。
        double expected[4]={1,0,0,0};
        if(scenario==5){expected[0]=sqrt(.8);expected[2]=-sqrt(.2);}
        if(scenario==6){expected[0]=0;expected[1]=1;}
        for(size_t i=0;i<4;++i)TEST_CHECK(fabs(QEKF_INS.q[i]-expected[i])<2e-6);
        TEST_CHECK(seq==(scenario==3?200u:100u));
        return 0;
    }
    // 非有限初始加速度必须由真实向量入口拒绝，并保持初始化失败。
    if(argc>1 && strcmp(argv[1],"--math-failure")==0)
    {
        scenario=4;
        TEST_CHECK(INS_Init()==NULL && !valid && !calib_valid && force_off);
        TEST_CHECK(INS_Init()==NULL);
        return 0;
    }
#ifdef INS_TEST_INIT_WRAP
    if(argc>1){fail_init=1;TEST_CHECK(INS_Init()==NULL && !valid && force_off);return 0;}
#else
    (void)argc;
#endif
    TEST_CHECK(INS_Init()!=NULL);tick();TEST_CHECK(valid && mode==1);
    INS_Control_Snapshot_s control_snapshot = {0};
    TEST_CHECK(INSGetControlSnapshot(&control_snapshot) == 1u);
    TEST_CHECK(control_snapshot.valid == 1u && control_snapshot.sequence == seq &&
               control_snapshot.timestamp_ms == now);
    TEST_CHECK(isfinite(control_snapshot.yaw_rad) && isfinite(control_snapshot.yaw_total_rad) &&
               isfinite(control_snapshot.pitch_rad));
    for (unsigned axis = 0u; axis < 3u; ++axis)
        TEST_CHECK(isfinite(control_snapshot.gyro_rad_s[axis]));
    scenario=1;
    for(unsigned i=1;i<=20;i++){tick();TEST_CHECK(valid && QEKF_INS.IMU_QuaternionEKF.result.mode==KF_PREDICTION);}
    tick();invalid();TEST_CHECK(force_off);
    TEST_CHECK(INSGetControlSnapshot(&control_snapshot) == 0u && control_snapshot.valid == 0u);
    scenario=0;tick();TEST_CHECK(valid && mode==1 && QEKF_INS.IMU_QuaternionEKF.result.mode==KF_CORRECTED);
    scenario=3;tick();invalid();scenario=0;tick();TEST_CHECK(valid);
    // 调度或原始采样延迟超过20ms时，正常预测加新鲜校正仍应恢复，不能附加dt故障。
    period_s=.025f;tick();TEST_CHECK(valid && QEKF_INS.IMU_QuaternionEKF.result.mode==KF_CORRECTED);
    scenario=1;tick();invalid();TEST_CHECK(!QEKF_INS.IMU_QuaternionEKF.fatal);
    period_s=.001f;scenario=0;tick();TEST_CHECK(valid);
    scenario=3;period_s=.025f;tick();invalid();
    scenario=0;period_s=.001f;tick();TEST_CHECK(valid);
    scenario=2;tick();invalid();scenario=0;tick();invalid();
    TEST_CHECK(QEKF_INS.IMU_QuaternionEKF.fatal);
    TEST_CHECK(INS_Init()!=NULL);tick();invalid(); // 重复INS_Init不能偷偷重建姿态参考。
    return 0;
}
