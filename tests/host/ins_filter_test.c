/** @file ins_filter_test.c
 * @brief 生产INS使用真实EKF/DSP，检查三类结果对有效许可和状态帧的影响；替换采样与传输。
 */
#include "test_check.h"
#include "ins_task.h"
#include "ins_sensor_port.h"
#include "attitude_filter.h"
#include "master_process.h"
#include "ins_hardware.h"
#include "imu_heater.h"
#include "vector3.h"
#include <math.h>
#include <string.h>
static uint32_t now,seq;
static int scenario,valid,mode,force_off,calib_valid=1;
static int fail_bmi_registration;
static unsigned bmi_registration_calls;
static float frame[4];
static VisionIMUSample_t vision_imu_sample;
static uint32_t imu_sample_updates;
static float period_s=.001f;
#ifdef INS_TEST_INIT_WRAP
static int fail_init;
KFStatus_t __real_AttitudeFilterInit(AttitudeFilter_t *,const AttitudeFilterConfig_t *,const float *,void *,size_t);
KFStatus_t __wrap_AttitudeFilterInit(AttitudeFilter_t *filter,const AttitudeFilterConfig_t *config,
    const float *q,void *workspace,size_t bytes)
{ return fail_init ? KF_ERROR_WORKSPACE : __real_AttitudeFilterInit(filter,config,q,workspace,bytes); }
#endif
extern volatile uint32_t imu_filter_mode_diag;
extern volatile uint32_t imu_filter_age_ms_diag;
extern volatile int32_t imu_filter_cause_diag;
int hspi2;
volatile uint32_t bmi088_init_error,bmi088_init_retry_count;
float DWT_GetDeltaT(uint32_t *p){ *p=now;return period_s; }
uint32_t DWT_ProbeStart(void){ return 0; }
void DWT_ProbeDone(volatile DWT_Probe_t *p,uint32_t s){(void)p;(void)s;}
uint32_t HAL_GetTick(void){return now;}
BMI088Instance *BMI088Register(const BMI088_Init_Config_s *c)
{
    static BMI088Instance i;
    // 不同片选值可以识别陀螺和加速度计绑定被交换的情况。
    TEST_CHECK(c != NULL && c->work_mode == BMI088_BLOCK_PERIODIC_MODE);
    TEST_CHECK(c->cali_mode == BMI088_CALIBRATE_ONLINE_MODE);
    TEST_CHECK(c->spi_gyro_config.spi_handle == &hspi2 && c->spi_acc_config.spi_handle == &hspi2);
    TEST_CHECK(c->spi_gyro_config.GPIOx == CS2_GYRO_GPIO_Port && c->spi_gyro_config.cs_pin == CS2_GYRO_Pin);
    TEST_CHECK(c->spi_acc_config.GPIOx == CS2_ACCEL_GPIO_Port && c->spi_acc_config.cs_pin == CS2_ACCEL_Pin);
    TEST_CHECK(c->spi_gyro_config.spi_work_mode == SPI_BLOCK_MODE && c->spi_acc_config.spi_work_mode == SPI_BLOCK_MODE);
    bmi_registration_calls++;
    return fail_bmi_registration ? NULL : &i;
}
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
VisionIMUSampleUpdateStatus_t VisionUpdateIMUSample(const VisionIMUSample_t *sample){
    if(!sample)return VISION_IMU_SAMPLE_UPDATE_ERR_ARGUMENT;
    vision_imu_sample=*sample;imu_sample_updates++;
    return VISION_IMU_SAMPLE_UPDATE_OK;
}
void TaskMonitorDelayMs(int task,uint32_t ms){(void)task;now+=ms;}
void IMUHeaterForceOff(void){force_off=1;}
void IMUHeaterInit(void){}
void IMUHeaterUpdate(const BMI088_Data_t *s,uint8_t off){force_off=off || !s || !s->valid;}
static void tick(void){now+=(uint32_t)lroundf(period_s*1000);INS_Task();}
static void invalid(void){TEST_CHECK(!valid && mode==0);for(int i=0;i<4;i++)TEST_CHECK(frame[i]==0);}
int main(int argc,char **argv)
{
    TEST_CHECK(!INS_ConfigureSensor(PLATFORM_IMU_INVALID)); // 未绑定时不能选择或启动传感器。
    TEST_CHECK(InsSensorRegister(PLATFORM_IMU_INVALID) == NULL && bmi_registration_calls == 0u);
    if (argc > 1 && strcmp(argv[1], "--unconfigured") == 0)
    {
        TEST_CHECK(INS_Init() == NULL && !valid && !calib_valid && force_off); // 缺少配置时保持失能。
        TEST_CHECK(bmi_registration_calls == 0u); // 不得尝试初始化未选择的 SPI 设备。
        return 0;
    }
    TEST_CHECK(INS_ConfigureSensor(PLATFORM_IMU_BOARD_1)); // 所有场景均使用显式板载选择。
    TEST_CHECK(!INS_ConfigureSensor(PLATFORM_IMU_INVALID)); // 无效选择不能覆盖已确认的编号。
    if(argc>1 && strcmp(argv[1],"--bmi-registration-failure")==0)
    {
        fail_bmi_registration=1;
        TEST_CHECK(INS_Init()==NULL && !valid && !calib_valid && force_off);
        TEST_CHECK(INS_Init()==NULL && bmi_registration_calls==1u);
        return 0;
    }
    if(argc>1 && strncmp(argv[1],"--initial-",10)==0)
    {
        const char *kind=argv[1]+10;
        if(strcmp(kind,"tilt")==0)scenario=5;
        else if(strcmp(kind,"inverted")==0)scenario=6;
        else if(strcmp(kind,"zero")==0)scenario=1;
        else {TEST_CHECK(strcmp(kind,"short")==0);scenario=3;}
        TEST_CHECK(INS_Init()!=NULL);
        // 各轴角输入仍须完成完整采样；姿态数值由公共后端接口单测固定向量覆盖。
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
    TEST_CHECK(INS_Init()!=NULL);
    TEST_CHECK(!INS_ConfigureSensor(PLATFORM_IMU_BOARD_1)); // 初始化后不能改变传感器。
    tick();TEST_CHECK(valid && mode==1);
    TEST_CHECK(imu_sample_updates==1u && vision_imu_sample.valid==1u &&
        vision_imu_sample.sample_sequence==seq && vision_imu_sample.sample_time_ms==now &&
        vision_imu_sample.acceleration_m_s2[2]==9.8f);
    INS_Control_Snapshot_s control_snapshot = {0};
    TEST_CHECK(INSGetControlSnapshot(&control_snapshot) == 1u);
    TEST_CHECK(control_snapshot.valid == 1u && control_snapshot.sequence == seq &&
               control_snapshot.timestamp_ms == now);
    TEST_CHECK(isfinite(control_snapshot.yaw_rad) && isfinite(control_snapshot.yaw_total_rad) &&
               isfinite(control_snapshot.pitch_rad));
    for (unsigned axis = 0u; axis < 3u; ++axis)
        TEST_CHECK(isfinite(control_snapshot.gyro_rad_s[axis]));
    scenario=1;
    for(unsigned i=1;i<=20;i++){tick();TEST_CHECK(valid && imu_filter_mode_diag==KF_PREDICTION);}
    tick();invalid();TEST_CHECK(force_off);
    TEST_CHECK(INSGetControlSnapshot(&control_snapshot) == 0u && control_snapshot.valid == 0u);
    scenario=0;tick();TEST_CHECK(valid && mode==1 && imu_filter_mode_diag==KF_CORRECTED);
    scenario=3;tick();invalid();
    TEST_CHECK(vision_imu_sample.valid==0u && vision_imu_sample.sample_sequence==0u &&
        vision_imu_sample.sample_time_ms==0u && vision_imu_sample.acceleration_m_s2[0]==0.0f &&
        vision_imu_sample.acceleration_m_s2[1]==0.0f && vision_imu_sample.acceleration_m_s2[2]==0.0f);
    scenario=0;tick();TEST_CHECK(valid && vision_imu_sample.valid==1u);
    // 调度或原始采样延迟超过20ms时，正常预测加新鲜校正仍应恢复，不能附加dt故障。
    period_s=.025f;tick();TEST_CHECK(valid && imu_filter_mode_diag==KF_CORRECTED);
    scenario=1;tick();invalid();
    period_s=.001f;scenario=0;tick();TEST_CHECK(valid);
    scenario=3;period_s=.025f;tick();invalid();
    scenario=0;period_s=.001f;tick();TEST_CHECK(valid);
    scenario=2;tick();invalid();TEST_CHECK(imu_filter_cause_diag==KF_ERROR_NUMERIC);
    TEST_CHECK(vision_imu_sample.valid==1u && vision_imu_sample.acceleration_m_s2[2]==9.8f);
    scenario=0;tick();TEST_CHECK(valid && imu_filter_mode_diag==KF_CORRECTED);
    TEST_CHECK(INS_Init()!=NULL);tick();TEST_CHECK(valid); // 重复INS_Init不能偷偷重建姿态参考。
    return 0;
}
