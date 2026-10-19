/** @file sample_filter_test.c
 * @brief 对生产四类滤波执行独立方程参考、失败、恢复、并发和频率响应检查。
 * @note 由CTest选择场景；export前缀另生成CSV供教学绘图，不连接硬件。
 */
#include "sample_filter.h"
#include "test_check.h"
#include <complex.h>
#include <float.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#ifdef FILTER_TEST_THREADS
#include <pthread.h>
#endif
#ifdef FILTER_TEST_ALLOC_WRAP
#include <stdatomic.h>
static _Atomic unsigned allocation_calls;
void *__real_malloc(size_t size);
void *__real_calloc(size_t count, size_t size);
void *__real_realloc(void *pointer, size_t size);
void __real_free(void *pointer);
/** @brief 统计malloc调用后转交标准分配器；仅供链接包装。 */
void *__wrap_malloc(size_t size) { ++allocation_calls; return __real_malloc(size); }
/** @brief 统计calloc调用后转交标准分配器；仅供链接包装。 */
void *__wrap_calloc(size_t count,size_t size) { ++allocation_calls; return __real_calloc(count,size); }
/** @brief 统计realloc调用后转交标准分配器；仅供链接包装。 */
void *__wrap_realloc(void *pointer,size_t size) { ++allocation_calls; return __real_realloc(pointer,size); }
/** @brief 统计free调用后转交标准分配器；仅供链接包装。 */
void __wrap_free(void *pointer) { ++allocation_calls; __real_free(pointer); }
#endif

static const double pi = 3.14159265358979323846;

static SampleFilterConfig_t config_for(SampleFilterType_t type)
{
    SampleFilterConfig_t config={.type=type,.dt_min_s=.0005f,.dt_max_s=.0015f};
    if(type==SAMPLE_FILTER_LOWPASS1) config.tau_s=.01f;
    if(type==SAMPLE_FILTER_AVERAGE) config.window_length=9u;
    if(type==SAMPLE_FILTER_LOWPASS2 || type==SAMPLE_FILTER_NOTCH) config.frequency_hz=20;
    if(type==SAMPLE_FILTER_NOTCH) config.quality_factor=4;
    return config;
}

static void near(float actual,double expected)
{
    TEST_CHECK(isfinite(actual) && isfinite(expected));
    TEST_CHECK(fabs(actual-expected)<=3e-6+3e-6*fabs(expected));
}

typedef struct
{
    SampleFilterConfig_t config;
    double first,second,history[255];
    int seeded;
} Reference;

// 独立参考由连续两状态模型建立2x2梯形积分方程，再用消元求下一时刻，输出取两端中点。
// 不调用生产系数或优化更新式；平均参考直接移动double数组。
static double reference_update(Reference *reference,float input,float dt)
{
    SampleFilterConfig_t *c=&reference->config;
    if(!reference->seeded)
    {
        reference->first=c->type==SAMPLE_FILTER_LOWPASS1?input:0;
        reference->second=input;
        for(size_t i=0;i<c->window_length;++i)reference->history[i]=input;
        reference->seeded=1;
        return input;
    }
    if(c->type==SAMPLE_FILTER_LOWPASS1)
    {
        reference->first=((double)c->tau_s*reference->first+(double)dt*input)/(c->tau_s+(double)dt);
        return reference->first;
    }
    if(c->type==SAMPLE_FILTER_AVERAGE)
    {
        for(size_t i=1;i<c->window_length;++i)reference->history[i-1]=reference->history[i];
        reference->history[c->window_length-1]=input;
        double sum=0;
        for(size_t i=0;i<c->window_length;++i)sum+=reference->history[i];
        return sum/(double)c->window_length;
    }
    double g=tan(pi*(double)c->frequency_hz*dt);
    double damping=c->type==SAMPLE_FILTER_LOWPASS2?sqrt(2.0):1.0/c->quality_factor;
    double matrix[2][3]={
        {1+g*damping,g,(1-g*damping)*reference->first-g*reference->second+2*g*input},
        {-g,1,g*reference->first+reference->second}
    };
    double factor=matrix[1][0]/matrix[0][0];
    for(size_t j=0;j<3;++j)matrix[1][j]-=factor*matrix[0][j];
    double second=matrix[1][2]/matrix[1][1];
    double first=(matrix[0][2]-matrix[0][1]*second)/matrix[0][0];
    double result=c->type==SAMPLE_FILTER_LOWPASS2?
        (second+reference->second)/2:input-damping*(first+reference->first)/2;
    reference->first=first;reference->second=second;
    return result;
}

static void initialize(SampleFilter_t *filter,const SampleFilterConfig_t *config,float *history,size_t capacity)
{
    TEST_CHECK(SampleFilterInit(filter,config,history,capacity)==FILTER_OK);
    SampleFilterState_t state;
    TEST_CHECK(SampleFilterGetState(filter,&state)==FILTER_OK && !state.valid && state.needs_rebuild);
    TEST_CHECK(state.last_status==FILTER_WAITING && state.accepted_count==0);
}

static void reference_tests(void)
{
    for(int type=SAMPLE_FILTER_LOWPASS1;type<=SAMPLE_FILTER_NOTCH;++type)
    {
        SampleFilterConfig_t config=config_for((SampleFilterType_t)type);
        SampleFilter_t filter={0};float history[255],out=79;
        initialize(&filter,&config,type==SAMPLE_FILTER_AVERAGE?history:NULL,
                   type==SAMPLE_FILTER_AVERAGE?255:0);
        Reference reference={.config=config};
        uint32_t random=71;
        // 三种dt模式分别从常值初始状态开始，脉冲、阶跃和噪声共同覆盖历史与响应。
        for(unsigned mode=0;mode<3;++mode)
        {
            TEST_CHECK(SampleFilterReset(&filter)==FILTER_OK);
            memset(&reference,0,sizeof(reference));reference.config=config;
            for(unsigned n=0;n<30000;++n)
            {
                random=random*1664525u+1013904223u;
                float dt=.001f;
                if(mode==1)dt=(n&1u)?config.dt_min_s:config.dt_max_s;
                if(mode==2)dt=.0005f+.001f*(float)(random%1001u)/1000;
                float value=n<20?2.0f:(n<300?(n==20?10.0f:0.0f):
                    (float)(sin(n*.017)+.7*cos(n*.131)));
                SampleFilterStatus_t status=SampleFilterUpdate(&filter,1,value,dt,&out);
                TEST_CHECK(status==(n==0?FILTER_REBUILT:FILTER_OK));
                near(out,reference_update(&reference,value,dt));
            }
        }
        // 常值和重置后第一份输入没有启动衰减，所有种类输出一致。
        TEST_CHECK(SampleFilterReset(&filter)==FILTER_OK);
        TEST_CHECK(SampleFilterUpdate(&filter,1,-7,NAN,&out)==FILTER_REBUILT && out==-7);
        for(unsigned n=0;n<100;++n)
            TEST_CHECK(SampleFilterUpdate(&filter,1,-7,.001f,&out)==FILTER_OK && out==-7);
    }
    SampleFilter_t scalar={0};SampleFilterConfig_t c=config_for(SAMPLE_FILTER_LOWPASS1);
    initialize(&scalar,&c,NULL,0);float out;
    TEST_CHECK(SampleFilterUpdate(&scalar,1,0,0,&out)==FILTER_REBUILT);
    for(unsigned n=1;n<=100;++n)
    {
        TEST_CHECK(SampleFilterUpdate(&scalar,1,1,.001f,&out)==FILTER_OK);
        near(out,1-pow((double)c.tau_s/(c.tau_s+.001f),n));
        if(n==1)near(out,1.0/11.0);
    }
    SampleFilter_t moving={0};c=config_for(SAMPLE_FILTER_AVERAGE);float window[9];
    initialize(&moving,&c,window,9);
    TEST_CHECK(SampleFilterUpdate(&moving,1,0,0,&out)==FILTER_REBUILT);
    TEST_CHECK(SampleFilterUpdate(&moving,1,9,.001f,&out)==FILTER_OK && out==1);
    // 初始化已经复制配置，随后修改原配置不能改变实例行为。
    c.window_length=1;
    TEST_CHECK(SampleFilterUpdate(&moving,1,0,.001f,&out)==FILTER_OK && out==1);
    for(size_t length=1;length<=255;length+=254)
    {
        SampleFilter_t average={0};c=config_for(SAMPLE_FILTER_AVERAGE);c.window_length=length;
        float history[256];for(size_t i=0;i<256;++i)history[i]=91;
        initialize(&average,&c,history,length);
        TEST_CHECK(history[length]==91);
        TEST_CHECK(SampleFilterUpdate(&average,1,FLT_MAX,0,&out)==FILTER_REBUILT && out==FLT_MAX);
        TEST_CHECK(SampleFilterUpdate(&average,1,FLT_MAX,.001f,&out)==FILTER_OK && out==FLT_MAX);
    }
}

// 只比较数值及已接受历史；数据错误可以改变有效性和待重建原因。
static void same_committed(const SampleFilter_t *a,const SampleFilter_t *b)
{
    TEST_CHECK(a->_state1==b->_state1 && a->_state2==b->_state2);
    TEST_CHECK(a->_result.last_output==b->_result.last_output);
    TEST_CHECK(a->_result.accepted_count==b->_result.accepted_count);
    TEST_CHECK(a->_result.rebuild_count==b->_result.rebuild_count);
}

static void config_failures(void)
{
    SampleFilter_t filter={0};SampleFilterConfig_t valid=config_for(SAMPLE_FILTER_AVERAGE);
    float history[16];initialize(&filter,&valid,history,16);
    float out;TEST_CHECK(SampleFilterUpdate(&filter,1,3,0,&out)==FILTER_REBUILT);
    SampleFilter_t before=filter;float old_history[16];memcpy(old_history,history,sizeof(history));
    for(unsigned n=0;n<16;++n)
    {
        SampleFilterConfig_t c=valid;
        switch(n)
        {
        case 0:c.type=0;break;
        case 1:c.type=(SampleFilterType_t)99;break;
        case 2:c.dt_min_s=0;break;
        case 3:c.dt_min_s=-1;break;
        case 4:c.dt_min_s=.1f;break;
        case 5:c.dt_max_s=INFINITY;break;
        case 6:c.dt_min_s=NAN;break;
        case 7:c.tau_s=NAN;break;
        case 8:c.frequency_hz=INFINITY;break;
        case 9:c.quality_factor=NAN;break;
        case 10:c.window_length=0;break;
        case 11:c.window_length=256;break;
        case 12:c.tau_s=1;break;
        case 13:c=config_for(SAMPLE_FILTER_LOWPASS1);c.tau_s=0;break;
        case 14:c=config_for(SAMPLE_FILTER_NOTCH);c.quality_factor=0;break;
        default:c=config_for(SAMPLE_FILTER_LOWPASS2);c.dt_max_s=1.0f/512;c.frequency_hz=256;break;
        }
        TEST_CHECK(SampleFilterInit(&filter,&c,history,16)==FILTER_ERROR_CONFIG);
        TEST_CHECK(memcmp(&filter,&before,sizeof(filter))==0 && memcmp(old_history,history,sizeof(history))==0);
    }
    SampleFilterConfig_t c=config_for(SAMPLE_FILTER_LOWPASS1);c.tau_s=FLT_MAX;
    TEST_CHECK(SampleFilterInit(&filter,&c,NULL,0)==FILTER_ERROR_CONFIG);
    c=config_for(SAMPLE_FILTER_NOTCH);c.quality_factor=FLT_MAX;
    TEST_CHECK(SampleFilterInit(&filter,&c,NULL,0)==FILTER_ERROR_CONFIG);
    TEST_CHECK(SampleFilterInit(&filter,&valid,NULL,16)==FILTER_ERROR_ARGUMENT);
    TEST_CHECK(SampleFilterInit(&filter,&valid,history,8)==FILTER_ERROR_ARGUMENT);
    TEST_CHECK(SampleFilterInit(&filter,&valid,history,SIZE_MAX)==FILTER_ERROR_ARGUMENT);
    TEST_CHECK(SampleFilterInit(&filter,&valid,(float *)((uintptr_t)history+1),16)==FILTER_ERROR_ARGUMENT);
    TEST_CHECK(SampleFilterInit(&filter,&valid,(float *)(UINTPTR_MAX-3u),16)==FILTER_ERROR_ARGUMENT);
    TEST_CHECK(SampleFilterInit(&filter,&valid,(float *)&filter,16)==FILTER_ERROR_ARGUMENT);
    TEST_CHECK(SampleFilterInit(&filter,&valid,(float *)&valid,16)==FILTER_ERROR_ARGUMENT);
    TEST_CHECK(SampleFilterInit(&filter,&filter._config,history,16)==FILTER_ERROR_ARGUMENT);
    TEST_CHECK(SampleFilterInit(NULL,&valid,history,16)==FILTER_ERROR_ARGUMENT);
    TEST_CHECK(SampleFilterInit(&filter,NULL,history,16)==FILTER_ERROR_ARGUMENT);
    TEST_CHECK(SampleFilterInit((SampleFilter_t *)((uintptr_t)&filter+1),&valid,history,16)==FILTER_ERROR_ARGUMENT);
    TEST_CHECK(SampleFilterInit(&filter,(SampleFilterConfig_t *)((uintptr_t)&valid+1),history,16)==FILTER_ERROR_ARGUMENT);
    TEST_CHECK(memcmp(&filter,&before,sizeof(filter))==0 && memcmp(old_history,history,sizeof(history))==0);
}

static void failures(void)
{
    config_failures();
    SampleFilter_t empty={0};float out=79;SampleFilterState_t state,old_state;
    memset(&state,0xa5,sizeof(state));old_state=state;
    TEST_CHECK(SampleFilterUpdate(&empty,1,1,.001f,&out)==FILTER_NOT_INITIALIZED && out==79);
    TEST_CHECK(SampleFilterGetState(&empty,&state)==FILTER_NOT_INITIALIZED && memcmp(&state,&old_state,sizeof(state))==0);
    TEST_CHECK(SampleFilterReset(&empty)==FILTER_NOT_INITIALIZED);
    TEST_CHECK(SampleFilterReset(NULL)==FILTER_ERROR_ARGUMENT);
    for(int type=SAMPLE_FILTER_LOWPASS1;type<=SAMPLE_FILTER_NOTCH;++type)
    {
        SampleFilter_t filter={0};SampleFilterConfig_t c=config_for((SampleFilterType_t)type);float history[16]={0};
        initialize(&filter,&c,type==SAMPLE_FILTER_AVERAGE?history:NULL,type==SAMPLE_FILTER_AVERAGE?16:0);
        TEST_CHECK(SampleFilterUpdate(&filter,1,3,0,&out)==FILTER_REBUILT);
        SampleFilter_t before=filter;
        TEST_CHECK(SampleFilterGetState(&filter,NULL)==FILTER_ERROR_ARGUMENT);
        TEST_CHECK(SampleFilterGetState(&filter,(SampleFilterState_t *)&filter)==FILTER_ERROR_ARGUMENT);
        TEST_CHECK(SampleFilterGetState(&filter,(SampleFilterState_t *)((uintptr_t)&state+1))==FILTER_ERROR_ARGUMENT);
        TEST_CHECK(SampleFilterUpdate(&filter,2,1,.001f,&out)==FILTER_ERROR_ARGUMENT);
        TEST_CHECK(SampleFilterUpdate(&filter,1,1,.001f,NULL)==FILTER_ERROR_ARGUMENT);
        TEST_CHECK(SampleFilterUpdate(&filter,1,1,.001f,(float *)&filter)==FILTER_ERROR_ARGUMENT);
        TEST_CHECK(SampleFilterUpdate(&filter,1,1,.001f,(float *)((uintptr_t)&out+1))==FILTER_ERROR_ARGUMENT);
        if(type==SAMPLE_FILTER_AVERAGE)
        {
            TEST_CHECK(SampleFilterUpdate(&filter,1,1,.001f,history+15)==FILTER_ERROR_ARGUMENT);
            TEST_CHECK(SampleFilterGetState(&filter,(SampleFilterState_t *)history)==FILTER_ERROR_ARGUMENT);
        }
        TEST_CHECK(memcmp(&before,&filter,sizeof(filter))==0);
        SampleFilter_t copy=filter;
        TEST_CHECK(SampleFilterReset(&copy)==FILTER_NOT_INITIALIZED);
        // 每次数据错误后，原输出及历史保留，下一样本忽略未知dt并重建。
        const float bad_dt[]={0,-1,NAN,INFINITY,-INFINITY,nextafterf(c.dt_min_s,0),nextafterf(c.dt_max_s,INFINITY)};
        for(size_t i=0;i<sizeof(bad_dt)/sizeof(bad_dt[0]);++i)
        {
            before=filter;float saved[16];memcpy(saved,history,sizeof(history));out=79;
            TEST_CHECK(SampleFilterUpdate(&filter,1,1,bad_dt[i],&out)==FILTER_ERROR_DT && out==79);
            same_committed(&before,&filter);
            TEST_CHECK(memcmp(saved,history,sizeof(history))==0);
            TEST_CHECK(SampleFilterGetState(&filter,&state)==FILTER_OK && !state.valid && state.needs_rebuild);
            TEST_CHECK(SampleFilterUpdate(&filter,1,-5,NAN,&out)==FILTER_REBUILT && out==-5);
            TEST_CHECK(SampleFilterUpdate(&filter,1,-5,c.dt_min_s,&out)==FILTER_OK);
            TEST_CHECK(SampleFilterUpdate(&filter,1,-5,c.dt_max_s,&out)==FILTER_OK);
        }
        const float bad_value[]={NAN,INFINITY,-INFINITY};
        for(size_t i=0;i<3;++i)
        {
            before=filter;out=79;
            TEST_CHECK(SampleFilterUpdate(&filter,1,bad_value[i],.001f,&out)==FILTER_ERROR_NONFINITE && out==79);
            same_committed(&before,&filter);
            TEST_CHECK(SampleFilterUpdate(&filter,1,2,0,&out)==FILTER_REBUILT && out==2);
        }
        before=filter;out=79;
        TEST_CHECK(SampleFilterUpdate(&filter,0,NAN,NAN,&out)==FILTER_NO_SAMPLE && out==79);
        same_committed(&before,&filter);
        TEST_CHECK(SampleFilterGetState(&filter,&state)==FILTER_OK && !state.valid && state.last_status==FILTER_NO_SAMPLE);
        before=filter;
        for(unsigned i=0;i<10;++i)TEST_CHECK(SampleFilterGetState(&filter,&state)==FILTER_OK);
        TEST_CHECK(memcmp(&before,&filter,sizeof(filter))==0);
        TEST_CHECK(SampleFilterUpdate(&filter,1,7,0,&out)==FILTER_REBUILT && out==7);
        TEST_CHECK(SampleFilterReset(&filter)==FILTER_OK);
        TEST_CHECK(SampleFilterGetState(&filter,&state)==FILTER_OK && state.accepted_count==0 && !state.valid);
    }
    // 合法极值阶跃导致Butterworth越过float范围时，仍不能发布部分状态。
    SampleFilter_t filter={0};SampleFilterConfig_t c=config_for(SAMPLE_FILTER_LOWPASS2);
    initialize(&filter,&c,NULL,0);
    TEST_CHECK(SampleFilterUpdate(&filter,1,FLT_MAX,0,&out)==FILTER_REBUILT);
    int overflow=0;
    for(unsigned n=0;n<100;++n)
    {
        SampleFilter_t before=filter;out=79;
        SampleFilterStatus_t status=SampleFilterUpdate(&filter,1,-FLT_MAX,.001f,&out);
        if(status==FILTER_ERROR_NUMERIC)
        {same_committed(&before,&filter);TEST_CHECK(out==79 && !filter._result.valid);overflow=1;break;}
        TEST_CHECK(status==FILTER_OK);
    }
    TEST_CHECK(overflow);
    TEST_CHECK(SampleFilterUpdate(&filter,1,4,0,&out)==FILTER_REBUILT && out==4);
}

typedef struct { SampleFilter_t filter; float history[9]; Reference reference; float bias; } Worker;
static void worker_init(Worker *worker,SampleFilterType_t type,float bias)
{
    SampleFilterConfig_t c=config_for(type);worker->bias=bias;worker->reference.config=c;
    initialize(&worker->filter,&c,type==SAMPLE_FILTER_AVERAGE?worker->history:NULL,type==SAMPLE_FILTER_AVERAGE?9:0);
}
static void worker_step(Worker *worker,unsigned n)
{
    float value=worker->bias+(float)sin(n*.07),dt=(n&1u)?.0006f:.0014f,out;
    TEST_CHECK(SampleFilterUpdate(&worker->filter,1,value,dt,&out)==(n==0?FILTER_REBUILT:FILTER_OK));
    near(out,reference_update(&worker->reference,value,dt));
}
#ifdef FILTER_TEST_THREADS
static void *thread_run(void *argument)
{
    Worker *worker=argument;
    for(unsigned n=0;n<10000;++n)worker_step(worker,n);
    return NULL;
}
#endif
static void instances(void)
{
    Worker workers[4]={0};
    for(unsigned i=0;i<4;++i)worker_init(&workers[i],(SampleFilterType_t)(i+1),(float)i*5);
    for(unsigned n=0;n<10000;++n)for(unsigned i=0;i<4;++i)worker_step(&workers[i],n);
#ifdef FILTER_TEST_THREADS
    Worker parallel[4]={0};pthread_t threads[4];
    for(unsigned i=0;i<4;++i)
    {
        worker_init(&parallel[i],(SampleFilterType_t)(i+1),-(float)i*5);
        TEST_CHECK(pthread_create(&threads[i],NULL,thread_run,&parallel[i])==0);
    }
    for(unsigned i=0;i<4;++i)TEST_CHECK(pthread_join(threads[i],NULL)==0);
#endif
}

// 解析传递函数独立于状态更新；z^-1对应一个采样间隔。
static double complex transfer(const SampleFilterConfig_t *c,double frequency,double dt)
{
    double complex delay=cexp(-I*2*pi*frequency*dt);
    if(c->type==SAMPLE_FILTER_LOWPASS1)
    {double alpha=dt/(c->tau_s+dt);return alpha/(1-(1-alpha)*delay);}
    if(c->type==SAMPLE_FILTER_AVERAGE)
    {double complex sum=0;for(size_t j=0;j<c->window_length;++j)sum+=cpow(delay,(double)j);return sum/(double)c->window_length;}
    double g=tan(pi*c->frequency_hz*dt),k=c->type==SAMPLE_FILTER_LOWPASS2?sqrt(2.0):1.0/c->quality_factor;
    double complex s=(1-delay)/(1+delay),denominator=s*s+k*g*s+g*g;
    return (c->type==SAMPLE_FILTER_LOWPASS2?g*g:s*s+g*g)/denominator;
}

static void responses(FILE *csv,unsigned points)
{
    if(csv)fprintf(csv,"kind,frequency_hz,gain,reference_gain,phase_rad,phase_delay_ms\n");
    for(int type=SAMPLE_FILTER_LOWPASS1;type<=SAMPLE_FILTER_NOTCH;++type)
    {
        SampleFilterConfig_t c=config_for((SampleFilterType_t)type);
        for(unsigned point=1;point<=points;++point)
        {
            double frequency=points==80?2.5*point:(double[]){1,5,20,40,100,200}[point-1];
            SampleFilter_t filter={0};float history[9],out;
            initialize(&filter,&c,type==SAMPLE_FILTER_AVERAGE?history:NULL,type==SAMPLE_FILTER_AVERAGE?9:0);
            TEST_CHECK(SampleFilterUpdate(&filter,1,0,0,&out)==FILTER_REBUILT);
            double real=0,imaginary=0;
            // 前8秒等待暂态消退，最后2秒包含整数周期，独立测量正弦增益和相位。
            for(unsigned n=1;n<=10000;++n)
            {
                double phase=2*pi*frequency*n*.001;
                TEST_CHECK(SampleFilterUpdate(&filter,1,(float)sin(phase),.001f,&out)==FILTER_OK);
                if(n>8000){real+=out*sin(phase)/1000;imaginary+=out*cos(phase)/1000;}
            }
            double complex expected=transfer(&c,frequency,(double).001f);
            // 实测输入按1ms生成，float dt与精确1ms之差也包含在容差内。
            TEST_CHECK(fabs(real-creal(expected))<2e-5 && fabs(imaginary-cimag(expected))<2e-5);
            double gain=hypot(real,imaginary),phase=atan2(imaginary,real);
            if(frequency==20 && type==SAMPLE_FILTER_LOWPASS2)TEST_CHECK(fabs(gain-sqrt(.5))<2e-5);
            if(frequency==20 && type==SAMPLE_FILTER_NOTCH)TEST_CHECK(gain<1e-4);
            if(csv)
            {
                fprintf(csv,"%d,%.9g,%.12g,%.12g,",type,frequency,gain,cabs(expected));
                if(gain>1e-4)fprintf(csv,"%.12g,%.12g",phase,-phase/(2*pi*frequency)*1000);
                fprintf(csv,"\n");
            }
        }
    }
}

static FILE *open_csv(const char *prefix,const char *suffix)
{
    char path[1024];int count=snprintf(path,sizeof(path),"%s_%s.csv",prefix,suffix);
    TEST_CHECK(count>0 && (size_t)count<sizeof(path));
    FILE *stream=fopen(path,"w");TEST_CHECK(stream!=NULL);return stream;
}

static void export_csv(const char *prefix)
{
    FILE *step=open_csv(prefix,"step");fprintf(step,"time_s,input,lowpass1,average,lowpass2,notch\n");
    SampleFilter_t filters[4]={0};float history[9];
    for(unsigned i=0;i<4;++i)
    {SampleFilterConfig_t c=config_for((SampleFilterType_t)(i+1));initialize(&filters[i],&c,i==1?history:NULL,i==1?9:0);}
    for(unsigned n=0;n<=500;++n)
    {
        float input=n>=50?1:0;fprintf(step,"%.6f,%.1f",n*.001,input);
        for(unsigned i=0;i<4;++i)
        {
            float out;TEST_CHECK(SampleFilterUpdate(&filters[i],1,input,.001f,&out)==(n==0?FILTER_REBUILT:FILTER_OK));
            fprintf(step,",%.9g",out);
        }
        fprintf(step,"\n");
    }
    TEST_CHECK(fclose(step)==0);
    FILE *frequency=open_csv(prefix,"frequency");responses(frequency,80);TEST_CHECK(fclose(frequency)==0);
}

/** @brief 按场景运行或导出CSV；未知参数返回失败，Release不移除检查。 */
int main(int argc,char **argv)
{
    TEST_CHECK(argc>=2);
#ifdef FILTER_TEST_ALLOC_WRAP
    unsigned before=allocation_calls;
#endif
    if(strcmp(argv[1],"reference")==0)reference_tests();
    else if(strcmp(argv[1],"failures")==0)failures();
    else if(strcmp(argv[1],"instances")==0)instances();
    else if(strcmp(argv[1],"response")==0)responses(NULL,6);
    else if(strcmp(argv[1],"export")==0){TEST_CHECK(argc==3);export_csv(argv[2]);return 0;}
    else return 1;
#ifdef FILTER_TEST_ALLOC_WRAP
    // 线程库可申请线程自身存储，其余场景完整检查公开计算接口没有分配／释放。
    if(strcmp(argv[1],"instances")!=0)TEST_CHECK(allocation_calls==before);
#endif
    return 0;
}
