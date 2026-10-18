/** @file math_utils_test.c
 * @brief 在主机执行真实数值入口，对照独立参考并检查失败保留、重叠和独立历史。
 * @note 由CTest传入scalars／vectors／average／instances；Release仍使用TEST_CHECK。
 */
#include "math_utils.h"
#include "sample_filter.h"
#include "vector3.h"
#include "test_check.h"
#include <float.h>
#include <limits.h>
#include <math.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#ifdef MATH_TEST_THREADS
#include <pthread.h>
#endif

#ifdef MATH_TEST_ALLOC_WRAP
static unsigned allocations;
void *__real_malloc(size_t n);
void *__real_calloc(size_t n, size_t s);
void *__real_realloc(void *p, size_t n);
void __real_free(void *p);
void *__wrap_malloc(size_t n) { ++allocations; return __real_malloc(n); }
void *__wrap_calloc(size_t n,size_t s) { ++allocations; return __real_calloc(n,s); }
void *__wrap_realloc(void *p,size_t n) { ++allocations; return __real_realloc(p,n); }
void __wrap_free(void *p) { ++allocations; __real_free(p); }
#endif

// 用绝对加相对误差比较，极小数另用精确断言覆盖。
static void close_to(float actual, double reference)
{
    TEST_CHECK(isfinite(actual));
    TEST_CHECK(fabs((double)actual-reference) <= 2e-6 + 2e-6*fabs(reference));
}

static void scalars(void)
{
    float out=71.0f;
    int16_t small=99;
    int32_t integer=71;
    uint8_t equal=7;
    TEST_CHECK(MathSqrtF32(0,&out)==MATH_OK && out==0);
    TEST_CHECK(MathSqrtF32(FLT_MAX,&out)==MATH_OK); close_to(out,sqrt((double)FLT_MAX));
    TEST_CHECK(MathSqrtF32(FLT_TRUE_MIN,&out)==MATH_OK); close_to(out,sqrt((double)FLT_TRUE_MIN));
    TEST_CHECK(out>0 && fabs(out/sqrt((double)FLT_TRUE_MIN)-1)<2e-6);
    TEST_CHECK(MathClampF32(3,-1,1,&out)==MATH_OK && out==1);
    TEST_CHECK(MathClampF32(-3,-1,1,&out)==MATH_OK && out==-1);
    TEST_CHECK(MathClampF32(0,-1,1,&out)==MATH_OK && out==0);
    TEST_CHECK(MathClampF32(3,2,2,&out)==MATH_OK && out==2);
    TEST_CHECK(MathClampI16(INT16_MIN,-10,10,&small)==MATH_OK && small==-10);
    TEST_CHECK(MathClampI16(INT16_MAX,-10,10,&small)==MATH_OK && small==10);
    TEST_CHECK(MathClampI16(0,0,0,&small)==MATH_OK && small==0);
    TEST_CHECK(MathAbsLimitF32(-3,2,&out)==MATH_OK && out==-2);
    TEST_CHECK(MathAbsLimitF32(3,0,&out)==MATH_OK && out==0);
    TEST_CHECK(MathSignF32(-0.0f,&out)==MATH_OK && out==1);
    TEST_CHECK(MathSignF32(-FLT_TRUE_MIN,&out)==MATH_OK && out==-1);
    TEST_CHECK(MathDeadbandF32(0,-1,1,&out)==MATH_OK && out==0);
    TEST_CHECK(MathDeadbandF32(1,-1,1,&out)==MATH_OK && out==1);
    TEST_CHECK(MathDeadbandF32(-1,-1,1,&out)==MATH_OK && out==-1);
    TEST_CHECK(MathDeadbandF32(2,2,2,&out)==MATH_OK && out==2);
    TEST_CHECK(MathNearlyEqualF32(FLT_MAX,-FLT_MAX,FLT_MAX,&equal)==MATH_OK && !equal);
    TEST_CHECK(MathNearlyEqualF32(1,1.25f,.25f,&equal)==MATH_OK && equal);
    TEST_CHECK(MathNearlyEqualF32(0,-0.0f,0,&equal)==MATH_OK && equal);

    const float rounds[]={0,-0.0f,.5f,-.5f,1.5f,-1.5f,2.5f,-2.5f,
        nextafterf(.5f,0),nextafterf(.5f,1),nextafterf(-.5f,0),nextafterf(-.5f,-1),
        2147483520.0f,-2147483648.0f};
    for(size_t i=0;i<sizeof(rounds)/sizeof(rounds[0]);++i)
    {
        TEST_CHECK(MathRoundI32(rounds[i],&integer)==MATH_OK);
        TEST_CHECK(integer==(int32_t)roundl((long double)rounds[i]));
    }
    // 网格还覆盖多个相邻半数，防止只修复正数而遗漏负数。
    for(int i=-10000;i<=10000;++i)
    {
        float value=(float)i/8.0f;
        TEST_CHECK(MathRoundI32(value,&integer)==MATH_OK);
        TEST_CHECK(integer==(int32_t)roundl(value));
        TEST_CHECK(MathWrapDegreesF32(value,&out)==MATH_OK);
        double reference=value;
        while(reference < -180) reference+=360;
        while(reference >= 180) reference-=360;
        close_to(out,reference);
    }
    TEST_CHECK(MathWrapDegreesF32(180,&out)==MATH_OK && out==-180);
    TEST_CHECK(MathWrapDegreesF32(-540,&out)==MATH_OK && out==-180);
    const float pi=3.14159265358979323846f;
    TEST_CHECK(MathWrapRadiansF32(pi,&out)==MATH_OK && out==-pi);
    TEST_CHECK(MathWrapF32(4,3,4,&out)==MATH_OK && out==3);
    TEST_CHECK(MathWrapF32(FLT_MAX,-180,180,&out)==MATH_OK);
    long double remainder=fmodl((long double)FLT_MAX,360.0L);
    if(remainder>=180)remainder-=360;
    close_to(out,(double)remainder);
    TEST_CHECK(MathWrapF32(-FLT_MAX,-FLT_MAX,FLT_MAX,&out)==MATH_OK && out==-FLT_MAX);
    TEST_CHECK(MathWrapF32(FLT_MAX,-FLT_MAX,FLT_MAX,&out)==MATH_OK && out==-FLT_MAX);
    float adjacent=nextafterf(1.0f,2.0f);
    TEST_CHECK(MathWrapF32(FLT_MAX,1,adjacent,&out)==MATH_OK && out>=1 && out<adjacent);

    out=71;small=99;integer=71;equal=7;
    TEST_CHECK(MathSqrtF32(-1,&out)==MATH_BAD_RANGE && out==71);
    TEST_CHECK(MathClampF32(0,2,1,&out)==MATH_BAD_RANGE && out==71);
    TEST_CHECK(MathClampI16(0,2,1,&small)==MATH_BAD_RANGE && small==99);
    TEST_CHECK(MathAbsLimitF32(0,-1,&out)==MATH_BAD_RANGE && out==71);
    TEST_CHECK(MathDeadbandF32(0,2,1,&out)==MATH_BAD_RANGE && out==71);
    TEST_CHECK(MathNearlyEqualF32(0,1,-1,&equal)==MATH_BAD_RANGE && equal==7);
    TEST_CHECK(MathWrapF32(1,0,0,&out)==MATH_BAD_RANGE && out==71);
    TEST_CHECK(MathWrapF32(1,2,1,&out)==MATH_BAD_RANGE && out==71);
    TEST_CHECK(MathRoundI32(2147483648.0f,&integer)==MATH_OVERFLOW && integer==71);
    TEST_CHECK(MathRoundI32(nextafterf(-2147483648.0f,-INFINITY),&integer)==MATH_OVERFLOW && integer==71);
    const float bad[]={NAN,INFINITY,-INFINITY};
    for(size_t i=0;i<3;++i)
    {
        TEST_CHECK(MathSqrtF32(bad[i],&out)==MATH_NONFINITE);
        TEST_CHECK(MathClampF32(bad[i],0,1,&out)==MATH_NONFINITE);
        TEST_CHECK(MathClampF32(0,bad[i],1,&out)==MATH_NONFINITE);
        TEST_CHECK(MathClampF32(0,0,bad[i],&out)==MATH_NONFINITE);
        TEST_CHECK(MathAbsLimitF32(0,bad[i],&out)==MATH_NONFINITE);
        TEST_CHECK(MathSignF32(bad[i],&out)==MATH_NONFINITE);
        TEST_CHECK(MathDeadbandF32(bad[i],0,1,&out)==MATH_NONFINITE);
        TEST_CHECK(MathDeadbandF32(0,bad[i],1,&out)==MATH_NONFINITE);
        TEST_CHECK(MathDeadbandF32(0,0,bad[i],&out)==MATH_NONFINITE);
        TEST_CHECK(MathNearlyEqualF32(bad[i],0,0,&equal)==MATH_NONFINITE);
        TEST_CHECK(MathNearlyEqualF32(0,bad[i],0,&equal)==MATH_NONFINITE);
        TEST_CHECK(MathNearlyEqualF32(0,0,bad[i],&equal)==MATH_NONFINITE);
        TEST_CHECK(MathRoundI32(bad[i],&integer)==MATH_NONFINITE);
        TEST_CHECK(MathWrapF32(bad[i],0,1,&out)==MATH_NONFINITE);
        TEST_CHECK(MathWrapF32(0,bad[i],1,&out)==MATH_NONFINITE);
        TEST_CHECK(MathWrapF32(0,0,bad[i],&out)==MATH_NONFINITE);
        TEST_CHECK(out==71 && integer==71 && equal==7);
    }
    TEST_CHECK(MathSqrtF32(1,NULL)==MATH_BAD_ARGUMENT);
    TEST_CHECK(MathClampF32(1,0,2,NULL)==MATH_BAD_ARGUMENT);
    TEST_CHECK(MathClampI16(1,0,2,NULL)==MATH_BAD_ARGUMENT);
    TEST_CHECK(MathAbsLimitF32(1,2,NULL)==MATH_BAD_ARGUMENT);
    TEST_CHECK(MathSignF32(1,NULL)==MATH_BAD_ARGUMENT);
    TEST_CHECK(MathDeadbandF32(1,0,2,NULL)==MATH_BAD_ARGUMENT);
    TEST_CHECK(MathNearlyEqualF32(1,1,0,NULL)==MATH_BAD_ARGUMENT);
    TEST_CHECK(MathRoundI32(1,NULL)==MATH_BAD_ARGUMENT);
    TEST_CHECK(MathWrapF32(1,0,2,NULL)==MATH_BAD_ARGUMENT);
}

static void vectors(void)
{
    float a[3]={3,4,0},b[3]={-2,5,7},out[3]={71,72,73},scalar=71;
    TEST_CHECK(Vec3NormF32(a,&scalar)==MATH_OK && scalar==5);
    TEST_CHECK(Vec3NormalizeF32(a,out)==MATH_OK);close_to(out[0],.6);close_to(out[1],.8);
    TEST_CHECK(Vec3DotF32(a,b,&scalar)==MATH_OK && scalar==14);
    TEST_CHECK(Vec3CrossF32(a,b,out)==MATH_OK);
    TEST_CHECK(out[0]==28 && out[1]==-21 && out[2]==23);
    // 随机但可重复的向量对照独立double公式。
    uint32_t seed=19;
    for(unsigned n=0;n<1000;++n)
    {
        for(size_t i=0;i<3;++i)
        {seed=seed*1664525u+1013904223u;a[i]=(float)(int)(seed%2001u)-1000;
         seed=seed*1664525u+1013904223u;b[i]=(float)(int)(seed%2001u)-1000;}
        double norm=hypot(hypot(a[0],a[1]),a[2]);
        TEST_CHECK(Vec3NormF32(a,&scalar)==MATH_OK);close_to(scalar,norm);
        TEST_CHECK(Vec3NormalizeF32(a,out)==MATH_OK);
        for(size_t i=0;i<3;++i)close_to(out[i],a[i]/norm);
        TEST_CHECK(Vec3DotF32(a,b,&scalar)==MATH_OK);
        close_to(scalar,(double)a[0]*b[0]+(double)a[1]*b[1]+(double)a[2]*b[2]);
        TEST_CHECK(Vec3CrossF32(a,b,out)==MATH_OK);
        for(size_t i=0;i<3;++i)
        {size_t j=(i+1)%3,k=(i+2)%3;close_to(out[i],(double)a[j]*b[k]-(double)a[k]*b[j]);}
    }
    float overlap[7]={3,4,0,0,-2,5,7};
    TEST_CHECK(Vec3CrossF32(overlap,overlap+4,overlap+1)==MATH_OK);
    TEST_CHECK(overlap[1]==28 && overlap[2]==-21 && overlap[3]==23);
    float exact_a[3]={3,4,0},exact_b[3]={-2,5,7};
    TEST_CHECK(Vec3CrossF32(exact_a,exact_b,exact_b)==MATH_OK);
    TEST_CHECK(exact_b[0]==28 && exact_b[1]==-21 && exact_b[2]==23);
    TEST_CHECK(Vec3DotF32(exact_a,exact_a,exact_a+1)==MATH_OK && exact_a[1]==25);
    TEST_CHECK(Vec3NormF32(exact_b,exact_b)==MATH_OK);
    close_to(exact_b[0],sqrt(28.0*28+21.0*21+23.0*23));
    float in_place[3]={3,4,0};
    TEST_CHECK(Vec3NormalizeF32(in_place,in_place)==MATH_OK);close_to(in_place[0],.6);
    float back[4]={0,3,4,0};
    TEST_CHECK(Vec3NormalizeF32(back+1,back)==MATH_OK);close_to(back[1],.8);
    float tiny[3]={FLT_TRUE_MIN,0,0};
    TEST_CHECK(Vec3NormF32(tiny,&scalar)==MATH_OK && scalar==FLT_TRUE_MIN);
    TEST_CHECK(Vec3NormalizeF32(tiny,tiny)==MATH_OK && tiny[0]==1);
    float huge[3]={FLT_MAX,FLT_MAX,0};
    scalar=71;
    TEST_CHECK(Vec3NormF32(huge,&scalar)==MATH_OVERFLOW && scalar==71);
    TEST_CHECK(Vec3NormalizeF32(huge,out)==MATH_OK);close_to(out[0],sqrt(.5));
    TEST_CHECK(Vec3CrossF32(huge,huge,out)==MATH_OK && out[0]==0 && out[1]==0 && out[2]==0);
    float zero[3]={0};
    TEST_CHECK(Vec3NormF32(zero,&scalar)==MATH_OK && scalar==0);
    float saved[3]={71,72,73};memcpy(out,saved,sizeof(out));scalar=71;
    TEST_CHECK(Vec3NormalizeF32(zero,out)==MATH_ZERO_VECTOR && memcmp(saved,out,sizeof(out))==0);
    float x[3]={FLT_MAX,0,0},y[3]={0,FLT_MAX,0};
    TEST_CHECK(Vec3CrossF32(x,y,out)==MATH_OVERFLOW && memcmp(saved,out,sizeof(out))==0);
    TEST_CHECK(Vec3DotF32(x,x,&scalar)==MATH_OVERFLOW && scalar==71);
    for(size_t i=0;i<3;++i)
    {
        float bad[3]={1,2,3};bad[i]=i==0?NAN:(i==1?INFINITY:-INFINITY);
        TEST_CHECK(Vec3NormF32(bad,&scalar)==MATH_NONFINITE && scalar==71);
        TEST_CHECK(Vec3NormalizeF32(bad,out)==MATH_NONFINITE);
        TEST_CHECK(Vec3DotF32(x,bad,&scalar)==MATH_NONFINITE && scalar==71);
        TEST_CHECK(Vec3CrossF32(bad,y,out)==MATH_NONFINITE);
        TEST_CHECK(Vec3CrossF32(x,bad,out)==MATH_NONFINITE && memcmp(saved,out,sizeof(out))==0);
    }
    TEST_CHECK(Vec3NormF32(NULL,&scalar)==MATH_BAD_ARGUMENT);
    TEST_CHECK(Vec3NormF32(x,NULL)==MATH_BAD_ARGUMENT);
    TEST_CHECK(Vec3NormalizeF32(NULL,out)==MATH_BAD_ARGUMENT);
    TEST_CHECK(Vec3NormalizeF32(x,NULL)==MATH_BAD_ARGUMENT);
    TEST_CHECK(Vec3DotF32(NULL,y,&scalar)==MATH_BAD_ARGUMENT);
    TEST_CHECK(Vec3DotF32(x,NULL,&scalar)==MATH_BAD_ARGUMENT);
    TEST_CHECK(Vec3DotF32(x,y,NULL)==MATH_BAD_ARGUMENT);
    TEST_CHECK(Vec3CrossF32(NULL,y,out)==MATH_BAD_ARGUMENT);
    TEST_CHECK(Vec3CrossF32(x,NULL,out)==MATH_BAD_ARGUMENT);
    TEST_CHECK(Vec3CrossF32(x,y,NULL)==MATH_BAD_ARGUMENT);
}

static void average(void)
{
    float buffer[256]={0},out=71;
    for(unsigned i=1;i<=8;++i)
    {
        TEST_CHECK(MovingAverageF32((float)i,buffer,256,5,&out)==MATH_OK);
        unsigned first=i>5?i-4:1;double sum=0;
        for(unsigned j=first;j<=i;++j)sum+=j;
        close_to(out,sum/5);
    }
    TEST_CHECK(MovingAverageF32(-3,buffer,1,1,&out)==MATH_OK && out==-3 && buffer[0]==-3);
    for(size_t i=0;i<255;++i)buffer[i]=FLT_MAX;
    TEST_CHECK(MovingAverageF32(FLT_MAX,buffer,255,255,&out)==MATH_OK && out==FLT_MAX);
    float saved[256];memcpy(saved,buffer,sizeof(buffer));out=71;
    TEST_CHECK(MovingAverageF32(0,NULL,5,5,&out)==MATH_BAD_ARGUMENT);
    TEST_CHECK(MovingAverageF32(0,buffer,5,5,NULL)==MATH_BAD_ARGUMENT);
    TEST_CHECK(MovingAverageF32(0,buffer,5,0,&out)==MATH_BAD_RANGE);
    TEST_CHECK(MovingAverageF32(0,buffer,256,256,&out)==MATH_BAD_RANGE);
    TEST_CHECK(MovingAverageF32(0,buffer,4,5,&out)==MATH_BAD_ARGUMENT);
    TEST_CHECK(MovingAverageF32(0,buffer,5,5,buffer+4)==MATH_BAD_ARGUMENT);
    TEST_CHECK(MovingAverageF32(0,(float *)((uintptr_t)buffer+1),5,5,&out)==MATH_BAD_ARGUMENT);
    TEST_CHECK(MovingAverageF32(0,buffer,5,5,(float *)((uintptr_t)&out+1))==MATH_BAD_ARGUMENT);
    TEST_CHECK(MovingAverageF32(0,(float *)(UINTPTR_MAX-3u),5,5,&out)==MATH_BAD_ARGUMENT);
    TEST_CHECK(memcmp(saved,buffer,sizeof(buffer))==0 && out==71);
    const float bad[]={NAN,INFINITY,-INFINITY};
    for(size_t i=0;i<3;++i)
    {
        TEST_CHECK(MovingAverageF32(bad[i],buffer,5,5,&out)==MATH_NONFINITE);
        TEST_CHECK(memcmp(saved,buffer,sizeof(buffer))==0 && out==71);
        buffer[i]=bad[i];memcpy(saved,buffer,sizeof(buffer));
        TEST_CHECK(MovingAverageF32(0,buffer,5,5,&out)==MATH_NONFINITE);
        TEST_CHECK(memcmp(saved,buffer,sizeof(buffer))==0 && out==71);
        buffer[i]=FLT_MAX;memcpy(saved,buffer,sizeof(buffer));
    }
    // 修复历史后可立即恢复，不需要共享初始化流程。
    memset(buffer,0,sizeof(buffer));
    TEST_CHECK(MovingAverageF32(10,buffer,5,5,&out)==MATH_OK && out==2);
}

typedef struct { float bias,buffer[7]; } Worker;
static void step(Worker *worker,unsigned index)
{
    float sample=worker->bias+(float)(index%11u),out;
    TEST_CHECK(MovingAverageF32(sample,worker->buffer,7,7,&out)==MATH_OK);
    double sum=0;for(size_t j=0;j<7;++j)sum+=worker->buffer[j];close_to(out,sum/7);
    int32_t rounded;TEST_CHECK(MathRoundI32(sample+.5f,&rounded)==MATH_OK);
    TEST_CHECK(rounded==(int32_t)roundl((long double)sample+.5L));
}
#ifdef MATH_TEST_THREADS
static void *run_worker(void *argument)
{
    Worker *worker=argument;
    for(unsigned i=0;i<3000;++i)step(worker,i);
    return NULL;
}
#endif
static void instances(void)
{
    Worker a={.bias=10},b={.bias=-30};
    for(unsigned i=0;i<3000;++i){step(&a,i);step(&b,i);}
#ifdef MATH_TEST_THREADS
    pthread_t ta,tb;
    TEST_CHECK(pthread_create(&ta,NULL,run_worker,&a)==0);
    TEST_CHECK(pthread_create(&tb,NULL,run_worker,&b)==0);
    TEST_CHECK(pthread_join(ta,NULL)==0 && pthread_join(tb,NULL)==0);
#endif
}

/** @brief 按CTest场景执行，未指定或名称错误时返回失败。 */
int main(int argc,char **argv)
{
    TEST_CHECK(argc==2);
#ifdef MATH_TEST_ALLOC_WRAP
    unsigned before=allocations;
#endif
    if(strcmp(argv[1],"scalars")==0)scalars();
    else if(strcmp(argv[1],"vectors")==0)vectors();
    else if(strcmp(argv[1],"average")==0)average();
    else if(strcmp(argv[1],"instances")==0)instances();
    else return 1;
#ifdef MATH_TEST_ALLOC_WRAP
    // 线程库可以分配线程自身存储，计算入口的无分配检查由其余三个场景覆盖。
    if(strcmp(argv[1],"instances")!=0)TEST_CHECK(allocations==before);
#endif
    return 0;
}
