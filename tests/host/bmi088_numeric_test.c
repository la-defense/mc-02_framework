/** @file bmi088_numeric_test.c
 * @brief 运行生产BMI088标定，注入一次／持续模长失败及分配失败，检查计数和保存结果。
 * @note SPI和时间由既有fixture提供；未注入失败时使用真实向量计算。
 */
#include "bmi088_host_fixture.h"
#include "bmi088_regNdef.h"
#include "vector3.h"
#include "test_check.h"
#include <stdlib.h>
#include <string.h>
#include <math.h>
static unsigned norm_calls,rejected,allocation_fail;
static int fail_norm;
extern volatile uint32_t cali_diag_acq_ok;
MathStatus_t __real_Vec3NormF32(const float *,float *);
void *__real_calloc(size_t,size_t);
MathStatus_t __wrap_Vec3NormF32(const float *vector,float *output)
{
    ++norm_calls;
    if(fail_norm<0 || (fail_norm>0 && norm_calls==(unsigned)fail_norm))
    {++rejected;return MATH_NONFINITE;}
    return __real_Vec3NormF32(vector,output);
}
void *__wrap_calloc(size_t count,size_t size)
{
    if(allocation_fail)return NULL;
    return __real_calloc(count,size);
}

/** @brief 每个场景由独立进程启动，避免保留生产模块的全局初始化状态。 */
int main(int argc,char **argv)
{
    TEST_CHECK(argc==2);
    BMI088HostReset();BMI088HostSetKernelRunning(1);
    if(strcmp(argv[1],"allocation")==0)
    {
        allocation_fail=1;
        TEST_CHECK(BMI088HostRegisterOnline()==NULL);
        TEST_CHECK(bmi088_init_error==BMI088_NO_SENSOR);
        TEST_CHECK(bmi088_host_fixture.transfer_count==0);
        return 0;
    }
    fail_norm=strcmp(argv[1],"persistent")==0?-1:1;
    BMI088Instance *instance=BMI088HostRegisterOnline();
    TEST_CHECK(instance!=NULL);
    if(fail_norm<0)
    {
        TEST_CHECK(rejected>0 && rejected==norm_calls && norm_calls==cali_diag_acq_ok);
        TEST_CHECK(!bmi088_host_fixture.calibration_meta_committed);
        TEST_CHECK(!bmi088_host_fixture.calibration_heater_end_success);
        TEST_CHECK(!BMI088CalibIsValid());
        TEST_CHECK(bmi088_host_fixture.synthetic_time_us<13000000u);
    }
    else
    {
        TEST_CHECK(rejected==1 && norm_calls==6001 && cali_diag_acq_ok==6001);
        TEST_CHECK(BMI088CalibIsValid() && bmi088_host_fixture.calibration_meta_committed);
        TEST_CHECK(bmi088_host_fixture.calibration_group_committed);
        TEST_CHECK(bmi088_host_fixture.legacy_calibration_parameter_access_count == 0u);
        TEST_CHECK(isfinite(instance->gNorm) && instance->gNorm>9 && instance->gNorm<10.5f);
    }
    return 0;
}
