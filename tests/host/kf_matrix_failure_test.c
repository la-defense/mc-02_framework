/** @file kf_matrix_failure_test.c
 * @brief Linux逐步注入矩阵失败；未注入的路径转发真实DSP，检查发布值不会混入部分校正。
 */
#include "test_check.h"
#include "kalman_filter.h"
#include <string.h>
static int calls,fail_at=-1;
#define WRAP_BINARY(name) \
arm_status __real_##name(const arm_matrix_instance_f32 *,const arm_matrix_instance_f32 *,arm_matrix_instance_f32 *); \
arm_status __wrap_##name(const arm_matrix_instance_f32 *a,const arm_matrix_instance_f32 *b,arm_matrix_instance_f32 *o) \
{ if(calls++==fail_at)return ARM_MATH_ARGUMENT_ERROR;return __real_##name(a,b,o); }
#define WRAP_UNARY(name) \
arm_status __real_##name(const arm_matrix_instance_f32 *,arm_matrix_instance_f32 *); \
arm_status __wrap_##name(const arm_matrix_instance_f32 *a,arm_matrix_instance_f32 *o) \
{ if(calls++==fail_at)return ARM_MATH_ARGUMENT_ERROR;return __real_##name(a,o); }
WRAP_BINARY(arm_mat_mult_f32)
WRAP_BINARY(arm_mat_add_f32)
WRAP_BINARY(arm_mat_sub_f32)
WRAP_UNARY(arm_mat_trans_f32)
WRAP_UNARY(arm_mat_inverse_f32)
static _Alignas(KF_WORKSPACE_ALIGNMENT) unsigned char w[KF_WORKSPACE_BYTES(1,0,1)];
int main(void)
{
    KalmanFilter_t k={0};KFConfig_t c={1,0,1,20,20,2,.02f,NULL,NULL,NULL};
    float x=0,p=1,z=2;KFStep_t s={.001f,1,1,1,&z,NULL,1,NULL};
    TEST_CHECK(KFInit(&k,&c,w,sizeof(w),&x,&p)==KF_OK);
    k.F_data[0]=k.H_data[0]=k.R_data[0]=1;
    calls=0;TEST_CHECK(KFUpdate(&k,&s)==KF_OK);int count=calls;TEST_CHECK(count>15);
    // 首次校正前任一矩阵失败都不能发布；已校正后校正失败允许有限预测。
    for(int phase=0;phase<2;phase++) for(int i=0;i<count;i++) {
        fail_at=-1;TEST_CHECK(KFInit(&k,&c,w,sizeof(w),&x,&p)==KF_OK);
        k.F_data[0]=k.H_data[0]=k.R_data[0]=1;s.now_ms=1;s.measurement_ms=1;s.measurement_sequence=1;
        if(phase)TEST_CHECK(KFUpdate(&k,&s)==KF_OK);
        float old=k.FilteredValue[0],oldp=k.published_P[0];
        s.now_ms=2;s.measurement_ms=2;s.measurement_sequence=2;calls=0;fail_at=i;
        KFStatus_t status=KFUpdate(&k,&s);
        TEST_CHECK(status!=KF_OK && k.FilteredValue[0]==old && k.published_P[0]==oldp);
        if(k.fatal)TEST_CHECK(!k.result.valid);
        else if(phase)TEST_CHECK(status==KF_PREDICTED && k.result.cause==KF_ERROR_MATRIX && k.result.measurement_ms==1);
        else TEST_CHECK(!k.result.valid);
    }
    return 0;
}
