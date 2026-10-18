/** @file quaternion_ekf_inv_sqrt_test.c
 * @brief 用真实DSP检查四元数、失效和恢复；无硬件依赖，Release仍检查。
 */
#include "test_check.h"
#include "QuaternionEKF.h"
#include <math.h>
#include <string.h>
static _Alignas(KF_WORKSPACE_ALIGNMENT) unsigned char workspace[KF_WORKSPACE_BYTES(6,0,3)];
int main(void)
{
    float initial[4]={1,0,0,0};
    QEKFConfig_t c={10,.001f,1000000,1,0,20,2,.020f};
    TEST_CHECK(IMU_QuaternionEKF_Init(initial,&c,workspace,sizeof(workspace))==KF_OK);
    TEST_CHECK(IMU_QuaternionEKF_Update(0,0,0,0,0,0,.001f,0,0,0)<0);
    TEST_CHECK(IMU_QuaternionEKF_Update(0,0,0,0,0,9.8f,.001f,1,1,1)==KF_OK);
    TEST_CHECK(isfinite(QEKF_INS.accl_norm) && fabsf(QEKF_INS.accl_norm-9.8f)<.05f);
    for(unsigned i=2;i<100;i++)
        TEST_CHECK(IMU_QuaternionEKF_Update(0,0,0,0,0,9.8f,.001f,i,i,i)==KF_OK);
    TEST_CHECK(QEKF_INS.IMU_QuaternionEKF.result.mode==KF_CORRECTED);
    TEST_CHECK(IMU_QuaternionEKF_Update(0,0,0,NAN,0,0,.001f,119,119,119)==KF_PREDICTED);
    TEST_CHECK(QEKF_INS.IMU_QuaternionEKF.result.measurement_ms==99);
    float q[4]; memcpy(q,QEKF_INS.q,sizeof(q));
    TEST_CHECK(IMU_QuaternionEKF_Update(0,0,0,0,0,0,.001f,120,120,120)==KF_PREDICTION_LIMIT);
    TEST_CHECK(memcmp(q,QEKF_INS.q,sizeof(q))==0 && !QEKF_INS.IMU_QuaternionEKF.result.valid);
    TEST_CHECK(IMU_QuaternionEKF_Update(0,0,0,0,0,9.8f,.001f,121,121,121)==KF_OK);
    // 持续拒绝的加速度不能因计数超过50而强行校正。
    for(unsigned i=122;i<190;i++) {
        KFStatus_t s=IMU_QuaternionEKF_Update(0,0,0,9.8f,0,0,.001f,i,i,i);
        TEST_CHECK(s!=KF_OK && QEKF_INS.IMU_QuaternionEKF.result.measurement_ms==121);
    }
    TEST_CHECK(IMU_QuaternionEKF_Update(0,0,0,0,0,9.8f,.001f,190,190,190)==KF_OK);
    memcpy(q,QEKF_INS.q,sizeof(q));
    TEST_CHECK(IMU_QuaternionEKF_Update(INFINITY,0,0,0,0,9.8f,.001f,191,191,191)<0);
    TEST_CHECK(!QEKF_INS.IMU_QuaternionEKF.result.valid && memcmp(q,QEKF_INS.q,sizeof(q))==0);
    TEST_CHECK(IMU_QuaternionEKF_Update(0,0,0,0,0,9.8f,.001f,192,192,192)==KF_ERROR_STATE);
    TEST_CHECK(IMU_QuaternionEKF_Init(initial,&c,workspace,sizeof(workspace))==KF_OK);
    TEST_CHECK(IMU_QuaternionEKF_Update(0,0,0,0,0,9.8f,.001f,193,193,193)==KF_OK);
    return 0;
}
