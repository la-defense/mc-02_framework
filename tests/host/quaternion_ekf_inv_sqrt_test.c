#include "test_check.h"

#include <math.h>
#include "QuaternionEKF.h"

int main(void)
{
    float initial_quaternion[4] = {1.0f, 0.0f, 0.0f, 0.0f};
    uint8_t init_result = IMU_QuaternionEKF_Init(initial_quaternion, 1.0f, 0.001f, 10.0f, 0.999f, 0.0f);
    TEST_CHECK(init_result == 1u);

    IMU_QuaternionEKF_Update(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 9.8f, 0.001f);
    TEST_CHECK(isfinite(QEKF_INS.accl_norm));
    TEST_CHECK(fabsf(QEKF_INS.accl_norm - 9.8f) < 0.05f);

    Kalman_Filter_Release(&QEKF_INS.IMU_QuaternionEKF);
    return 0;
}
