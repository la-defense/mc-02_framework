/** @file numeric_migration_trace.c
 * @brief 用同一组输入输出数值迁移前后的可比较轨迹；只供主机离线验证。
 * @note 分别链接对应版本的真实算法与 CMSIS-DSP。旧版本定义 OLD_VERSION，
 *       include 路径指向该版本的 tests/host、modules/algorithm 和 DSP 头。
 *       两次输出均使用九位有效数字，可无损区分有限的 float 数值。
 */
#include <stdio.h>
#include <math.h>
#include "arm_math.h"
#include "dsp_matrix.h"
#ifndef OLD_VERSION
#include "fast_trig.h"
#endif

// 复用两版已有的测试输入工厂；改名避免测试程序的 main 与本工具冲突。
#define main previous_kf_test_main
#include "kalman_filter_init_test.c"
#undef main
#define main previous_attitude_test_main
#include "attitude_filter_test.c"
#undef main
#define main previous_rls_test_main
#include "rls_test.c"
#undef main

int main(void)
{
    float a[4] = {3.0f, 1.0f, 1.0f, 2.0f}; // 非奇异输入矩阵。
    float b[4] = {2.0f, 3.0f, -1.0f, 4.0f}; // 矩阵乘法的第二个输入。
    float result[4] = {0}; // 求逆失败时保留前一次结果，供两版比较。
    float scratch[8]; // 两行两列矩阵求逆所需的独立工作区。
#ifdef OLD_VERSION
    arm_matrix_instance_f32 ma = {2, 2, a}, mb = {2, 2, b}, mo = {2, 2, result};
    int status = arm_mat_mult_f32(&ma, &mb, &mo); // 迁移前直接调用 CMSIS。
#else
    DSPMatrixF32_t ma = {2, 2, a}, mb = {2, 2, b}, mo = {2, 2, result};
    int status = DSPMatrixMultiplyF32(&ma, &mb, &mo); // 迁移后经框架接口调用。
#endif
    printf("matrix %d %.9g %.9g %.9g %.9g\n", status, result[0], result[1], result[2], result[3]);
    status = DSPMatrixInverseF32(&ma, &mo, scratch, 8);
    printf("inverse %d %.9g %.9g %.9g %.9g\n", status, result[0], result[1], result[2], result[3]);
    a[0] = a[1] = a[2] = a[3] = 1.0f; // 奇异矩阵应返回错误并保持上次结果。
    status = DSPMatrixInverseF32(&ma, &mo, scratch, 8);
    printf("singular %d %.9g\n", status, result[0]);

    // 覆盖正负角及周期边界，分别读取两版实际使用的正弦与余弦入口。
    for (int i = -128; i <= 128; ++i)
    {
        const float angle = (float)i * 0.09817477f; // 约 π/32 rad 的步长。
#ifdef OLD_VERSION
        const float sine = arm_sin_f32(angle), cosine = arm_cos_f32(angle);
#else
        const float sine = FastTrigSinF32(angle), cosine = FastTrigCosF32(angle);
#endif
        printf("trig %d %.9g %.9g\n", i, sine, cosine);
    }

    Fixture fixture = {0};
    init(&fixture, 1, 1); // 使用现有 KF 测试工厂创建一维滤波器。
    for (unsigned i = 0; i < 32; ++i)
    {
        float measurement = (float)((int)(i % 7) - 3) * 0.125f; // 周期变化的有限量测。
        const KFStatus_t step_status = step(&fixture, i + 1, i + 1, i + 1, &measurement, 1);
        printf("kf %u %d %.9g %.9g\n", i, step_status,
               fixture.k.FilteredValue[0], fixture.k.published_P[0]);
    }
    float invalid_measurement = NAN;
    status = step(&fixture, 33, 33, 33, &invalid_measurement, 1); // 比较非法量测的处理。
    printf("kf_bad %d %.9g %.9g %d\n", status, fixture.k.FilteredValue[0],
           fixture.k.published_P[0], fixture.k.result.cause);

    AttitudeFilter_t filter = {0};
    AttitudeFilterConfig_t config = make_config(ATTITUDE_BACKEND_QUATERNION_EKF);
    AttitudeFilterState_t state = {0};
    max_align_t workspace[(ATTITUDE_FILTER_STORAGE_BYTES + sizeof(max_align_t) - 1) / sizeof(max_align_t)];
    float quaternion[4] = {1, 0, 0, 0}; // 两版使用同一初始姿态。
    status = AttitudeFilterInit(&filter, &config, quaternion, workspace, sizeof workspace);
    printf("att_init %d\n", status);
    for (unsigned i = 0; i < 32; ++i)
    {
        AttitudeFilterInput_t input = make_input(i + 1, i + 1); // 同一组采样时间与序号。
        input.gyroscope_rad_s[2] = (float)(i % 5) * 0.04f; // 输入周期变化的偏航角速度。
        status = AttitudeFilterUpdate(&filter, &input, &state);
        printf("att %u %d %d %.9g %.9g %.9g %.9g %.9g\n", i, status, state.filter.valid,
               state.quaternion[0], state.quaternion[1], state.quaternion[2],
               state.quaternion[3], state.yaw_deg);
    }
    AttitudeFilterInput_t invalid_input = make_input(33, 33);
    invalid_input.accelerometer_m_s2[2] = NAN; // 观察非法加速度时的预测结果。
    status = AttitudeFilterUpdate(&filter, &invalid_input, &state);
    printf("att_bad %d %d %.9g\n", status, state.filter.valid, state.yaw_deg);

    RLS_t rls = {0};
    RLSConfig_t rls_config = {2, 10.0f, 0.99f};
    _Alignas(double) unsigned char rls_workspace[RLS_WORKSPACE_CAPACITY(2)];
    float initial[2] = {0, 0};
    status = RLSInit(&rls, &rls_config, initial, 2, rls_workspace, sizeof rls_workspace);
    printf("rls_init %d\n", status);
    for (unsigned i = 0; i < 32; ++i)
    {
        float input[2] = {1.0f, (float)((int)(i % 9) - 4) * 0.25f};
        float params[2], covariance[4], gain[2];
        RLSStateInfo_t info;
        status = RLSUpdate(&rls, input, 2, 1.5f - 0.75f * input[1]); // 同一线性模型产生量测。
        if (RLSGetState(&rls, params, 2, covariance, 4, gain, 2, &info) != RLS_STATUS_OK)
            return 1; // 取状态失败时不能输出未初始化的数值。
        printf("rls %u %d %.9g %.9g %.9g %.9g\n", i, status,
               params[0], params[1], covariance[0], covariance[3]);
    }
    float invalid_input_rls[2] = {NAN, 0};
    status = RLSUpdate(&rls, invalid_input_rls, 2, 1.0f);
    printf("rls_bad %d\n", status);
    return 0;
}
