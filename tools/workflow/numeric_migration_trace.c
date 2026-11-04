/** @file numeric_migration_trace.c
 * @brief 用同一组输入输出数值迁移前后的可比较轨迹；只供主机离线验证。
 * @note 分别链接对应版本的真实算法与 CMSIS-DSP。旧版本定义 OLD_VERSION，
 *       include 路径指向该版本的 modules/algorithm 和 DSP 头。
 *       两次输出均使用九位有效数字，可无损区分有限的 float 数值。
 */
#include <stdio.h>
#include <math.h>
#include "arm_math.h"
#include "dsp_matrix.h"
#include "kalman_filter.h"
#include "attitude_filter.h"
#include "RLS.h"
#ifndef OLD_VERSION
#include "fast_trig.h"
#endif

/** @brief 一维 KF 实例及独占工作区，布局可同时用于两版接口。 */
typedef struct {
    KalmanFilter_t k;
    _Alignas(KF_WORKSPACE_ALIGNMENT) unsigned char w[KF_WORKSPACE_BYTES(2, 1, 3)];
} TraceFixture;

// 使用两版公共接口建立相同的一维模型，失败时不执行后续样本。
static KFStatus_t trace_init(TraceFixture *fixture)
{
    const KFConfig_t config = {1, 0, 1, 20u, 20u, 2.0f, 0.1f, NULL, NULL, NULL};
    const float initial[2] = {0, 0}, covariance[4] = {1, 0, 0, 1};
    const KFStatus_t status = KFInit(&fixture->k, &config, fixture->w,
                                     KF_WORKSPACE_BYTES(1, 0, 1), initial, covariance);
    if (status != KF_OK)
        return status;
    fixture->k.F_data[0] = 1.0f; // 状态转移采用单位矩阵。
    fixture->k.H_data[0] = 1.0f; // 直接测量该状态。
    fixture->k.R_data[0] = 1.0f; // 量测噪声协方差。
    return KF_OK;
}

// 每个量测使用相同的本机时刻、源时刻和递增序号。
static KFStatus_t trace_step(TraceFixture *fixture, uint32_t time, float *measurement)
{
    const KFStep_t step = {0.001f, time, time, time, measurement, NULL, 1, NULL};
    return KFUpdate(&fixture->k, &step);
}

// 姿态输入沿用现有后端测试的重力和样本时间约定。
static AttitudeFilterInput_t trace_input(uint32_t time)
{
    AttitudeFilterInput_t input = {0};
    input.dt_s = 0.001f; // 相邻样本间隔1 ms。
    input.now_ms = time;
    input.sample_time_ms = time;
    input.sequence = time;
    input.accelerometer_valid = 1u;
    input.accelerometer_m_s2[2] = 9.81f;
    return input;
}

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
    if (status != 0)
        return 1; // 两版同时失败也不能作为正确的矩阵乘法结果。
    status = DSPMatrixInverseF32(&ma, &mo, scratch, 8);
    printf("inverse %d %.9g %.9g %.9g %.9g\n", status, result[0], result[1], result[2], result[3]);
    if (status != 0)
        return 1; // 非奇异矩阵必须可以求逆。
    a[0] = a[1] = a[2] = a[3] = 1.0f; // 奇异矩阵应返回错误并保持上次结果。
    status = DSPMatrixInverseF32(&ma, &mo, scratch, 8);
    printf("singular %d %.9g\n", status, result[0]);
    if (status != -5)
        return 1; // 两版约定的奇异矩阵状态均为 -5。

    // 覆盖正负角及周期边界，分别读取两版实际使用的正弦与余弦入口。
    for (int i = -128; i <= 128; ++i)
    {
        const float angle = (float)i * 0.09817477f; // 约 π/32 rad 的步长。
#ifdef OLD_VERSION
        const float sine = arm_sin_f32(angle), cosine = arm_cos_f32(angle);
#else
        const float sine = FastTrigSinF32(angle), cosine = FastTrigCosF32(angle);
#endif
        if (!isfinite(sine) || !isfinite(cosine))
            return 1; // 合法角度不能产生非有限结果。
        printf("trig %d %.9g %.9g\n", i, sine, cosine);
    }

    TraceFixture fixture = {0};
    if (trace_init(&fixture) != KF_OK)
        return 1; // 初始化失败时没有可比较的轨迹。
    for (unsigned i = 0; i < 32; ++i)
    {
        float measurement = (float)((int)(i % 7) - 3) * 0.125f; // 周期变化的有限量测。
        const KFStatus_t step_status = trace_step(&fixture, i + 1, &measurement);
        printf("kf %u %d %.9g %.9g\n", i, step_status,
               fixture.k.FilteredValue[0], fixture.k.published_P[0]);
        if (step_status != KF_OK)
            return 1; // 合法新量测应完成校正。
    }
    float invalid_measurement = NAN;
    status = trace_step(&fixture, 33, &invalid_measurement); // 比较非法量测的处理。
    printf("kf_bad %d %.9g %.9g %d\n", status, fixture.k.FilteredValue[0],
           fixture.k.published_P[0], fixture.k.result.cause);
    if (status != KF_PREDICTED || fixture.k.result.cause != KF_ERROR_NUMERIC)
        return 1; // 非法量测使用有限预测并报告数值原因。

    AttitudeFilter_t filter = {0};
    AttitudeFilterConfig_t config = {0};
    config.backend = ATTITUDE_BACKEND_QUATERNION_EKF; // 比较四元数 EKF 后端。
    config.max_prediction_ms = 20u; // 允许缺少校正数据时预测20 ms。
    config.max_measurement_age_ms = 20u; // 加速度样本最大年龄20 ms。
    config.max_dt_s = 0.02f; // 最大采样间隔0.02 s。
    config.qekf.process_noise1 = 10.0f; // 四元数状态过程噪声。
    config.qekf.process_noise2 = 0.001f; // 陀螺偏置过程噪声。
    config.qekf.measure_noise = 1000000.0f; // 加速度量测噪声。
    config.qekf.lambda = 1.0f; // 协方差渐消系数。
    config.qekf.lpf = 0.0f; // 不额外低通加速度。
    config.qekf.variance_ratio = 2.0f; // 预测协方差允许的增长倍率。
    AttitudeFilterState_t state = {0};
    max_align_t workspace[(ATTITUDE_FILTER_STORAGE_BYTES + sizeof(max_align_t) - 1) / sizeof(max_align_t)];
    float quaternion[4] = {1, 0, 0, 0}; // 两版使用同一初始姿态。
    status = AttitudeFilterInit(&filter, &config, quaternion, workspace, sizeof workspace);
    printf("att_init %d\n", status);
    if (status != KF_OK)
        return 1; // 未成功初始化时不运行姿态轨迹。
    for (unsigned i = 0; i < 32; ++i)
    {
        AttitudeFilterInput_t input = trace_input(i + 1); // 同一组采样时间与序号。
        input.gyroscope_rad_s[2] = (float)(i % 5) * 0.04f; // 输入周期变化的偏航角速度。
        status = AttitudeFilterUpdate(&filter, &input, &state);
        printf("att %u %d %d %.9g %.9g %.9g %.9g %.9g\n", i, status, state.filter.valid,
               state.quaternion[0], state.quaternion[1], state.quaternion[2],
               state.quaternion[3], state.yaw_deg);
        if (status != KF_OK || state.filter.valid == 0u)
            return 1; // 有效样本应获得可用的校正姿态。
    }
    AttitudeFilterInput_t invalid_input = trace_input(33);
    invalid_input.accelerometer_m_s2[2] = NAN; // 观察非法加速度时的预测结果。
    status = AttitudeFilterUpdate(&filter, &invalid_input, &state);
    printf("att_bad %d %d %.9g\n", status, state.filter.valid, state.yaw_deg);
    if (status != KF_PREDICTED || state.filter.valid == 0u)
        return 1; // 一次非法加速度应进入仍在期限内的预测。

    RLS_t rls = {0};
    RLSConfig_t rls_config = {2, 10.0f, 0.99f};
    _Alignas(double) unsigned char rls_workspace[RLS_WORKSPACE_CAPACITY(2)];
    float initial[2] = {0, 0};
    status = RLSInit(&rls, &rls_config, initial, 2, rls_workspace, sizeof rls_workspace);
    printf("rls_init %d\n", status);
    if (status != RLS_STATUS_OK)
        return 1; // 未成功初始化时不执行辨识。
    for (unsigned i = 0; i < 32; ++i)
    {
        float input[2] = {1.0f, (float)((int)(i % 9) - 4) * 0.25f};
        float params[2], covariance[4], gain[2];
        RLSStateInfo_t info;
        status = RLSUpdate(&rls, input, 2, 1.5f - 0.75f * input[1]); // 同一线性模型产生量测。
        if (status != RLS_STATUS_OK)
            return 1; // 有效量测必须成功更新参数。
        if (RLSGetState(&rls, params, 2, covariance, 4, gain, 2, &info) != RLS_STATUS_OK)
            return 1; // 取状态失败时不能输出未初始化的数值。
        printf("rls %u %d %.9g %.9g %.9g %.9g\n", i, status,
               params[0], params[1], covariance[0], covariance[3]);
    }
    float invalid_input_rls[2] = {NAN, 0};
    status = RLSUpdate(&rls, invalid_input_rls, 2, 1.0f);
    printf("rls_bad %d\n", status);
    if (status != RLS_ERROR_NUMERIC)
        return 1; // 非有限输入必须明确拒绝。
    return 0;
}
