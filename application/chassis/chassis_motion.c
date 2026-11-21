/**
 * @file chassis_motion.c
 * @brief 按模式计算底盘转速，再将平移和转动换算为四轮电机轴速度。
 *
 * 供 ChassisTask 调用，只计算目标，不直接发送电机指令。
 * 当前沿用工程原有的四轮计算方式，参数和返回值见 chassis_motion.h。
 */
#include "chassis_motion.h"
#include "../config/build_selection.h"
#include "../../modules/general_def.h"

#include "fast_trig.h"

#include <float.h>
#include <math.h>
#include <string.h>

static const float mecanum_rotor_to_output_sign = -1.0f; // 麦轮旧转速接口换到驱动输出轴时反向。

#if !defined(MC02_CONFIG_GENERATED) || !MC02_CHASSIS_IS_OMNI
// 从固定运动学轮序映射到旧电机转速接口，数组下标是旧接口的电机槽位。
static const struct {
    uint8_t canonical_index; // 运动学使用的左前、右前、左后、右后编号。
    float sign; // 固定轮轴正向换到旧接口的正向。
} mecanum_legacy_wheel[CHASSIS_WHEEL_COUNT] = {
    {3u, -1.0f}, // 左前槽位使用反向的运动学右后轮。
    {1u, -1.0f}, // 右前槽位使用反向的运动学右前轮。
    {2u,  1.0f}, // 左后槽位使用同向的运动学左后轮。
    {0u,  1.0f}, // 右后槽位使用同向的运动学左前轮。
};
#endif

/* 数字大于0且不是 NaN 或无穷大才通过。 */
static uint8_t finite_positive(float value)
{
    return isfinite(value) && value > 0.0f;
}

/* 核对尺寸、速度范围和输入，避免除零或按未知模式计算。 */
static uint8_t input_is_valid(const ChassisMotionInput_s *input,
                              const ChassisMotionPrepared_s *prepared,
                              const ChassisMotionLimits_s *limits)
{
    return input != NULL && prepared != NULL && prepared->projector.ready == 1u && limits != NULL &&
           (prepared->model == CHASSIS_MOTION_MODEL_OMNI || prepared->model == CHASSIS_MOTION_MODEL_MECANUM) &&
           limits->configured &&
           finite_positive(prepared->reduction_ratio) &&
           finite_positive(limits->max_wz_deg_s) &&
           finite_positive(limits->max_motor_speed_deg_s) &&
           isfinite(limits->rotate_wz_deg_s) &&
           fabsf(limits->rotate_wz_deg_s) <= limits->max_wz_deg_s &&
           isfinite(input->vx_mm_s) && isfinite(input->vy_mm_s) &&
           isfinite(input->requested_wz_deg_s) &&
           isfinite(input->offset_angle_deg) &&
           input->offset_angle_deg >= -180.0f && input->offset_angle_deg <= 180.0f &&
           (input->mode == CHASSIS_MOTION_MODE_NO_FOLLOW ||
            input->mode == CHASSIS_MOTION_MODE_FOLLOW ||
            input->mode == CHASSIS_MOTION_MODE_ROTATE);
}

uint8_t ChassisMotionRotateToBody(const ChassisTwist_t *source, float frame_heading_rad,
                                   ChassisTwist_t *body)
{
    const float pi = 3.14159265358979323846f; // 单圈方向角用半圈作为正负端点。
    if (source == NULL || body == NULL || !isfinite(frame_heading_rad) || fabsf(frame_heading_rad) > pi ||
        !isfinite(source->vx_m_s) || !isfinite(source->vy_m_s) || !isfinite(source->wz_rad_s))
        return 0u; // 快速三角函数与单位转换只处理有效输入。
    const float cos_theta = FastTrigCosF32(frame_heading_rad); // 参照前方投影到底盘前向的比例。
    const float sin_theta = FastTrigSinF32(frame_heading_rad); // 参照前方投影到底盘左向的比例。
    const ChassisTwist_t rotated = {
        .vx_m_s = source->vx_m_s * cos_theta - source->vy_m_s * sin_theta, // 参照左轴在底盘前向的投影为负正弦。
        .vy_m_s = source->vx_m_s * sin_theta + source->vy_m_s * cos_theta, // 参照左轴在底盘左向的投影为余弦。
        .wz_rad_s = source->wz_rad_s, // 平移参照绕同一竖轴旋转，底盘角速度不变。
    };
    if (!isfinite(rotated.vx_m_s) || !isfinite(rotated.vy_m_s))
        return 0u; // 有限输入相加也可能溢出，失败时保留调用方原值。
    *body = rotated; // 完成两轴计算后一起交付，也允许原位转换。
    return 1u;
}

uint8_t ChassisMotionCalculate(const ChassisMotionInput_s *input,
                               const ChassisMotionGeometry_s *geometry,
                               const ChassisMotionLimits_s *limits,
                               ChassisMotionOutput_s *output)
{
    if (output == NULL) return 0u;
    ChassisMotionPrepared_s prepared; // 单次计算调用者仍可直接提供几何。
    if (ChassisMotionPrepare(geometry, &prepared))
        return ChassisMotionCalculatePrepared(input, &prepared, limits, output);
    *output = (ChassisMotionOutput_s){0}; // 启动式检查失败也交付清零输出。
    return 0u;
}

uint8_t ChassisMotionPrepare(const ChassisMotionGeometry_s *geometry, ChassisMotionPrepared_s *prepared)
{
    if (prepared == NULL) return 0u;
    prepared->projector.ready = 0u; // 不允许在准备失败后沿用旧车型。
    if (geometry == NULL || !finite_positive(geometry->reduction_ratio)) return 0u;
    ChassisKinematicsStatus_t status = CHASSIS_KINEMATICS_UNSUPPORTED; // 只使用已经编入的轮组模型。
#if !defined(MC02_CONFIG_GENERATED) || MC02_CHASSIS_IS_OMNI
    if (geometry->model == CHASSIS_MOTION_MODEL_OMNI)
        status = OmniPrepareProjector(&geometry->omni, &prepared->projector); // 保留每轮的单位滚动方向检查。
#endif
#if !defined(MC02_CONFIG_GENERATED) || !MC02_CHASSIS_IS_OMNI
    if (geometry->model == CHASSIS_MOTION_MODEL_MECANUM)
        status = MecanumPrepareProjector(&geometry->mecanum, &prepared->projector); // 半轴距、半轮距和偏移采用车型 SI 参数。
#endif
    if (status != CHASSIS_KINEMATICS_OK) return 0u;
    prepared->model = geometry->model; // 保留该车型需要的输出轮序。
    prepared->reduction_ratio = geometry->reduction_ratio; // 与反馈使用同一装车参数。
    return 1u;
}

ChassisKinematicsStatus_t ChassisMotionPrepareVelocityEstimator(const ChassisMotionGeometry_s *geometry,
    ChassisVelocityEstimator_t *estimator)
{
    if (estimator == NULL) return CHASSIS_KINEMATICS_ARGUMENT;
    estimator->ready = 0u; // 车型变更或检查失败时清除旧系数的可用标记。
    if (geometry == NULL) return CHASSIS_KINEMATICS_ARGUMENT;
#if !defined(MC02_CONFIG_GENERATED) || MC02_CHASSIS_IS_OMNI
    if (geometry->model == CHASSIS_MOTION_MODEL_OMNI)
        return OmniPrepareEstimator(&geometry->omni, estimator); // 全向轮的驱动反馈已采用固定轮序和正向。
#endif
#if !defined(MC02_CONFIG_GENERATED) || !MC02_CHASSIS_IS_OMNI
    if (geometry->model == CHASSIS_MOTION_MODEL_MECANUM)
    {
        ChassisVelocityEstimator_t canonical; // 先按固定轮序生成麦轮几何的估速系数。
        const ChassisKinematicsStatus_t status = MecanumPrepareEstimator(&geometry->mecanum, &canonical);
        if (status != CHASSIS_KINEMATICS_OK) return status; // 几何失败时不交付部分结果。
        for (unsigned axis = 0u; axis < 3u; ++axis) // 分别调整前向、左向和绕上轴的系数。
            for (unsigned i = 0u; i < CHASSIS_WHEEL_COUNT; ++i) // 每列对应一台电机的反馈。
            {
                const unsigned wheel = mecanum_legacy_wheel[i].canonical_index; // 找到这台电机在固定轮序里的位置。
                const float sign = mecanum_legacy_wheel[i].sign * mecanum_rotor_to_output_sign; // 将驱动输出轴方向还原为该轮正向。
                estimator->gain[axis][i] = canonical.gain[axis][wheel] * sign; // 预先换算，周期内将反馈直接乘系数。
            }
        estimator->ready = 1u; // 全部轮位和方向已换算，可按电机槽位读取。
        return CHASSIS_KINEMATICS_OK;
    }
#endif
    return CHASSIS_KINEMATICS_UNSUPPORTED;
}

float ChassisMotionRotorToOutputScale(const ChassisMotionPrepared_s *prepared)
{
    const float sign = prepared->model == CHASSIS_MOTION_MODEL_MECANUM ? mecanum_rotor_to_output_sign : 1.0f;
    return sign * DEGREE_2_RAD / prepared->reduction_ratio; // 把旧接口的电机轴 deg/s 换到驱动输出轴 rad/s。
}

uint8_t ChassisMotionCalculatePrepared(const ChassisMotionInput_s *input,
    const ChassisMotionPrepared_s *prepared, const ChassisMotionLimits_s *limits, ChassisMotionOutput_s *output)
{
    if (output == NULL)
        return 0;
    // 先清零，后面的检查失败时就不会留下上一次的轮速目标。
    memset(output, 0, sizeof(*output));
    if (!input_is_valid(input, prepared, limits))
        return 0;

    float wz_deg_s;
    switch (input->mode)
    {
    case CHASSIS_MOTION_MODE_NO_FOLLOW: // 不跟随云台，保留调用方明确给出的底盘角速度。
        wz_deg_s = input->requested_wz_deg_s;
        if (wz_deg_s > limits->max_wz_deg_s)
            wz_deg_s = limits->max_wz_deg_s;
        else if (wz_deg_s < -limits->max_wz_deg_s)
            wz_deg_s = -limits->max_wz_deg_s;
        break;
    case CHASSIS_MOTION_MODE_FOLLOW: // 按角度差选择跟随方向和转速。
        wz_deg_s = -1.5f * input->offset_angle_deg * fabsf(input->offset_angle_deg);
        // 角度差较大时，转速仍保持在设置的正负上限之间。
        if (wz_deg_s > limits->max_wz_deg_s)
            wz_deg_s = limits->max_wz_deg_s;
        else if (wz_deg_s < -limits->max_wz_deg_s)
            wz_deg_s = -limits->max_wz_deg_s;
        break;
    case CHASSIS_MOTION_MODE_ROTATE: // 使用旋转模式的设定转速。
        wz_deg_s = limits->rotate_wz_deg_s;
        break;
    default: // 未知模式不产生目标。
        return 0;
    }

    const float meters_per_millimeter = 0.001f;
    const ChassisTwist_t source = {
        .vx_m_s = input->vx_mm_s * meters_per_millimeter, // 旧消息的前向 mm/s 换为 m/s。
        .vy_m_s = -input->vy_mm_s * meters_per_millimeter, // 旧消息向右为正，先换为左向再旋转。
        .wz_rad_s = wz_deg_s * DEGREE_2_RAD, // 使用本轮模式选择的底盘转速。
    };
    const float frame_heading_rad = -input->offset_angle_deg * DEGREE_2_RAD; // 旧方向角向右转为正。
    ChassisTwist_t twist; // 麦轮与全向轮都使用同一个底盘本体速度。
    const float legacy_speed_limit_m_s = FLT_MAX * meters_per_millimeter; // 沿用旧接口旋转结果能表达为 mm/s 的范围。
    if (!ChassisMotionRotateToBody(&source, frame_heading_rad, &twist) ||
        fabsf(twist.vx_m_s) > legacy_speed_limit_m_s || fabsf(twist.vy_m_s) > legacy_speed_limit_m_s)
        return 0; // 旧消息数值越界时保留清零输出。

    ChassisWheelSpeeds_t wheels;
    if (ChassisKinematicsCalculatePrepared(&twist, &prepared->projector, &wheels) != CHASSIS_KINEMATICS_OK)
        return 0u; // 无法计算全部轮速时保留清零结果。
#if !defined(MC02_CONFIG_GENERATED) || !MC02_CHASSIS_IS_OMNI
    if (prepared->model == CHASSIS_MOTION_MODEL_MECANUM)
    {
        // 旧应用的电机顺序与新轮序不同；保留旧接口的符号和编号结果。
        const ChassisWheelSpeeds_t canonical = wheels;
        for (unsigned i = 0u; i < CHASSIS_WHEEL_COUNT; ++i) // 依次填充旧接口的各电机槽位。
        {
            const unsigned wheel = mecanum_legacy_wheel[i].canonical_index; // 找到这一槽位关联的运动学轮位。
            wheels.rad_s[i] = mecanum_legacy_wheel[i].sign * canonical.rad_s[wheel]; // 写入旧接口要求的方向。
        }
    }
#endif
    const float wheel_to_motor_deg_s = RAD_2_DEGREE * prepared->reduction_ratio; // 沿用启动校验使用的电机轴转速换算。
    float motor_deg_s[4];
    float largest = 0.0f;
    // 逐轮换算，找出绝对值最大的电机轴转速。
    for (unsigned i = 0; i < 4; ++i)
    {
        motor_deg_s[i] = wheels.rad_s[i] * wheel_to_motor_deg_s;
        if (!isfinite(motor_deg_s[i]))
            return 0;
        const float magnitude = fabsf(motor_deg_s[i]);
        if (magnitude > largest)
            largest = magnitude;
    }

    // 有一轮过快时，四轮共同降速，保留轮速之间的比例和正负方向。
    float scale = largest > limits->max_motor_speed_deg_s
                      ? limits->max_motor_speed_deg_s / largest
                      : 1.0f;
    if (!isfinite(scale) || scale <= 0.0f)
        return 0;

    // 小数计算可能让轮速略超上限，最多计算4次。
    for (unsigned attempt = 0; attempt < 4; ++attempt)
    {
        // 底盘转速也按相同比例减小，与四轮目标保持一致。
        ChassisMotionOutput_s result = {.valid = 1, .wz_deg_s = wz_deg_s * scale};
        if (!isfinite(result.wz_deg_s) || fabsf(result.wz_deg_s) > limits->max_wz_deg_s)
            return 0;
        uint8_t rounding_overshoot = 0;
        // 重新检查四个输出，确认共同降速后没有轮子超出上限。
        for (unsigned i = 0; i < 4; ++i)
        {
            result.motor_speed_deg_s[i] = motor_deg_s[i] * scale;
            if (!isfinite(result.motor_speed_deg_s[i]))
                return 0;
            if (fabsf(result.motor_speed_deg_s[i]) > limits->max_motor_speed_deg_s)
                rounding_overshoot = 1;
        }
        if (!rounding_overshoot)
        {
            // 全部通过后再写输出，失败时不会留下只算了一部分的结果。
            *output = result;
            return 1;
        }
        // 仍略超上限，就把共同倍率调小到 float 能保存的下一个更小数。
        scale = nextafterf(scale, 0.0f);
        if (!isfinite(scale) || scale <= 0.0f)
            return 0;
    }
    // 四次都没通过，保留清零输出。
    return 0;
}
