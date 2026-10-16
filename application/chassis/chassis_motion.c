/**
 * @file chassis_motion.c
 * @brief 按模式计算底盘转速，再将平移和转动换算为四轮电机轴速度。
 *
 * 供 ChassisTask 调用，只计算目标，不直接发送电机指令。
 * 当前沿用工程原有的四轮计算方式，参数和返回值见 chassis_motion.h。
 */
#include "chassis_motion.h"

#include "arm_math.h"

#include <math.h>
#include <string.h>

/* 数字大于0且不是 NaN 或无穷大才通过。 */
static uint8_t finite_positive(float value)
{
    return isfinite(value) && value > 0.0f;
}

/* 核对尺寸、速度范围和输入，避免除零或按未知模式计算。 */
static uint8_t input_is_valid(const ChassisMotionInput_s *input,
                              const ChassisMotionGeometry_s *geometry,
                              const ChassisMotionLimits_s *limits)
{
    return input != NULL && geometry != NULL && limits != NULL &&
           limits->configured &&
           finite_positive(geometry->wheel_base_mm) &&
           finite_positive(geometry->track_width_mm) &&
           isfinite(geometry->center_gimbal_offset_x_mm) &&
           isfinite(geometry->center_gimbal_offset_y_mm) &&
           finite_positive(geometry->wheel_radius_mm) &&
           finite_positive(geometry->reduction_ratio) &&
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

uint8_t ChassisMotionCalculate(const ChassisMotionInput_s *input,
                               const ChassisMotionGeometry_s *geometry,
                               const ChassisMotionLimits_s *limits,
                               ChassisMotionOutput_s *output)
{
    if (output == NULL)
        return 0;
    // 先清零，后面的检查失败时就不会留下上一次的轮速目标。
    memset(output, 0, sizeof(*output));
    if (!input_is_valid(input, geometry, limits))
        return 0;

    const float radians_per_degree = 0.01745329252f;
    const float degrees_per_radian = 57.2957795f;
    float wz_deg_s;
    switch (input->mode)
    {
    case CHASSIS_MOTION_MODE_NO_FOLLOW: // 只计算平移，不主动转动。
        wz_deg_s = 0.0f;
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

    // 按云台与底盘的角度差，把平移速度转到底盘方向。
    const float angle_rad = input->offset_angle_deg * radians_per_degree;
    const float cos_theta = arm_cos_f32(angle_rad);
    const float sin_theta = arm_sin_f32(angle_rad);
    const float vx_mm_s = input->vx_mm_s * cos_theta - input->vy_mm_s * sin_theta;
    const float vy_mm_s = input->vx_mm_s * sin_theta + input->vy_mm_s * cos_theta;
    // 输入数字正常，计算结果仍可能过大；此时保留清零输出。
    if (!isfinite(wz_deg_s) || !isfinite(cos_theta) || !isfinite(sin_theta) ||
        !isfinite(vx_mm_s) || !isfinite(vy_mm_s))
        return 0;

    // 用安装尺寸和云台中心偏移，计算底盘转动给各轮带来的速度。
    const float half_base = geometry->wheel_base_mm * 0.5f;
    const float half_track = geometry->track_width_mm * 0.5f;
    const float lf_center = (half_track + geometry->center_gimbal_offset_x_mm +
                             half_base - geometry->center_gimbal_offset_y_mm) * radians_per_degree;
    const float rf_center = (half_track - geometry->center_gimbal_offset_x_mm +
                             half_base - geometry->center_gimbal_offset_y_mm) * radians_per_degree;
    const float lb_center = (half_track + geometry->center_gimbal_offset_x_mm +
                             half_base + geometry->center_gimbal_offset_y_mm) * radians_per_degree;
    const float rb_center = (half_track - geometry->center_gimbal_offset_x_mm +
                             half_base + geometry->center_gimbal_offset_y_mm) * radians_per_degree;
    // 叠加平移与转动带来的轮速，正负号沿用工程原有计算方式。
    const float wheel_mm_s[4] = {
        -vx_mm_s - vy_mm_s - wz_deg_s * lf_center,
        -vx_mm_s + vy_mm_s - wz_deg_s * rf_center,
        vx_mm_s - vy_mm_s - wz_deg_s * lb_center,
        vx_mm_s + vy_mm_s - wz_deg_s * rb_center,
    };
    // 轮边线速度换成轮子角速度，再乘减速比得到电机轴转速。
    const float mm_s_to_motor_deg_s = degrees_per_radian * geometry->reduction_ratio /
                                      geometry->wheel_radius_mm;
    if (!isfinite(lf_center) || !isfinite(rf_center) ||
        !isfinite(lb_center) || !isfinite(rb_center) ||
        !isfinite(mm_s_to_motor_deg_s) || mm_s_to_motor_deg_s <= 0.0f)
        return 0;

    float motor_deg_s[4];
    float largest = 0.0f;
    // 逐轮换算，找出绝对值最大的电机轴转速。
    for (unsigned i = 0; i < 4; ++i)
    {
        if (!isfinite(wheel_mm_s[i]))
            return 0;
        motor_deg_s[i] = wheel_mm_s[i] * mm_s_to_motor_deg_s;
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
