/**
 * @file chassis_motion.h
 * @brief 四轮底盘运动计算所用的模式、安装参数、速度范围和输出。
 *
 * ChassisTask 提供本次运动要求，调用 ChassisMotionCalculate 取回四轮目标。
 * 输入、设置和输出要分开存放，计算期间不能改写输入和设置。
 */
#ifndef CHASSIS_MOTION_H
#define CHASSIS_MOTION_H

#include <stdint.h>
#include "omni.h"

/** @brief 选择底盘怎样转动，各有效模式下都计算平移速度。 */
typedef enum
{
    CHASSIS_MOTION_MODE_INVALID = 0, ///< 模式未设置，计算失败。
    CHASSIS_MOTION_MODE_NO_FOLLOW,   ///< 不主动转动。
    CHASSIS_MOTION_MODE_FOLLOW,      ///< 按云台与底盘的角度差计算跟随转速。
    CHASSIS_MOTION_MODE_ROTATE,      ///< 使用设置中的旋转转速。
} ChassisMotionMode_e;

/** @brief 按云台方向给出的平移速度，以及云台与底盘的角度差。 */
typedef struct
{
    float vx_mm_s;             ///< 前进速度，mm/s（毫米/秒）。
    float vy_mm_s;             ///< 横移速度，mm/s。
    float requested_wz_deg_s;  ///< 保留的转速输入，deg/s；当前只检查数字，转速由 mode 决定。
    float offset_angle_deg;   ///< 云台相对底盘归中方向的角度差，-180～180 deg，含两端。
    ChassisMotionMode_e mode;
} ChassisMotionInput_s;

/** @brief 选择底盘几何计算模型。
 * @note 车型构建只能选择与当前固件编译进来的运动学实现相匹配的类型。
 */
typedef enum
{
    CHASSIS_MOTION_MODEL_MECANUM = 0, ///< 使用麦克纳姆轮运动学。
    CHASSIS_MOTION_MODEL_OMNI = 1,    ///< 使用四轮全向轮运动学。
} ChassisMotionModel_e;

/**
 * @brief 底盘安装尺寸，长度单位为 mm；轴距、轮距、轮半径和减速比必须大于0。
 * @note 云台中心的两个方向偏移可为负，沿用底盘安装配置中的 x、y 定义。
 */
typedef struct
{
    float wheel_base_mm;             ///< 前后轮轴之间的距离。
    float track_width_mm;            ///< 左右轮之间的距离。
    float center_gimbal_offset_x_mm; ///< 云台转动中心相对底盘几何中心的 x 方向偏移。
    float center_gimbal_offset_y_mm; ///< 云台转动中心相对底盘几何中心的 y 方向偏移。
    float wheel_radius_mm;           ///< 轮子半径。
    float reduction_ratio;           ///< 电机轴转速与轮子转速之比。
    ChassisMotionModel_e model;      ///< 当前底盘采用的计算模型。
    OmniGeometry_t omni;             ///< 全向轮模型使用，长度单位为米。
} ChassisMotionGeometry_s;

/** @brief 底盘和电机轴的转速设置，单位均为 deg/s（度/秒）。 */
typedef struct
{
    uint8_t configured;             ///< 非0表示设置已准备好；为0时计算失败。
    float max_wz_deg_s;             ///< 底盘转速的最大绝对值，必须大于0。
    float rotate_wz_deg_s;          ///< 旋转模式转速，可正可负，绝对值不超过 max_wz_deg_s。
    float max_motor_speed_deg_s;    ///< 电机轴转速的最大绝对值，必须大于0。
} ChassisMotionLimits_s;

/** @brief 本次计算得到的底盘转速和四轮电机轴目标速度。 */
typedef struct
{
    uint8_t valid;    ///< 1=计算成功；0=结果不能使用。
    float wz_deg_s;   ///< 底盘转速，deg/s；四轮共同降速时也随之减小。
    /* 左前、右前、左后、右后，单位为电机轴 deg/s，已计入减速比。 */
    float motor_speed_deg_s[4];
} ChassisMotionOutput_s;

/**
 * @brief 将平移和转动要求换算为四轮电机轴目标速度，并检查允许的速度范围。
 * @param[in] input 本次运动要求；NULL 时失败。
 * @param[in] geometry 安装尺寸和减速比；NULL 时失败。
 * @param[in] limits 转速设置；NULL、未准备好或数值不合要求时失败。
 * @param[out] output 计算结果；NULL 时失败。
 * @return 1=成功，output->valid 为1；0=失败，非 NULL 的 output 各项清零。
 * @note 所有浮点参数都不能是 NaN 或无穷大，计算中出现这类结果也会失败。
 *       所有模式都检查全部参数，未用到的字段也要填合法值。
 *       有轮子超过转速上限时，四轮和输出 wz_deg_s 按同一比例减小。
 *       若仍无法把结果保持在允许范围内，返回失败。
 */
uint8_t ChassisMotionCalculate(const ChassisMotionInput_s *input,
                               const ChassisMotionGeometry_s *geometry,
                               const ChassisMotionLimits_s *limits,
                               ChassisMotionOutput_s *output);

#endif
