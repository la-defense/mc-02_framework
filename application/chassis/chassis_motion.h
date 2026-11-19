/**
 * @file chassis_motion.h
 * @brief 四轮底盘运动计算所用的模式、安装参数、速度范围和输出。
 *
 * ChassisTask 提供本次运动要求，使用启动时准备好的几何取回四轮目标。
 * 输入、设置和输出要分开存放，计算期间不能改写输入和设置。
 */
#ifndef CHASSIS_MOTION_H
#define CHASSIS_MOTION_H

#include <stdint.h>
#include "mecanum.h"
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
    float vy_mm_s;             ///< 旧消息的横移速度，向右为正，mm/s。
    float requested_wz_deg_s;  ///< 不跟随模式请求的底盘角速度，deg/s。
    float offset_angle_deg;    ///< 旧消息的方向角，向右转为正，-180～180 deg，含两端。
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
 * @brief 底盘模型和公共减速比。只读取选中的轮组几何，尺寸采用前左上坐标下的 m。
 */
typedef struct
{
    float reduction_ratio;           ///< 电机轴转速与轮子转速之比。
    ChassisMotionModel_e model;      ///< 当前底盘采用的计算模型。
    MecanumGeometry_t mecanum;        ///< 麦轮几何，仅麦轮使用，长度单位为米。
    OmniGeometry_t omni;             ///< 全向轮模型使用，长度单位为米。
} ChassisMotionGeometry_s;

/** @brief 由只读车型配置准备的轮速投影、模型和减速比。 */
typedef struct
{
    ChassisWheelProjector_t projector; ///< 四轮输出轴投影，准备失败时不可用。
    ChassisMotionModel_e model;        ///< 选择对应的轮序和符号约定。
    float reduction_ratio;             ///< 电机轴转速与轮轴转速之比。
} ChassisMotionPrepared_s;

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

/** @brief 将一个平移参照下的底盘中心速度转到底盘本体坐标。
 * @param[in] source 转换前速度，前进及左移为 m/s，绕上轴为 rad/s。
 * @param[in] frame_heading_rad 参照正前方相对底盘正前方的角，左转为正，范围 [-π, π]。
 * @param[out] body 成功时写入本体坐标速度，角速度保持原值；允许与 source 指向同一对象。
 * @return 1 成功；空指针、非有限值或角度越界返回 0，保持原输出。
 */
uint8_t ChassisMotionRotateToBody(const ChassisTwist_t *source, float frame_heading_rad,
                                   ChassisTwist_t *body);

/**
 * @brief 将平移和转动要求换算为四轮电机轴目标速度，并检查允许的速度范围。
 * @param[in] input 本次运动要求；NULL 时失败。
 * @param[in] geometry 安装尺寸和减速比；NULL 时失败。
 * @param[in] limits 转速设置；NULL、未准备好或数值不合要求时失败。
 * @param[out] output 计算结果；NULL 时失败。
 * @return 1=成功，output->valid 为1；0=失败，非 NULL 的 output 各项清零。
 * @note 所有浮点参数都不能是 NaN 或无穷大，计算中出现这类结果也会失败。
 *       运动输入和转速配置必须有效，轮组几何按所选模型检查。
 *       有轮子超过转速上限时，四轮和输出 wz_deg_s 按同一比例减小。
 *       若仍无法把结果保持在允许范围内，返回失败。
 */
uint8_t ChassisMotionCalculate(const ChassisMotionInput_s *input,
                               const ChassisMotionGeometry_s *geometry,
                               const ChassisMotionLimits_s *limits,
                               ChassisMotionOutput_s *output);

/** @brief 启动或更换车型几何时检查参数并准备轮速投影。
 * @param[in] geometry 按当前固件编译选项提供的安装尺寸和减速比。
 * @param[out] prepared 调用者提供的只读运行配置；失败时投影标为不可用。
 * @return 1 成功，0 为指针、几何或选择的模型无效。
 */
uint8_t ChassisMotionPrepare(const ChassisMotionGeometry_s *geometry, ChassisMotionPrepared_s *prepared);

/** @brief 使用启动时的投影计算目标，保留原有模式、单位与限速检查。
 * @param[in] input 本次运动要求。
 * @param[in] prepared 与当前车辆几何一致的运行配置。
 * @param[in] limits 车型的转速限制。
 * @param[out] output 计算结果；失败时非空输出清零。
 * @return 1 表示全部四轮目标有效，0 表示参数或计算结果无效。
 */
uint8_t ChassisMotionCalculatePrepared(const ChassisMotionInput_s *input,
    const ChassisMotionPrepared_s *prepared, const ChassisMotionLimits_s *limits, ChassisMotionOutput_s *output);

#endif
