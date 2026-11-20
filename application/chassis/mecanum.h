/** @file mecanum.h
 * @brief 四轮45度滚子麦轮计算；参数使用米，前左上坐标。
 */
#ifndef MC02_MECANUM_H
#define MC02_MECANUM_H
#include "chassis_kinematics.h"
typedef struct {
    float half_base_m, half_track_m;
    float center_x_m, center_y_m; /**< 计算中心相对轮组几何中心的偏移。 */
    float radius_m;
} MecanumGeometry_t;
/** @brief 计算固定轮序的麦轮轮轴速度。
 * @param[in] input 底盘中心速度，m/s、rad/s，不可为NULL。
 * @param[in] geometry 正半轴距、正半轮距、有限偏移及正轮径，不可为NULL。
 * @param[out] output 轮轴rad/s；失败时非NULL输出清零。
 * @return OK或ARGUMENT；不改变输入，不操作硬件。
 */
ChassisKinematicsStatus_t MecanumCalculate(const ChassisTwist_t *input,
    const MecanumGeometry_t *geometry, ChassisWheelSpeeds_t *output);
/** @brief 把麦轮尺寸转成固定轮序的目标轮速投影。
 * @param[in] geometry 正半轴距、正半轮距、有限偏移及正轮径。
 * @param[out] projector 保存到几何改变或底盘停止；失败时标为不可用。
 * @return OK 或 ARGUMENT。
 */
ChassisKinematicsStatus_t MecanumPrepareProjector(const MecanumGeometry_t *geometry, ChassisWheelProjector_t *projector);
/** @brief 根据麦轮几何准备从四轮轮速估算底盘速度的系数。
 * @param[in] geometry 与目标计算相同的几何，尺寸为米。
 * @param[out] estimator 接受左前、右前、左后、右后的轮轴 rad/s；失败时标为不可用。
 * @return OK 或 ARGUMENT。
 */
ChassisKinematicsStatus_t MecanumPrepareEstimator(const MecanumGeometry_t *geometry, ChassisVelocityEstimator_t *estimator);
#endif
