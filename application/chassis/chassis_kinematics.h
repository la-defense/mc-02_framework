/** @file chassis_kinematics.h
 * @brief 纯数值底盘接口；前左上坐标，固定轮序为左前、右前、左后、右后。
 */
#ifndef MC02_CHASSIS_KINEMATICS_H
#define MC02_CHASSIS_KINEMATICS_H
#include <stdint.h>
#define CHASSIS_WHEEL_COUNT 4u
typedef enum {
    CHASSIS_KINEMATICS_OK = 0,
    CHASSIS_KINEMATICS_ARGUMENT,
    CHASSIS_KINEMATICS_UNSUPPORTED
} ChassisKinematicsStatus_t;
typedef struct {
    float vx_m_s, vy_m_s, wz_rad_s;
} ChassisTwist_t;
typedef struct {
    float rad_s[CHASSIS_WHEEL_COUNT]; /**< 轮轴速度，不含电机减速比及安装反向。 */
} ChassisWheelSpeeds_t;
typedef struct {
    float x_m, y_m; /**< 轮中心相对底盘中心的位置。 */
    float drive_x, drive_y; /**< 滚动方向投影；麦轮还计入滚子带来的投影比例。 */
    float radius_m;
} ChassisWheelGeometry_t;
/** @brief 将底盘中心速度投影到四轮，拒绝不能独立控制三个平面自由度的轮组。
 * @param[in] input 速度要求；各数值须有限。
 * @param[in] wheels 四个轮位、投影比例和正轮径；仅计算期间读取。
 * @param[out] output 轮轴rad/s；任何失败时，非NULL输出清零。
 * @return OK或ARGUMENT；参数均不可为NULL。
 */
ChassisKinematicsStatus_t ChassisKinematicsCalculate(const ChassisTwist_t *input,
    const ChassisWheelGeometry_t wheels[CHASSIS_WHEEL_COUNT], ChassisWheelSpeeds_t *output);
#endif
