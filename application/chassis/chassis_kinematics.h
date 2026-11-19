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
    float x_m, y_m; /**< 从本段起点累计的前向、左向位移，m。 */
    float yaw_rad; /**< 相对本段起始朝向的角度，左转为正，rad。 */
} ChassisLocalPose_t;

/** @brief 检查可选轮速里程计的间隔配置。
 * @param[in] max_gap_ms 允许积分的最大样本间隔，ms；0 关闭。
 * @param[in] has_wheel_velocity 轮组可提供底盘中心速度估计时传入 1。
 * @return 1 表示功能关闭，或开启且间隔可用 32 位时间区分先后；其他返回 0。
 */
static inline uint8_t ChassisOdometryGapIsValid(uint32_t max_gap_ms, uint8_t has_wheel_velocity)
{
    return max_gap_ms == 0u || (has_wheel_velocity == 1u && max_gap_ms <= INT32_MAX);
}
typedef struct {
    float rad_s[CHASSIS_WHEEL_COUNT]; /**< 轮轴速度，不含电机减速比及安装反向。 */
} ChassisWheelSpeeds_t;
typedef struct {
    float x_m, y_m; /**< 轮中心相对底盘中心的位置。 */
    float drive_x, drive_y; /**< 滚动方向投影；麦轮还计入滚子带来的投影比例。 */
    float radius_m;
} ChassisWheelGeometry_t;

/** @brief 轮组几何固定后即可提前算好的轮速到车体速度系数。 */
typedef struct {
    double gain[3][CHASSIS_WHEEL_COUNT]; /**< 每行分别输出前进、左移或转动速度。 */
    uint8_t ready; /**< 准备成功置 1，失败或清零后不能用于估速。 */
} ChassisVelocityEstimator_t;
/** @brief 将底盘中心速度投影到四轮，拒绝不能独立控制三个平面自由度的轮组。
 * @param[in] input 速度要求；各数值须有限。
 * @param[in] wheels 四个轮位、投影比例和正轮径；仅计算期间读取。
 * @param[out] output 轮轴rad/s；任何失败时，非NULL输出清零。
 * @return OK或ARGUMENT；参数均不可为NULL。
 */
ChassisKinematicsStatus_t ChassisKinematicsCalculate(const ChassisTwist_t *input,
    const ChassisWheelGeometry_t wheels[CHASSIS_WHEEL_COUNT], ChassisWheelSpeeds_t *output);
/** @brief 用四轮的实测输出轴速度估计三个底盘速度分量。
 * @param[in] input 四轮统一编号的输出轴速度，rad/s。
 * @param[in] wheels 四轮位置、滚动投影和正轮径，和目标轮速计算使用同一套几何。
 * @param[out] output 底盘中心速度，m/s、rad/s，失败时非空输出清零。
 * @return OK 或 ARGUMENT；不使用轮速以外的滑移或场地定位信息。
 */
ChassisKinematicsStatus_t ChassisKinematicsEstimate(const ChassisWheelSpeeds_t *input,
    const ChassisWheelGeometry_t wheels[CHASSIS_WHEEL_COUNT], ChassisTwist_t *output);

/** @brief 检查轮组几何并提前计算估速系数。
 * @param[in] wheels 四轮位置、滚动投影和正轮径。
 * @param[out] estimator 要准备的系数，非空且须与 wheels 分开存放；失败时标为不可用。
 * @return OK 或 ARGUMENT；更换几何后必须重新准备。
 */
ChassisKinematicsStatus_t ChassisKinematicsPrepareEstimator(
    const ChassisWheelGeometry_t wheels[CHASSIS_WHEEL_COUNT], ChassisVelocityEstimator_t *estimator);

/** @brief 用已准备的系数把四轮速度换算为底盘速度。
 * @param[in] input 固定轮序的实测轮轴 rad/s，须为有限数。
 * @param[in] estimator 按当前几何准备成功的系数。
 * @param[out] output 底盘中心速度，m/s、rad/s；失败时非空输出清零。
 * @return OK 或 ARGUMENT；所有指针必须非空。
 */
ChassisKinematicsStatus_t ChassisKinematicsEstimatePrepared(const ChassisWheelSpeeds_t *input,
    const ChassisVelocityEstimator_t *estimator, ChassisTwist_t *output);
/** @brief 将一段底盘速度积分到本地位姿，假定区间内为两端速度的平均值。
 * @param[in,out] pose 起始位姿与积分结果；失败时保持原值。
 * @param[in] previous 较早样本的底盘速度，m/s、rad/s。
 * @param[in] current 较新样本的底盘速度，m/s、rad/s。
 * @param[in] dt_s 两个不同样本时刻的间隔，单位 s，必须大于零。
 * @return OK 或 ARGUMENT；所有指针必须非空，输入及结果必须为有限数。
 */
ChassisKinematicsStatus_t ChassisKinematicsIntegrate(ChassisLocalPose_t *pose,
    const ChassisTwist_t *previous, const ChassisTwist_t *current, float dt_s);
#endif
