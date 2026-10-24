/** @file vision_imu_sample.h
 * @brief 定义INS交给视觉链路的原始BMI088加速度快照。
 * @note 数据只在应用内复制，不作为线上结构体；V2帧会单独编码各字段。
 */
#ifndef MC02_VISION_IMU_SAMPLE_H
#define MC02_VISION_IMU_SAMPLE_H

#include <stdint.h>

/** @brief 一份BMI088传感器坐标加速度样本；有效数值单位m/s²且包含重力。
 * @note x/y/z沿BMI088传感器轴，尚未做机器人安装方向修正；无效记录其余字段全为0。
 */
typedef struct
{
    uint8_t valid; /* 只允许0或1；1时序号必须非0。 */
    uint32_t sample_sequence; /* 成功采样序号，回绕时从1继续。 */
    uint32_t sample_time_ms; /* BMI088完整样本准备时的MCU毫秒时刻，0合法。 */
    float acceleration_m_s2[3]; /* 传感器x/y/z轴加速度，包含重力。 */
} VisionIMUSample_t;

/** @brief 发布IMU样本快照的结果；失败时消息中心中的旧快照不会被本次替换。 */
typedef enum
{
    VISION_IMU_SAMPLE_UPDATE_OK = 0, /**< 已复制并发布本次样本。 */
    VISION_IMU_SAMPLE_UPDATE_ERR_ARGUMENT, /**< 输入指针为空。 */
    VISION_IMU_SAMPLE_UPDATE_ERR_VALUE, /**< 有效位、序号、无效字段或有限数值不合法。 */
    VISION_IMU_SAMPLE_UPDATE_ERR_NOT_INITIALIZED, /**< VisionInit尚未登记V2话题。 */
    VISION_IMU_SAMPLE_UPDATE_ERR_UNAVAILABLE, /**< 当前构建使用SP，无法传输此V2数据。 */
    VISION_IMU_SAMPLE_UPDATE_ERR_PUBLISH /**< 消息中心拒绝本次快照。 */
} VisionIMUSampleUpdateStatus_t;

#endif
