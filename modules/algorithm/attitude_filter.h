/**
 * @file attitude_filter.h
 * @brief 用统一结果和调用者工作区切换 QuaternionEKF、Fusion 姿态算法。
 * @author MC-02 contributors
 */
#ifndef MC02_ATTITUDE_FILTER_H
#define MC02_ATTITUDE_FILTER_H

#include "kalman_filter.h"
#include "QuaternionEKF.h"
#include <stddef.h>
#include <stdint.h>

#define ATTITUDE_FILTER_STORAGE_BYTES 4096u
#define ATTITUDE_FILTER_STORAGE_ALIGNMENT _Alignof(max_align_t)

typedef enum {
    ATTITUDE_BACKEND_QUATERNION_EKF = 0,
    ATTITUDE_BACKEND_FUSION = 1
} AttitudeBackend_t;

/**
 * @brief Fusion参数。调用者提供BMI088使用的物理单位，适配层转换后再调用上游算法。
 */
typedef struct {
    float sample_rate_hz; /**< 标称采样率，Hz，范围(0,10000]。 */
    float gain; /**< 算法增益，范围[0,10]；姿态表现需在目标平台另行验证。 */
    float gyroscope_range_deg_s; /**< 陀螺仪量程，deg/s；0关闭超量程检测。 */
    float acceleration_rejection_deg; /**< 加速度方向拒绝角，度，范围[0,180)；0关闭拒绝。 */
    float rejection_timeout_s; /**< 上游恢复计数超时，秒；拒绝角非零时必须大于0。 */
    float accel_min_m_s2; /**< 静止加速度模长下限，m/s²，边界允许。 */
    float accel_max_m_s2; /**< 静止加速度模长上限，m/s²，须大于下限。 */
} AttitudeFusionConfig_t;

/**
 * @brief QuaternionEKF算法专用参数；全部有限，过程噪声非负、量测噪声为正、lambda在(0,1]、LPF非负、方差倍率不小于1。
 */
typedef struct {
    float process_noise1;
    float process_noise2;
    float measure_noise;
    float lambda;
    float lpf;
    float variance_ratio;
} AttitudeQekfConfig_t;

/** @brief 两种后端共同使用的时限及算法参数；只读取当前选择的后端参数。 */
typedef struct {
    AttitudeBackend_t backend; /**< 编译时已包含的后端；Fusion固件由MC02_IMU_BACKEND选择。 */
    uint32_t max_prediction_ms; /**< 校正后允许发布预测的最长年龄，毫秒，1～INT32_MAX，边界允许。 */
    uint32_t max_measurement_age_ms; /**< 加速度源样本的最大年龄，毫秒，1～INT32_MAX。 */
    float max_dt_s; /**< 实测采样间隔上限，秒，有限正数，等于上限时允许。 */
    AttitudeQekfConfig_t qekf; /**< 选择QuaternionEKF时使用的参数。 */
    AttitudeFusionConfig_t fusion; /**< 选择Fusion时使用的参数。 */
} AttitudeFilterConfig_t;

/** @brief 每份完整BMI088样本只传入一次；加速度不存在时只预测，不刷新校正源时间。 */
typedef struct {
    float gyroscope_rad_s[3]; /**< 机体系x/y/z角速度，rad/s，三项均须有限。 */
    float accelerometer_m_s2[3]; /**< 机体系x/y/z加速度，m/s²；缺样时内容不使用。 */
    float dt_s; /**< 调用方实测两次样本间隔，秒，须在配置上下限内。 */
    uint32_t now_ms; /**< 当前毫秒时刻，与源时刻同一32位时钟；允许自然回绕。 */
    uint32_t sample_time_ms; /**< BMI088样本原始接收时刻；不得由预测时刻替代。 */
    uint32_t sequence; /**< BMI088递增序号，回绕按32位半区间比较。 */
    uint8_t accelerometer_valid; /**< 非零表示加速度字段可参与质量检查；零表示只使用陀螺预测。 */
} AttitudeFilterInput_t;

/** @brief 输出姿态和估计有效性；无效时四元数及角度保留最后一次有限值供诊断。 */
typedef struct {
    KFResult_t filter; /**< 模式、错误原因、最近校正源时刻、估计时刻及预测年龄。 */
    float quaternion[4]; /**< 当前机体到NWU导航系的w/x/y/z四元数。 */
    float roll_deg; /**< 横滚角，度。 */
    float pitch_deg; /**< 俯仰角，度。 */
    float yaw_deg; /**< 偏航角，度。 */
    float yaw_total_deg; /**< 以首个有效输出为基准展开的多圈偏航角，度。 */
    uint32_t reference_version; /**< 每次成功初始化递增并跳过0；换后端后须重新对齐下游参考。 */
} AttitudeFilterState_t;

/** @brief 一份算法实例；内容由接口维护，调用者只负责长期保存和串行访问。 */
typedef struct {
    uint32_t initialized;
    uint32_t reference_version;
    AttitudeBackend_t backend;
    AttitudeFilterConfig_t config;
    void *workspace;
    size_t workspace_bytes;
    uint32_t last_seen_sequence;
    uint32_t last_seen_time_ms;
    uint8_t has_seen_sample;
    uint8_t has_corrected_measurement;
    uint8_t has_published_orientation;
    uint8_t fatal;
    float last_yaw_deg;
    AttitudeFilterState_t state;
} AttitudeFilter_t;

/**
 * @brief 查询后端所需的工作区，不分配内存。
 * @param[in] backend 后端类型。
 * @param[out] bytes 返回精确字节数；失败保持原值。
 * @param[out] alignment 返回对齐字节数；失败保持原值。
 * @return KF_OK、KF_ERROR_ARGUMENT、KF_ERROR_CONFIG或KF_ERROR_WORKSPACE。出错时两个输出不变。
 * @note 固件只编译所选后端；查询未编译的Fusion后端会返回KF_ERROR_CONFIG。
 */
KFStatus_t AttitudeFilterGetWorkspaceRequirements(AttitudeBackend_t backend,
    size_t *bytes,size_t *alignment);

/**
 * @brief 初始化指定姿态后端，成功后参考版本递增并等待首份有效加速度校正。
 * @param[in,out] filter 调用者长期保存的实例；首次使用前须清零，不得并发访问。
 * @param[in] config 后端、预测期限、量测期限、最大dt及算法参数；成功后复制并固定。
 * @param[in] initial_quaternion 有限非零四元数，顺序w/x/y/z，表示当前机体到NWU导航系。
 * @param[in,out] workspace 按容量查询结果提供的独占可写存储，需满足查询对齐。
 * @param[in] workspace_bytes 可写容量；至少为查询值且不超过ATTITUDE_FILTER_STORAGE_BYTES。
 * @return KF_OK或具体参数／配置／工作区／数值错误；失败不改写实例或工作区。
 * @note 不分配内存。切换算法须重新初始化；每次成功初始化都会递增参考版本。
 */
KFStatus_t AttitudeFilterInit(AttitudeFilter_t *filter,const AttitudeFilterConfig_t *config,
    const float initial_quaternion[4],void *workspace,size_t workspace_bytes);

/**
 * @brief 使用一份新样本更新后端并复制当前状态。
 * @param[in,out] filter 已初始化实例；不同实例可并行，同一实例由调用者串行访问。
 * @param[in] input 实测dt秒、本机毫秒时刻、源时间／序号及rad/s、m/s²输入；样本序号和时间须新于上次接受样本。
 * @param[out] state 接收本轮状态；无效时保留最后数值并清除filter.valid。
 * @return KF_OK为已校正、KF_PREDICTED为有限预测，其他状态表示本轮无效或参数错误。
 * @note 预测不更新最后校正源时刻；超过预测期限后有效标记撤销，新的合格量测可恢复。内部数值计算失败后须重新初始化；单份非法输入不会写入后端历史。
 */
KFStatus_t AttitudeFilterUpdate(AttitudeFilter_t *filter,const AttitudeFilterInput_t *input,
    AttitudeFilterState_t *state);

/**
 * @brief 复制当前结果，不推进滤波或延长源时限。
 * @param[in] filter 已初始化实例。
 * @param[out] state 接收结果；失败保持原值。
 * @return KF_OK或具体参数／状态错误。
 */
KFStatus_t AttitudeFilterGetState(const AttitudeFilter_t *filter,AttitudeFilterState_t *state);

#endif
