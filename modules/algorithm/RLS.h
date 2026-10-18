/**
 * @file RLS.h
 * @brief 声明无动态分配的递推最小二乘模块。
 * 调用者先清零 RLS_t，再查询并提供对齐工作区，随后可重置、更新和复制状态。
 */
#ifndef MC02_RLS_H
#define MC02_RLS_H

#include <stddef.h>
#include <stdint.h>

#define RLS_MAX_DIMENSION 255U
#define RLS_WORKSPACE_ALIGNMENT ((size_t)_Alignof(double))

/* 该容量宏适合定义静态字节数组；实际所需大小由查询接口给出。 */
#define RLS_WORKSPACE_CAPACITY(dimension)                                      \
    (sizeof(float) * (3U * (size_t)(dimension) * (size_t)(dimension) +       \
                      5U * (size_t)(dimension)) +                             \
     (RLS_WORKSPACE_ALIGNMENT - 1U) +                                         \
     sizeof(double) * ((size_t)(dimension) * (size_t)(dimension) +            \
                       (size_t)(dimension)))

/** @brief RLS 接口状态；成功为0，错误状态为负数。 */
typedef enum
{
    RLS_STATUS_OK = 0,
    RLS_ERROR_NULL = -1,      /**< 必需指针为空或地址范围无效。 */
    RLS_ERROR_DIMENSION = -2, /**< 维度超出1～255或工作区尺寸溢出。 */
    RLS_ERROR_CONFIG = -3,    /**< delta或lambda不满足配置范围。 */
    RLS_ERROR_CAPACITY = -4,  /**< 元素数或工作区字节数不足。 */
    RLS_ERROR_ALIGNMENT = -5, /**< 工作区或数据指针未按类型对齐。 */
    RLS_ERROR_OVERLAP = -6,   /**< 实例、工作区、输入或输出范围重叠。 */
    RLS_ERROR_STATE = -7,     /**< 实例尚未初始化或内部配置已损坏。 */
    RLS_ERROR_LENGTH = -8,    /**< 初值或输入元素数不等于实例维度。 */
    RLS_ERROR_MATRIX = -9,    /**< CMSIS-DSP 矩阵函数返回失败。 */
    RLS_ERROR_NUMERIC = -10  /**< 输入或中间结果非有限，或协方差非正定。 */
} RLSStatus_t;

/** @brief 一次 RLS 实例的固定维度和遗忘参数。 */
typedef struct
{
    uint16_t dimension; /**< 参数数量，范围为1～255。 */
    float delta;        /**< 初始协方差对角值，必须有限且大于0。 */
    float lambda;       /**< 每次成功更新的遗忘因子，范围为(0, 1]。 */
} RLSConfig_t;

/**
 * @brief 保存一个 RLS 实例的配置、工作区归属和最近一次成功结果。
 * 字段供模块内部使用，调用者只应在首次使用前将整个对象清零；初始化后不要复制实例。
 * 每个实例需要独占一块工作区，同一实例由调用者串行访问。
 */
typedef struct
{
    uint32_t _magic;
    uint16_t _dimension;
    uint16_t _reserved;
    float _delta;
    float _lambda;
    void *_workspace;
    size_t _workspace_bytes;
    size_t _successful_update_count;
    float _last_prediction;
    float _last_residual;
    uint8_t _has_successful_update;
} RLS_t;

/** @brief 最近一次成功更新的诊断信息。 */
typedef struct
{
    size_t successful_update_count; /**< 成功处理的样本数。 */
    float last_prediction;          /**< 更新前的预测值，单位与输出 y 相同。 */
    float last_residual;             /**< 实际输出减去预测值，单位与 y 相同。 */
    uint8_t has_successful_update;   /**< 初始化或重置后为0，成功更新后为1。 */
} RLSStateInfo_t;

/**
 * @brief 查询指定维度所需的工作区字节数和对齐值。
 * @param[in] dimension 参数数量，允许1～255。
 * @param[out] bytes 成功时写入工作区所需字节数；失败时保持原值。
 * @param[out] alignment 成功时写入工作区起始地址的对齐字节数；失败时保持原值。
 * @return RLS_STATUS_OK；空指针、维度错误或输出范围重叠时返回具体错误。
 * @note 同一维度可用 RLS_WORKSPACE_CAPACITY(dimension) 声明静态数组；数组首地址仍须满足返回的对齐要求。
 */
RLSStatus_t RLSGetWorkspaceRequirements(uint16_t dimension, size_t *bytes,
                                        size_t *alignment);

/**
 * @brief 使用调用者工作区初始化实例，并复制恰好 dimension 个初始参数。
 * @param[in,out] rls 实例对象；首次调用前必须清零，成功后由调用者长期保存。
 * @param[in] config 有限配置；dimension为1～255、delta>0、0<lambda<=1。
 * @param[in] initial_params 恰好 config->dimension 个有限 float，作为初始参数。
 * @param[in] initial_count initial_params 的元素数，必须等于配置维度。
 * @param[in,out] workspace 调用者提供的工作区；成功后不得释放、移动或与其他实例共用。
 * @param[in] workspace_bytes 工作区可用字节数，至少等于查询结果。
 * @return 成功返回 RLS_STATUS_OK；失败返回具体状态。
 * @note 所有参数验证完成后才写入实例和工作区；失败时保留原实例及工作区内容。工作区和配置、初值、实例对象不得重叠。
 */
RLSStatus_t RLSInit(RLS_t *rls, const RLSConfig_t *config,
                    const float *initial_params, size_t initial_count,
                    void *workspace, size_t workspace_bytes);

/**
 * @brief 清除协方差、增益和更新历史，并设置新的初始参数。
 * @param[in,out] rls 已成功初始化的实例。
 * @param[in] initial_params 恰好维度个有限 float。
 * @param[in] initial_count 参数元素数，必须等于实例维度。
 * @return RLS_STATUS_OK 或具体错误；失败时已提交的状态保持不变。
 * @note 初值数组不得与实例或工作区重叠。
 */
RLSStatus_t RLSReset(RLS_t *rls, const float *initial_params,
                     size_t initial_count);

/**
 * @brief 用一组输入和实际输出完成一次递推更新。
 * @param[in,out] rls 已成功初始化的实例；同一实例须由调用者串行访问。
 * @param[in] input 恰好维度个有限输入值；合法的全零向量也参与计算。
 * @param[in] input_count 输入元素数，必须等于实例维度。
 * @param[in] actual_output 有限的实测输出值，与模型输出使用相同单位。
 * @return RLS_STATUS_OK；输入、矩阵或数值错误时返回具体状态。
 * @note 输入不得与实例或工作区重叠。失败只会改写工作区临时区域，不改参数、协方差、增益、成功计数或最近一次结果；后续合法输入仍可更新。
 */
RLSStatus_t RLSUpdate(RLS_t *rls, const float *input, size_t input_count,
                      float actual_output);

/**
 * @brief 将参数、协方差、未乘残差的增益和最近一次结果复制到调用者缓冲区。
 * @param[in] rls 已成功初始化的实例。
 * @param[out] params 参数输出，容量至少为维度个 float。
 * @param[in] params_capacity params 的元素容量。
 * @param[out] covariance 协方差输出，按行排列，容量至少为维度平方个 float。
 * @param[in] covariance_capacity covariance 的元素容量。
 * @param[out] gain 最近一次成功更新的 K；尚无成功更新时为全零，容量至少为维度。
 * @param[in] gain_capacity gain 的元素容量。
 * @param[out] info 最近一次成功更新的诊断数据。
 * @return RLS_STATUS_OK 或具体错误；失败时所有输出保持原值。
 * @note 各输出缓冲区彼此独立，且不得与实例或工作区重叠。
 */
RLSStatus_t RLSGetState(const RLS_t *rls, float *params,
                        size_t params_capacity, float *covariance,
                        size_t covariance_capacity, float *gain,
                        size_t gain_capacity, RLSStateInfo_t *info);

#endif /* MC02_RLS_H */
