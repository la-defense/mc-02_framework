/**
 * @file sample_filter.h
 * @brief 独立实例的四类采样滤波及旧整窗平均入口；无硬件时钟和动态分配。
 * 新接口先清零实例、初始化配置，再更新；历史存储随实例长期保存。
 * @author Wang Hongxi
 * @author modified by neozng
 */
#ifndef MC02_SAMPLE_FILTER_H
#define MC02_SAMPLE_FILTER_H
#include "math_utils.h"
#include <stddef.h>
#include <stdint.h>

/** @brief 丢弃最早一项、追加新样本并交付整窗平均值。
 * @param[in] new_data 有限新样本，单位与历史一致。
 * @param[in,out] buffer 非空、按float对齐的历史数组，至少capacity个元素，生命周期覆盖调用。
 * @param[in] capacity 缓冲元素容量，须不小于length。
 * @param[in] length 实际窗口元素数，1～255；只读写buffer的前length项。
 * @param[out] output 非空且按float对齐，与所用历史不得重叠；成功结果沿用样本单位。
 * @return MATH_OK、MATH_BAD_ARGUMENT、MATH_BAD_RANGE 或 MATH_NONFINITE。
 * @note 历史的每一项均须有限；零填充启动仍除以整个窗口长度。
 * 失败保留历史和输出；不同缓冲可交错／并发，同一缓冲由调用者串行访问。
 */
MathStatus_t MovingAverageF32(float new_data, float *buffer, size_t capacity,
                            size_t length, float *output);

/** @brief 固定滤波种类；改变种类或参数须重新初始化。 */
typedef enum
{
    SAMPLE_FILTER_LOWPASS1 = 1,
    SAMPLE_FILTER_AVERAGE,
    SAMPLE_FILTER_LOWPASS2,
    SAMPLE_FILTER_NOTCH
} SampleFilterType_t;

/** @brief 更新结果；只有OK和REBUILT交付本次输出。 */
typedef enum
{
    FILTER_OK = 0,
    FILTER_REBUILT = 1,      /**< 当前样本建立常值历史，输出等于当前输入。 */
    FILTER_NO_SAMPLE = 2,    /**< 没有样本，本轮无有效输出，等待新样本重建。 */
    FILTER_WAITING = 3,      /**< 初始化／重置后尚未接受样本，仅由状态查询读到。 */
    FILTER_ERROR_ARGUMENT = -1,
    FILTER_ERROR_CONFIG = -2,
    FILTER_NOT_INITIALIZED = -3,
    FILTER_ERROR_DT = -4,
    FILTER_ERROR_NONFINITE = -5,
    FILTER_ERROR_NUMERIC = -6
} SampleFilterStatus_t;

/** @brief 固定配置，未使用的参数必须置0；全部浮点参数须有限。 */
typedef struct
{
    SampleFilterType_t type;
    float dt_min_s;        /**< 运行中实测间隔下限，秒，必须>0。 */
    float dt_max_s;        /**< 上限，秒，须>=下限，两个边界均允许。 */
    float tau_s;           /**< 一阶低通时间常数，秒，必须>0；其他种类为0。 */
    float frequency_hz;    /**< 二阶低通截止／陷波中心频率，Hz，须>0且乘dt_max_s<0.5。 */
    float quality_factor;  /**< 陷波Q，必须>0；其他种类为0，二阶低通固定Butterworth。 */
    size_t window_length;  /**< 平均窗口1～255，按样本数计；其他种类为0。 */
} SampleFilterConfig_t;

/** @brief 只读诊断副本；查询不产生新样本，也不证明来源仍未过期。 */
typedef struct
{
    SampleFilterType_t type;
    SampleFilterStatus_t last_status;
    float last_output;       /**< 最后一次成功数值；初始化／重置后为0。 */
    uint8_t valid;            /**< 最近一次有样本的计算成功，缺样／数值或dt失败时撤销。 */
    uint8_t needs_rebuild;    /**< 下一份有限样本会重新建立历史，忽略该次dt。 */
    uint64_t accepted_count;  /**< 成功样本数，达到UINT64_MAX后保持。 */
    uint64_t rebuild_count;   /**< 含首次采样的重建次数，达到UINT64_MAX后保持。 */
} SampleFilterState_t;

/** @brief 调用者保存的实例；以下带下划线字段仅供实现使用，不得直接改写或复制实例。
 * @note 地址在使用期间固定；不同实例须独占各自历史，同一实例访问须串行。
 */
typedef struct
{
    uint32_t _initialized;
    uintptr_t _owner;
    SampleFilterConfig_t _config;
    float *_history;
    size_t _capacity;
    double _state1, _state2;
    SampleFilterState_t _result;
} SampleFilter_t;

/** @brief 接受完整配置并清除历史，无动态分配。
 * @param[out] filter 非空、按SampleFilter_t对齐的长期实例，首次使用前清零；失败保持原对象。
 * @param[in] config 非空且按类型对齐，与实例和历史不重叠；内容被复制，返回后可以释放。
 * @param[in,out] history 平均种类的独占float数组，按float对齐且长期保存；其他种类必须NULL。
 * @param[in] capacity history实际元素容量，平均种类须>=window_length；其他种类必须0。
 * @return FILTER_OK或参数／配置错误。失败保持实例及历史；成功只清零窗口内的历史。
 * @note 实例、配置和所提供历史不能重叠。低通／陷波在dt两端生成的double系数须有限且严格稳定；
 * 极端参数无法满足该检查时拒绝配置。初始化不交付有效输出，首份有限样本才建立历史。
 * 原history在成功重新初始化并改用其他存储后可以释放；失败时仍须保留原存储。
 */
SampleFilterStatus_t SampleFilterInit(SampleFilter_t *filter, const SampleFilterConfig_t *config,
                                    float *history, size_t capacity);

/** @brief 清除历史、数值及计数，保留配置和历史归属，重新等待样本。
 * @param[in,out] filter 已初始化且地址固定的实例，非空；失败不修改。
 * @return FILTER_OK、FILTER_ERROR_ARGUMENT或FILTER_NOT_INITIALIZED。
 */
SampleFilterStatus_t SampleFilterReset(SampleFilter_t *filter);

/** @brief 处理一次采样；缺样或计算失败后由下一份有限样本自动重建。
 * @param[in,out] filter 已初始化实例，非空，运行期间不得改写其配置或历史。
 * @param[in] sample_present 0表示缺样并忽略input和dt_s，1表示有新鲜样本，其他值拒绝。
 * @param[in] input 有限新值，单位由调用者确定；只有sample_present为1时读取。
 * @param[in] dt_s 运行中相邻源样本的实测秒间隔，须有限且在配置闭区间；首次／重建时忽略。
 * @param[out] output 非空、按float对齐且与实例及整个history存储不重叠；失败保持原值。
 * @return FILTER_OK或FILTER_REBUILT表示有新输出；缺样返回FILTER_NO_SAMPLE，其他返回具体错误。
 * @note 缺样、非有限输入、非法dt和数值失败只改有效标记、重建标记及原因，保留数值历史和计数；
 * 下一份有限输入填充平均常值窗口或建立低通／陷波常值稳态，返回REBUILT且不混入旧数据。
 * 参数／未初始化错误保持整份实例。算法不保存或更新源时间，使用前仍须由调用方检查原来源有效性。
 */
SampleFilterStatus_t SampleFilterUpdate(SampleFilter_t *filter, uint8_t sample_present,
                                      float input, float dt_s, float *output);

/** @brief 复制诊断数据，不推进状态或更新来源时间。
 * @param[in] filter 非空已初始化实例，复制期间不能更新或重置。
 * @param[out] state 非空、按SampleFilterState_t对齐，与实例和整个history存储不重叠；失败保持原值。
 * @return FILTER_OK、FILTER_ERROR_ARGUMENT或FILTER_NOT_INITIALIZED；OK仅表示复制成功。
 */
SampleFilterStatus_t SampleFilterGetState(const SampleFilter_t *filter, SampleFilterState_t *state);
#endif
