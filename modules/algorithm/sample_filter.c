/**
 * @file sample_filter.c
 * @brief 实现四类滤波和兼容整窗平均；先检查候选结果，再提交历史。
 * @author Wang Hongxi
 * @author modified by neozng
 */
#include "sample_filter.h"
#include <math.h>
#include <stdint.h>
#include <string.h>
#include <float.h>

#define SAMPLE_FILTER_INITIALIZED UINT32_C(0x53464c54)

// 指针先检查加法和对齐，实际可读写容量仍由调用者提供。
static int valid_storage(const void *pointer, size_t bytes, size_t alignment)
{
    uintptr_t address = (uintptr_t)pointer;
    return pointer != NULL && address % alignment == 0u && bytes <= UINTPTR_MAX - address;
}

static int storage_overlap(const void *a, size_t an, const void *b, size_t bn)
{
    if (an == 0u || bn == 0u) return 0;
    return (uintptr_t)a < (uintptr_t)b + bn && (uintptr_t)b < (uintptr_t)a + an;
}

// 检查种类实际使用的参数，不让填错字段的配置被静默接受。
static int valid_config(const SampleFilterConfig_t *c)
{
    if (!isfinite(c->dt_min_s) || !isfinite(c->dt_max_s) || !isfinite(c->tau_s) ||
        !isfinite(c->frequency_hz) || !isfinite(c->quality_factor) ||
        c->dt_min_s <= 0.0f || c->dt_max_s < c->dt_min_s)
        return 0;
    switch (c->type)
    {
    case SAMPLE_FILTER_LOWPASS1:
        return c->tau_s > 0.0f && c->frequency_hz == 0.0f &&
               c->quality_factor == 0.0f && c->window_length == 0u;
    case SAMPLE_FILTER_AVERAGE:
        return c->window_length >= 1u && c->window_length <= 255u && c->tau_s == 0.0f &&
               c->frequency_hz == 0.0f && c->quality_factor == 0.0f;
    case SAMPLE_FILTER_LOWPASS2:
    case SAMPLE_FILTER_NOTCH:
        return c->tau_s == 0.0f && c->window_length == 0u && c->frequency_hz > 0.0f &&
               (double)c->frequency_hz * c->dt_max_s < 0.5 &&
               (c->type == SAMPLE_FILTER_LOWPASS2 ? c->quality_factor == 0.0f : c->quality_factor > 0.0f);
    default:
        return 0;
    }
}

// 两状态梯形积分：g=tan(pi*f*dt)、k=1/Q，Butterworth固定k=sqrt(2)。
// 推导参考Andrew Simper, SvfLinearTrapOptimised2.pdf（2016-11-20版本）；自行按方程实现。
static int coefficients(const SampleFilterConfig_t *c, double dt, double *g, double *k, double *a1)
{
    *g = tan(3.14159265358979323846 * (double)c->frequency_hz * dt);
    *k = c->type == SAMPLE_FILTER_LOWPASS2 ? 1.41421356237309504880 : 1.0 / c->quality_factor;
    double denominator = 1.0 + *g * (*g + *k);
    *a1 = 1.0 / denominator;
    if (!isfinite(*g) || *g <= 0.0 || !isfinite(denominator) || denominator <= 0.0 ||
        !isfinite(*a1) || *a1 <= 0.0)
        return 0;
    // 对实际double状态转移矩阵做二阶Schur检查；过于极端而舍入到不稳定边界时拒绝。
    double a = 2.0 * *a1 - 1.0, b = -2.0 * *g * *a1;
    double d = 1.0 - 2.0 * *g * *g * *a1;
    double trace = a + d, determinant = a * d + b * b;
    return isfinite(trace) && isfinite(determinant) && 1.0 - determinant > 0.0 &&
           1.0 - trace + determinant > 0.0 && 1.0 + trace + determinant > 0.0;
}

static int usable_coefficients(const SampleFilterConfig_t *c, double dt)
{
    if (c->type == SAMPLE_FILTER_LOWPASS1)
    {
        double alpha = dt / ((double)c->tau_s + dt);
        return isfinite(alpha) && alpha > 0.0 && alpha <= 1.0 && 1.0 - alpha < 1.0;
    }
    if (c->type == SAMPLE_FILTER_AVERAGE) return 1;
    double g, k, a1;
    return coefficients(c, dt, &g, &k, &a1);
}

// 仅验证本实例的存储；不同实例之间历史不重叠由调用者保证，不建立全局登记表。
static int history_storage(const SampleFilterConfig_t *c, float *history, size_t capacity)
{
    if (c->type != SAMPLE_FILTER_AVERAGE) return history == NULL && capacity == 0u;
    return capacity >= c->window_length && capacity <= SIZE_MAX / sizeof(float) &&
           valid_storage(history, capacity * sizeof(float), _Alignof(float));
}

static SampleFilterStatus_t live_filter(const SampleFilter_t *filter)
{
    if (!valid_storage(filter, sizeof(*filter), _Alignof(SampleFilter_t)))
        return FILTER_ERROR_ARGUMENT;
    if (filter->_initialized != SAMPLE_FILTER_INITIALIZED || filter->_owner != (uintptr_t)filter ||
        !valid_config(&filter->_config) || !history_storage(&filter->_config, filter->_history, filter->_capacity))
        return FILTER_NOT_INITIALIZED;
    if (storage_overlap(filter, sizeof(*filter), filter->_history, filter->_capacity * sizeof(float)))
        return FILTER_ERROR_ARGUMENT;
    return FILTER_OK;
}

static void waiting_result(SampleFilter_t *filter)
{
    memset(&filter->_result, 0, sizeof(filter->_result));
    filter->_result.type = filter->_config.type;
    filter->_result.last_status = FILTER_WAITING;
    filter->_result.needs_rebuild = 1u;
}

SampleFilterStatus_t SampleFilterInit(SampleFilter_t *filter, const SampleFilterConfig_t *config,
                                    float *history, size_t capacity)
{
    if (!valid_storage(filter, sizeof(*filter), _Alignof(SampleFilter_t)) ||
        !valid_storage(config, sizeof(*config), _Alignof(SampleFilterConfig_t)) ||
        storage_overlap(filter, sizeof(*filter), config, sizeof(*config)))
        return FILTER_ERROR_ARGUMENT;
    if (!valid_config(config)) return FILTER_ERROR_CONFIG;
    if (!history_storage(config, history, capacity) ||
        storage_overlap(filter, sizeof(*filter), history, capacity * sizeof(float)) ||
        storage_overlap(config, sizeof(*config), history, capacity * sizeof(float)))
        return FILTER_ERROR_ARGUMENT;
    if (!usable_coefficients(config, config->dt_min_s) || !usable_coefficients(config, config->dt_max_s))
        return FILTER_ERROR_CONFIG;
    // 所有可能失败的检查完成后，再接受配置并清零调用者窗口。
    SampleFilter_t candidate = {0};
    candidate._initialized = SAMPLE_FILTER_INITIALIZED;
    candidate._owner = (uintptr_t)filter;
    candidate._config = *config;
    candidate._history = history;
    candidate._capacity = capacity;
    waiting_result(&candidate);
    if (config->type == SAMPLE_FILTER_AVERAGE)
        memset(history, 0, config->window_length * sizeof(float));
    *filter = candidate;
    return FILTER_OK;
}

SampleFilterStatus_t SampleFilterReset(SampleFilter_t *filter)
{
    SampleFilterStatus_t status = live_filter(filter);
    if (status != FILTER_OK) return status;
    // 重置清除旧积分状态及平均历史，但不改变配置和存储归属。
    filter->_state1 = filter->_state2 = 0.0;
    if (filter->_config.type == SAMPLE_FILTER_AVERAGE)
        memset(filter->_history, 0, filter->_config.window_length * sizeof(float));
    waiting_result(filter);
    return FILTER_OK;
}

// 缺样和数据失败只撤销有效性，最后数值和已接受历史仍供诊断。
static SampleFilterStatus_t invalidate(SampleFilter_t *filter, SampleFilterStatus_t cause)
{
    filter->_result.valid = 0u;
    filter->_result.needs_rebuild = 1u;
    filter->_result.last_status = cause;
    return cause;
}

static void accept(SampleFilter_t *candidate, float value, SampleFilterStatus_t status)
{
    candidate->_result.last_output = value;
    candidate->_result.valid = 1u;
    candidate->_result.needs_rebuild = 0u;
    candidate->_result.last_status = status;
    if (candidate->_result.accepted_count != UINT64_MAX) ++candidate->_result.accepted_count;
    if (status == FILTER_REBUILT && candidate->_result.rebuild_count != UINT64_MAX)
        ++candidate->_result.rebuild_count;
}

SampleFilterStatus_t SampleFilterUpdate(SampleFilter_t *filter, uint8_t sample_present,
                                      float input, float dt_s, float *output)
{
    SampleFilterStatus_t status = live_filter(filter);
    if (status != FILTER_OK) return status;
    if (sample_present > 1u || !valid_storage(output, sizeof(*output), _Alignof(float)) ||
        storage_overlap(filter, sizeof(*filter), output, sizeof(*output)) ||
        storage_overlap(filter->_history, filter->_capacity * sizeof(float), output, sizeof(*output)))
        return FILTER_ERROR_ARGUMENT;
    if (!sample_present) return invalidate(filter, FILTER_NO_SAMPLE);
    if (!isfinite(input)) return invalidate(filter, FILTER_ERROR_NONFINITE);
    SampleFilter_t next = *filter;
    if (next._result.needs_rebuild)
    {
        // 首次及中断后以当前样本建立常值稳态，不把未知的旧间隔送入计算。
        next._state1 = next._config.type == SAMPLE_FILTER_LOWPASS1 ? input : 0.0;
        next._state2 = input;
        if (next._config.type == SAMPLE_FILTER_AVERAGE)
            for (size_t i = 0u; i < next._config.window_length; ++i) next._history[i] = input;
        accept(&next, input, FILTER_REBUILT);
        *filter = next;
        *output = input;
        return FILTER_REBUILT;
    }
    if (!isfinite(dt_s) || dt_s < next._config.dt_min_s || dt_s > next._config.dt_max_s)
        return invalidate(filter, FILTER_ERROR_DT);
    if (!isfinite(next._state1) || !isfinite(next._state2))
        return invalidate(filter, FILTER_ERROR_NUMERIC);
    double value = input;
    if (next._config.type == SAMPLE_FILTER_LOWPASS1)
    {
        double alpha = (double)dt_s / ((double)next._config.tau_s + dt_s);
        next._state1 += alpha * ((double)input - next._state1);
        value = next._state1;
    }
    else if (next._config.type == SAMPLE_FILTER_AVERAGE)
    {
        float averaged;
        // 兼容平均已经先检查整窗再移动；其成功后本分支不再有可失败步骤。
        if (MovingAverageF32(input, next._history, next._capacity, next._config.window_length, &averaged) != MATH_OK)
            return invalidate(filter, FILTER_ERROR_NONFINITE);
        value = averaged;
    }
    else
    {
        double g, k, a1;
        if (!coefficients(&next._config, dt_s, &g, &k, &a1))
            return invalidate(filter, FILTER_ERROR_NUMERIC);
        // 梯形积分隐式方程化为两步求解，保留等效积分历史，允许逐样本改变dt。
        double band = a1 * (next._state1 + g * ((double)input - next._state2));
        double low = next._state2 + g * band;
        next._state1 = 2.0 * band - next._state1;
        next._state2 = 2.0 * low - next._state2;
        value = next._config.type == SAMPLE_FILTER_LOWPASS2 ? low : (double)input - k * band;
    }
    if (!isfinite(value) || fabs(value) > FLT_MAX || !isfinite(next._state1) || !isfinite(next._state2))
        return invalidate(filter, FILTER_ERROR_NUMERIC);
    accept(&next, (float)value, FILTER_OK);
    *filter = next;
    *output = (float)value;
    return FILTER_OK;
}

SampleFilterStatus_t SampleFilterGetState(const SampleFilter_t *filter, SampleFilterState_t *state)
{
    SampleFilterStatus_t status = live_filter(filter);
    if (status != FILTER_OK) return status;
    if (!valid_storage(state, sizeof(*state), _Alignof(SampleFilterState_t)) ||
        storage_overlap(filter, sizeof(*filter), state, sizeof(*state)) ||
        storage_overlap(filter->_history, filter->_capacity * sizeof(float), state, sizeof(*state)))
        return FILTER_ERROR_ARGUMENT;
    *state = filter->_result;
    return FILTER_OK;
}

MathStatus_t MovingAverageF32(float new_data, float *buffer, size_t capacity,
                            size_t length, float *output)
{
    if (buffer == NULL || output == NULL) return MATH_BAD_ARGUMENT;
    if (length == 0u || length > 255u) return MATH_BAD_RANGE;
    if (capacity < length) return MATH_BAD_ARGUMENT;
    size_t bytes = length * sizeof(float);
    uintptr_t start = (uintptr_t)buffer, out = (uintptr_t)output;
    // 地址先检查加法和对齐，避免后续读取无效或把结果写回历史中。
    if (start % _Alignof(float) != 0u || out % _Alignof(float) != 0u ||
        bytes > UINTPTR_MAX - start || sizeof(float) > UINTPTR_MAX - out ||
        (start < out + sizeof(float) && out < start + bytes))
        return MATH_BAD_ARGUMENT;
    if (!isfinite(new_data)) return MATH_NONFINITE;
    double sum = new_data;
    // 连将被丢弃的最早元素也检查，发现历史损坏时保持整窗供调用者处理。
    for (size_t i = 0u; i < length; ++i)
    {
        if (!isfinite(buffer[i])) return MATH_NONFINITE;
        if (i != 0u) sum += buffer[i];
    }
    // 有限float的平均值仍在float范围内，double累加避免中间求和溢出。
    float candidate = (float)(sum / (double)length);
    memmove(buffer, buffer + 1u, (length - 1u) * sizeof(float));
    buffer[length - 1u] = new_data;
    *output = candidate;
    return MATH_OK;
}
