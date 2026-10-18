/**
 * @file sample_filter.h
 * @brief 调用者保存历史的整窗平均；无需初始化函数，先清零或填写有限历史再调用。
 * @author Wang Hongxi
 * @author modified by neozng
 */
#ifndef MC02_SAMPLE_FILTER_H
#define MC02_SAMPLE_FILTER_H
#include "math_utils.h"
#include <stddef.h>

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
#endif
