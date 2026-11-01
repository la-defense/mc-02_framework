/** @file fast_trig.h
 * @brief 以弧度计算快速正弦和余弦，由构建选择数值后端。
 * @note 输入须有限；除以 2π 的结果以及其加减 1 后均须在 int32_t 范围内；返回近似值。
 */
#ifndef MC02_FAST_TRIG_H
#define MC02_FAST_TRIG_H

/** @brief 计算角度的快速正弦。
 * @param[in] radians 输入角度，单位 rad；须满足文件头所述输入条件。
 * @return 后端计算的近似正弦值。
 */
float FastTrigSinF32(float radians);

/** @brief 计算角度的快速余弦。
 * @param[in] radians 输入角度，单位 rad；须满足文件头所述输入条件。
 * @return 后端计算的近似余弦值。
 */
float FastTrigCosF32(float radians);

#endif
