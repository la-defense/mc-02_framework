/**
 * @file math_utils.h
 * @brief 标量和角度纯计算入口；无需初始化，结果成功后才写入调用者存储。
 * @note 输入须有限；所有输出指针须非空、按其类型对齐且可写。
 * 原 user_lib 数值工具由本接口替代；矩阵入口见 dsp_matrix.h，向量见 vector3.h。
 * @author Wang Hongxi
 * @author modified by neozng
 */
#ifndef MC02_MATH_UTILS_H
#define MC02_MATH_UTILS_H

#include <stdint.h>

/** @brief 计算结果；任何失败都不改写输出或调用者历史数据。 */
typedef enum
{
    MATH_OK = 0,
    MATH_BAD_ARGUMENT, /**< 空指针、容量不符或不允许的缓冲重叠。 */
    MATH_BAD_RANGE,    /**< 上下限、窗口或容差不在允许范围。 */
    MATH_NONFINITE,    /**< 输入含 NaN 或 Inf。 */
    MATH_OVERFLOW,     /**< 结果超过输出类型可表示范围。 */
    MATH_ZERO_VECTOR   /**< 零向量没有可计算的单位方向。 */
} MathStatus_t;

/** @brief 求非负数平方根。
 * @param[in] value 有限且不小于0；不换算单位。
 * @param[out] output 非空结果，失败保持原值。
 * @return MATH_OK、MATH_BAD_ARGUMENT、MATH_NONFINITE 或 MATH_BAD_RANGE。
 */
MathStatus_t MathSqrtF32(float value, float *output);

/** @brief 将有限数限制在闭区间内。
 * @param[in] value 有限输入，与上下限单位一致。
 * @param[in] minimum 有限下限，允许等于 maximum。
 * @param[in] maximum 有限上限，须不小于 minimum。
 * @param[out] output 非空结果，失败保持原值。
 * @return MATH_OK、MATH_BAD_ARGUMENT、MATH_NONFINITE 或 MATH_BAD_RANGE。
 */
MathStatus_t MathClampF32(float value, float minimum, float maximum, float *output);

/** @brief 将16位整数限制在闭区间内。
 * @param[in] value 输入，与上下限单位一致。
 * @param[in] minimum 下限，允许等于 maximum。
 * @param[in] maximum 上限，须不小于 minimum。
 * @param[out] output 非空结果，失败保持原值。
 * @return MATH_OK、MATH_BAD_ARGUMENT 或 MATH_BAD_RANGE。
 */
MathStatus_t MathClampI16(int16_t value, int16_t minimum, int16_t maximum, int16_t *output);

/** @brief 将数值限制在负上限和正上限之间。
 * @param[in] value 有限输入。
 * @param[in] limit 有限且不小于0，与 value 单位一致。
 * @param[out] output 非空结果，失败保持原值。
 * @return 与 MathClampF32 相同。
 */
MathStatus_t MathAbsLimitF32(float value, float limit, float *output);

/** @brief 返回输入的正负方向，正零和负零均返回1。
 * @param[in] value 有限输入。
 * @param[out] output 非空结果，成功为1或-1，失败保持原值。
 * @return MATH_OK、MATH_BAD_ARGUMENT 或 MATH_NONFINITE。
 */
MathStatus_t MathSignF32(float value, float *output);

/** @brief 将严格位于死区内的数值改为0，等于边界时保留输入。
 * @param[in] value 有限输入，与边界单位一致。
 * @param[in] minimum 有限下界，允许等于 maximum。
 * @param[in] maximum 有限上界，须不小于 minimum。
 * @param[out] output 非空结果，失败保持原值。
 * @return 与 MathClampF32 相同。
 */
MathStatus_t MathDeadbandF32(float value, float minimum, float maximum, float *output);

/** @brief 判断两个有限数的绝对差是否不大于给定容差。
 * @param[in] a 有限输入。
 * @param[in] b 有限参照值，与 a 单位一致。
 * @param[in] absolute_tolerance 有限且不小于0，与 a 单位一致；0表示精确相等。
 * @param[out] output 非空结果，成功写0或1，失败保持原值。
 * @return MATH_OK、MATH_BAD_ARGUMENT、MATH_NONFINITE 或 MATH_BAD_RANGE。
 */
MathStatus_t MathNearlyEqualF32(float a, float b, float absolute_tolerance, uint8_t *output);

/** @brief 舍入到最近整数，恰好半数时远离0。
 * @param[in] value 有限输入，如1.5得2、-1.5得-2。
 * @param[out] output 非空 int32_t 结果，超出范围或其他失败时保持原值。
 * @return MATH_OK、MATH_BAD_ARGUMENT、MATH_NONFINITE 或 MATH_OVERFLOW。
 */
MathStatus_t MathRoundI32(float value, int32_t *output);

/** @brief 将数值绕回半开区间 [minimum, maximum)。
 * @param[in] value 有限输入，与边界单位一致。
 * @param[in] minimum 有限下界，必须小于 maximum。
 * @param[in] maximum 有限上界；等于此值的输入返回 minimum。
 * @param[out] output 非空结果，浮点舍入到上界时返回下界；失败保持原值。
 * @return MATH_OK、MATH_BAD_ARGUMENT、MATH_NONFINITE 或 MATH_BAD_RANGE。
 * @note 使用固定次数的余数计算，不按绕回圈数反复加减。
 */
MathStatus_t MathWrapF32(float value, float minimum, float maximum, float *output);

/** @brief 将角度绕回 [-180, 180)。
 * @param[in] degrees 有限角度，单位度。
 * @param[out] output 非空角度，单位度，失败保持原值。
 * @return 与 MathWrapF32 相同。
 */
MathStatus_t MathWrapDegreesF32(float degrees, float *output);

/** @brief 将弧度绕回 [-pi, pi)，pi采用单精度常量3.14159265358979323846f。
 * @param[in] radians 有限角度，单位弧度。
 * @param[out] output 非空角度，单位弧度，失败保持原值。
 * @return 与 MathWrapF32 相同。
 */
MathStatus_t MathWrapRadiansF32(float radians, float *output);

#endif
