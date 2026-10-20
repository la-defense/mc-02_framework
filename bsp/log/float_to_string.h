/**
 * @file float_to_string.h
 * @brief 将有限 float 转成固定三位小数文字；Float2Str 不需要初始化或硬件。
 * @note 调用者提供可写数组；不同数组可同时使用，同一数组由调用者串行访问。
 */
#ifndef MC02_FLOAT_TO_STRING_H
#define MC02_FLOAT_TO_STRING_H

#include <stddef.h>

/** @brief 最大结果所需容量，包含负号、十位整数、小数点、三位小数及结尾0。 */
#define FLOAT2STR_BUFFER_SIZE 16u

/** @brief 文字转换结果，失败时调用者数组的原字节保持不变。 */
typedef enum
{
    FLOAT2STR_OK = 0,              /**< 完整文字及结尾0已写入。 */
    FLOAT2STR_BAD_ARGUMENT,        /**< 输出指针为空。 */
    FLOAT2STR_NONFINITE,           /**< 输入为 NaN 或无穷大。 */
    FLOAT2STR_OUT_OF_RANGE,        /**< 输入或舍入后的值超出允许范围。 */
    FLOAT2STR_BUFFER_TOO_SMALL     /**< 容量不足以保存完整文字及结尾0。 */
} Float2StrStatus_t;

/**
 * @brief 输出固定三位小数，舍入到最近的千分之一，恰好半数时远离0。
 * @param[out] buffer 调用者长期或局部保存的可写数组，不能为 NULL。
 * @param[in] capacity 数组容量，单位字节，包含结尾0；数值合法时容量为0返回容量不足。
 * @param[in] value 实际 float 数值，须有限且处于[-2147483648,2147483647]；舍入后仍须在此范围内。
 * @return 返回参数错误、非有限值、超范围或容量不足；仅 FLOAT2STR_OK 写入结果。
 * @note 不需要初始化。使用 ASCII 数字和小数点，不使用科学计数法；正负零及舍入后的零均输出0.000。
 *       失败保留整个数组；成功仅写文字和结尾0。FLOAT2STR_BUFFER_SIZE 可容纳全部允许结果。
 *       不申请内存、不保存共享历史。半数规则针对实际 float 值，而非源码中十进制写法的理想值。
 */
Float2StrStatus_t Float2Str(char *buffer, size_t capacity, float value);

#endif
