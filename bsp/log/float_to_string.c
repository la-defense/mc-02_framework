/**
 * @file float_to_string.c
 * @brief 实现 Float2Str 的三位小数舍入和有容量检查的文字生成，只依赖标准C。
 * @note 所有计算先写局部数组，检查通过后再交付；不读取时钟或访问 RTT。
 */
#include "float_to_string.h"
#include <math.h>
#include <stdint.h>
#include <string.h>

Float2StrStatus_t Float2Str(char *buffer, size_t capacity, float value)
{
    if (buffer == NULL)
        return FLOAT2STR_BAD_ARGUMENT;
    if (!isfinite(value))
        return FLOAT2STR_NONFINITE;

    // 上界不能先转为float，否则2147483647会变成2147483648而放过越界输入。
    double input = (double)value;
    if (input < (double)INT32_MIN || input > (double)INT32_MAX)
        return FLOAT2STR_OUT_OF_RANGE;

    double rounded = round(input * 1000.0);
    if (rounded < (double)INT32_MIN * 1000.0 || rounded > (double)INT32_MAX * 1000.0)
        return FLOAT2STR_OUT_OF_RANGE;

    // 范围检查后再转整数，千分数用int64_t保存，小数进位因此自然进入整数部分。
    int64_t thousandths = (int64_t)rounded;
    uint64_t magnitude = thousandths < 0 ? (uint64_t)(-thousandths) : (uint64_t)thousandths;
    uint32_t whole = (uint32_t)(magnitude / 1000u);
    uint32_t fraction = (uint32_t)(magnitude % 1000u);
    char reverse_digits[10];
    size_t digits = 0u;
    // 逐次取出个位，整数为0时也至少生成一个数字；最多处理十位整数。
    do
    {
        reverse_digits[digits++] = (char)('0' + whole % 10u);
        whole /= 10u;
    } while (whole != 0u);

    char candidate[FLOAT2STR_BUFFER_SIZE];
    size_t length = 0u;
    // 只有舍入后仍为负数才写负号，避免出现-0.000。
    if (thousandths < 0)
        candidate[length++] = '-';
    // 把先取出的低位反过来写，恢复正常的十进制顺序。
    while (digits != 0u)
        candidate[length++] = reverse_digits[--digits];
    candidate[length++] = '.';
    candidate[length++] = (char)('0' + fraction / 100u);
    candidate[length++] = (char)('0' + (fraction / 10u) % 10u);
    candidate[length++] = (char)('0' + fraction % 10u);
    candidate[length] = '\0';

    // 连同结尾0一起检查，放不下时不向调用者写半条文字。
    if (capacity <= length)
        return FLOAT2STR_BUFFER_TOO_SMALL;
    memcpy(buffer, candidate, length + 1u);
    return FLOAT2STR_OK;
}
