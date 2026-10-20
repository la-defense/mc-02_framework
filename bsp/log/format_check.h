/**
 * @file format_check.h
 * @brief 为自有 printf 风格接口声明编译检查；MC02_PRINTF_FORMAT 不生成运行时代码。
 * @note 参数位置从1开始；格式方言与当前标准库一致，不引入HAL、RTT或RTOS。
 */
#ifndef MC02_FORMAT_CHECK_H
#define MC02_FORMAT_CHECK_H

#include <stdio.h>

/** @brief 声明格式串与首个可变参数的位置，不生成运行时值。
 * @param[in] format_index 格式串在函数参数中的位置，从1开始，须对应真实声明。
 * @param[in] first_argument 首个待检查实参的位置，从1开始；va_list入口填0。
 * @note 放在函数声明后，不需要初始化；不能证明指针有效或动态文字一定放得下。
 */
#if defined(__GNUC__)
#if defined(__MINGW_PRINTF_FORMAT)
#define MC02_PRINTF_FORMAT(format_index, first_argument) \
    __attribute__((__format__(__MINGW_PRINTF_FORMAT, format_index, first_argument)))
#else
#define MC02_PRINTF_FORMAT(format_index, first_argument) \
    __attribute__((__format__(__printf__, format_index, first_argument)))
#endif
#else
// 属性可被不支持它的编辑器解析；正式构建另外检查所用编译器和诊断能力。
#define MC02_PRINTF_FORMAT(format_index, first_argument)
#endif

#endif
