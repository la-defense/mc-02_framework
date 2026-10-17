/**
* @file user_lib.h
* @brief 声明常用数值、角度、三维向量和滑动平均辅助函数。
* 各函数直接使用 float 或数组元素运算，不会自动换算物理单位。
* @author Wang Hongxi
 * @version V1.0.0
* @date 2021/2/18
 */
#ifndef _USER_LIB_H
#define _USER_LIB_H

#include "stdint.h"
#include "main.h"
#include "cmsis_os.h"
//#include "stm32f407xx.h"
#include "arm_math.h"

#define msin(x) (arm_sin_f32(x))
#define mcos(x) (arm_cos_f32(x))

typedef arm_matrix_instance_f32 mat;
// 矩阵运算接口使用 CMSIS-DSP 的单精度浮点实现。
#define MatAdd arm_mat_add_f32
#define MatSubtract arm_mat_sub_f32
#define MatMultiply arm_mat_mult_f32
#define MatTranspose arm_mat_trans_f32
#define MatInverse arm_mat_inverse_f32
/**
* @brief 创建指定行数和列数的单精度矩阵存储空间。
* @param m 要初始化的矩阵对象。
* @param row 行数。
* @param col 列数。
* @note 数据由 zmalloc 分配并清零；分配失败时 m->pData 为 NULL。
 */
void MatInit(mat *m, uint8_t row, uint8_t col);

/* 布尔值常量；仅在工程尚未定义时提供。 */
#ifndef TRUE
#define TRUE 1 /**< boolean true  */
#endif

#ifndef FALSE
#define FALSE 0 /**< boolean fails */
#endif

/* 圆周率常量；仅在工程尚未定义时提供。 */
#ifndef PI
#define PI 3.14159265354f
#endif

/* 把变量限制在上下限之间，直接改写传入的变量，不产生返回值。 */
#define VAL_LIMIT(val, min, max) \
    do                           \
    {                            \
        if ((val) <= (min))      \
        {                        \
            (val) = (min);       \
        }                        \
        else if ((val) >= (max)) \
        {                        \
            (val) = (max);       \
        }                        \
    } while (0)

/* 保留角度的小数部分，并将整数部分按360取余；负数遵循C语言余数规则。 */
#define ANGLE_LIMIT_360(val, angle)     \
    do                                  \
    {                                   \
        (val) = (angle) - (int)(angle); \
        (val) += (int)(angle) % 360;    \
    } while (0)

/* 输入大于180时减去360一次；其余输入保持不变。 */
#define ANGLE_LIMIT_360_TO_180(val) \
    do                              \
    {                               \
        if ((val) > 180)            \
            (val) -= 360;           \
    } while (0)

/* 返回两个数中较小或较大的一个。 */
#define VAL_MIN(a, b) ((a) < (b) ? (a) : (b))
#define VAL_MAX(a, b) ((a) > (b) ? (a) : (b))

/**
* @brief 分配并清零一块内存。
* @param size 要申请的字节数。
* @return 成功时返回内存地址；size 为0或申请失败时返回 NULL。
* @note 返回类型是 void *，赋给具体指针时需要转换为对应类型。
 */
void *zmalloc(size_t size);

/**
* @brief 用牛顿迭代近似计算平方根。
* @param x 输入数值。
* @return x 大于0时返回平方根近似值；x 小于等于0时返回0。
 */
float Sqrt(float x);
/**
* @brief 把数值限制在 -Limit 到 Limit 之间。
* @param num 要处理的数值。
* @param Limit 正向和负向使用的上限。
* @return 限制后的数值。
 */
float abs_limit(float num, float Limit);
/**
* @brief 根据正负返回符号。
* @param value 输入数值。
* @return value 大于等于0时返回1，否则返回-1。
 */
float sign(float value);
/**
* @brief 把严格位于两个边界之间的数值改为0。
* @param Value 输入数值。
* @param minValue 死区下界。
* @param maxValue 死区上界。
* @return Value 大于 minValue 且小于 maxValue 时返回0；等于边界或在范围外时保留原值。
 */
float float_deadband(float Value, float minValue, float maxValue);
/**
* @brief 判断两个浮点数的绝对差是否不超过固定误差。
* @param num 要比较的数值。
* @param Value 参照数值。
* @return 条件成立时返回1，否则返回0。
 */
float floatEqual(float num, float Value);
/**
* @brief 把浮点数限制在给定上下限内。
* @param Value 输入数值。
* @param minValue 下限。
* @param maxValue 上限。
* @return 限制后的数值。
 */
float float_constrain(float Value, float minValue, float maxValue);
/**
* @brief 把 int16_t 数值限制在给定上下限内。
* @param Value 输入数值。
* @param minValue 下限。
* @param maxValue 上限。
* @return 限制后的数值。
 */
int16_t int16_constrain(int16_t Value, int16_t minValue, int16_t maxValue);
/**
* @brief 按区间长度反复加减，把数值绕回给定范围。
* @param Input 输入数值。
* @param minValue 下限。
* @param maxValue 上限，必须大于 minValue。
* @return 绕回后的数值；上限小于下限时原样返回。
* @note 不检查相等的上下限；调用时须保证 minValue < maxValue。
 */
float loop_float_constrain(float Input, float minValue, float maxValue);
/**
* @brief 将角度绕回 -180 到 180 度范围。
* @param Ang 输入角度，单位度。
* @return 绕回后的角度。
 */
float theta_format(float Ang);

/**
* @brief 按当前实现处理小数部分并转成整数。
* @param raw 输入浮点数。
* @return 先向0取整；仅当小数部分大于0.5时再加1。
 */
int float_rounding(float raw);

/**
* @brief 把三维向量原地缩放到长度为1。
* @param v 至少包含3个 float 的可写数组，且向量长度须大于0。
* @return 返回同一个数组地址。
* @note 不检查空指针或零长度向量。
 */
float *Norm3d(float *v);

/**
* @brief 计算三维向量的长度。
* @param v 至少包含3个 float 的数组。
* @return 向量长度。
 */
float NormOf3d(float *v);

/**
* @brief 计算 v1 与 v2 的三维叉积。
* @param v1 至少包含3个 float 的数组。
* @param v2 至少包含3个 float 的数组。
* @param res 至少包含3个 float 的可写数组，用来保存结果。
 */
void Cross3d(float *v1, float *v2, float *res);

/**
* @brief 计算两个三维向量的点积。
* @param v1 至少包含3个 float 的数组。
* @param v2 至少包含3个 float 的数组。
* @return 点积结果。
 */
float Dot3d(float *v1, float *v2);

/**
* @brief 将缓冲区向前移动一格，放入新数据并返回当前平均值。
* @param new_data 要加入的最新数据。
* @param buf 至少包含 len 个 float 的可写数组。
* @param len 缓冲区元素个数，必须大于0。
* @return 更新后 len 个数据的平均值。
* @note 不检查空指针或 len；len 为0时不可调用。
 */
float AverageFilter(float new_data, float *buf, uint8_t len);

/* 将弧度绕回 -PI 到 PI；输入和输出单位都是弧度。 */
#define rad_format(Ang) loop_float_constrain((Ang), -PI, PI)

#endif
