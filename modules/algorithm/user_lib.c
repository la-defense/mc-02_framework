/**
* @file user_lib.c
* @brief 实现常用数值、角度、三维向量、矩阵和滑动平均辅助函数。
* @author Wang Hongxi
* @author modified by neozng
 * @version 0.2 beta
* @date 2021/2/18
 */
#include "stdlib.h"
#include "memory.h"
#include "user_lib.h"
#include "math.h"
#include "main.h"

#ifdef _CMSIS_OS_H
#define user_malloc pvPortMalloc
#else
#define user_malloc malloc
#endif

void *zmalloc(size_t size)
{
    if (size == 0u)
        return NULL;
    void *ptr = malloc(size);
    if (ptr == NULL)
        return NULL;
    memset(ptr, 0, size);
    return ptr;
}

// 快速开方
float Sqrt(float x)
{
    float y;
    float delta;
    float maxError;

    if (x <= 0)
    {
        return 0;
    }

    // 从输入值的一半开始，再逐次修正平方根估计。
    y = x / 2;

    // 当平方结果与输入的差值已经够小时停止。
    maxError = x * 0.001f;

    do
    {
        delta = (y * y) - x;
        y -= delta / (2 * y);
    } while (delta > maxError || delta < -maxError);

    return y;
}

// 绝对值限制
float abs_limit(float num, float Limit)
{
    if (num > Limit)
    {
        num = Limit;
    }
    else if (num < -Limit)
    {
        num = -Limit;
    }
    return num;
}

// 判断符号位
float sign(float value)
{
    if (value >= 0.0f)
    {
        return 1.0f;
    }
    else
    {
        return -1.0f;
    }
}

// 浮点死区
float float_deadband(float Value, float minValue, float maxValue)
{
    if (Value < maxValue && Value > minValue)
    {
        Value = 0.0f;
    }
    return Value;
}

// 判断数值相等
float floatEqual(float num, float Value)
{
    return (num <= (Value + 1.192092896e-07f)) && (num >= (Value - 1.192092896e-07f));
}

// 限幅函数
float float_constrain(float Value, float minValue, float maxValue)
{
    if (Value < minValue)
        return minValue;
    else if (Value > maxValue)
        return maxValue;
    else
        return Value;
}

// 限幅函数
int16_t int16_constrain(int16_t Value, int16_t minValue, int16_t maxValue)
{
    if (Value < minValue)
        return minValue;
    else if (Value > maxValue)
        return maxValue;
    else
        return Value;
}

// 通过加减一个区间长度，把输入绕回给定范围。
float loop_float_constrain(float Input, float minValue, float maxValue)
{
    if (maxValue < minValue)
    {
        return Input;
    }

    if (Input > maxValue)
    {
        float len = maxValue - minValue;
        // 超过上限时每次减去一个区间长度，直到回到范围内。
        while (Input > maxValue)
        {
            Input -= len;
        }
    }
    else if (Input < minValue)
    {
        float len = maxValue - minValue;
        // 低于下限时每次加上一个区间长度，直到回到范围内。
        while (Input < minValue)
        {
            Input += len;
        }
    }
    return Input;
}

// 输入和输出均为角度；弧度版本由头文件中的 rad_format 宏提供。
float theta_format(float Ang)
{
    return loop_float_constrain(Ang, -180.0f, 180.0f);
}

int float_rounding(float raw)
{
    static int integer;
    static float decimal;
    integer = (int)raw;
    decimal = raw - integer;
    if (decimal > 0.5f)
        integer++;
    return integer;
}

// 把向量按自身长度缩放，使结果长度为1。
float *Norm3d(float *v)
{
    float len = Sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
    v[0] /= len;
    v[1] /= len;
    v[2] /= len;
    return v;
}

// 计算模长
float NormOf3d(float *v)
{
    return Sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
}

// 三维向量叉乘v1 x v2
void Cross3d(float *v1, float *v2, float *res)
{
    res[0] = v1[1] * v2[2] - v1[2] * v2[1];
    res[1] = v1[2] * v2[0] - v1[0] * v2[2];
    res[2] = v1[0] * v2[1] - v1[1] * v2[0];
}

// 三维向量点乘
float Dot3d(float *v1, float *v2)
{
    return v1[0] * v2[0] + v1[1] * v2[1] + v1[2] * v2[2];
}

/* 把后续元素前移一格，在末尾放入新数据，再计算整段平均值。 */
float AverageFilter(float new_data, float *buf, uint8_t len)
{
    float sum = 0;
    // 将较新的旧数据前移，最早的一项会被新值替换。
    for (uint8_t i = 0; i < len - 1; i++)
    {
        buf[i] = buf[i + 1];
        sum += buf[i];
    }
    buf[len - 1] = new_data;
    sum += new_data;
    return sum / len;
}

/** 分配并清零矩阵数据；失败时 pData 为 NULL，函数不返回状态。 */
void MatInit(mat *m, uint8_t row, uint8_t col)
{
    m->numCols = col;
    m->numRows = row;
    m->pData = (float *)zmalloc(row * col * sizeof(float));
}
