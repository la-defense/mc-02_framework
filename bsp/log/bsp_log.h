#ifndef _BSP_LOG_H
#define _BSP_LOG_H

#include "SEGGER_RTT.h"
#include "SEGGER_RTT_Conf.h"
#include <stdio.h>
#include <stdint.h>

#define BUFFER_INDEX 0

/**
 * @brief 日志系统初始化
 *
 */
void BSPLogInit();

/**
 * @brief 日志功能原型,供下面的LOGI,LOGW,LOGE等使用
 *
 */
#define LOG_PROTO(type, color, format, ...)                       \
        SEGGER_RTT_printf(BUFFER_INDEX, "  %s%s" format "\r\n%s", \
                          color,                                  \
                          type,                                   \
                          ##__VA_ARGS__,                          \
                          RTT_CTRL_RESET)

/*----------------------------------------下面是日志输出的接口-------------------------------------------------*/

/* 清屏 */
#define LOG_CLEAR() SEGGER_RTT_WriteString(0, "  " RTT_CTRL_CLEAR)

/* 无颜色日志输出 */
#define LOG(format, ...) LOG_PROTO("", "", format, ##__VA_ARGS__)

/**
 *  有颜色格式日志输出,建议使用这些宏来输出日志
 *  @attention 注意这些接口不支持浮点格式化输出,若有需要,请使用Float2Str()函数进行转换后再打印
 *  @note 在release版本上车使用时,与makefile中添加的宏DISABLE_LOG_SYSTEM一起使用,可以关闭日志系统
 */
#if DISABLE_LOG_SYSTEM
#define LOGINFO(format, ...) 
#define LOGWARNING(format, ...) 
#define LOGERROR(format, ...) 
#else
// information level
#define LOGINFO(format, ...) LOG_PROTO("I:", RTT_CTRL_TEXT_BRIGHT_GREEN, format, ##__VA_ARGS__)
// warning level
#define LOGWARNING(format, ...) LOG_PROTO("W:", RTT_CTRL_TEXT_BRIGHT_YELLOW, format, ##__VA_ARGS__)
// error level
#define LOGERROR(format, ...) LOG_PROTO("E:", RTT_CTRL_TEXT_BRIGHT_RED, format, ##__VA_ARGS__)
#endif //  DISABLE_LOG_SYSTEM

/* ---------------- 热路径日志限速(2026-09) ----------------
   高频路径(任务超时、电机离线、CAN 丢帧、串口错误…)如果不限速, 会把 CPU 和 1KB 的 RTT
   缓冲一起刷爆(本工程实测过: 每秒上千条, 连占用率汇总行都被挤掉)。

   用法:
       static LogRateLimit_t rl = {0};
       if (LogRateLimitAllow(&rl, 1000u))   // 同一类消息最多 1 条/秒
           LOGWARNING("[xxx] 出问题了 (累计 %lu 次, 期间限速 %lu 条)",
                      (unsigned long)rl.total, (unsigned long)rl.dropped);

   注意: 该组件用 HAL_GetTick() 计时; 调度器启动前 TIM23 时间基准还没跑起来(HAL_GetTick
   不递增), 启动阶段的限速日志会只打第一条 —— 这是可接受的(启动阶段不是热点)。 */
typedef struct
{
    uint32_t last_ms; /* 上次真正打印的时间 */
    uint32_t total;   /* 累计事件数(每次调用 +1) */
    uint32_t printed; /* 上次打印时的累计数 */
    uint32_t dropped; /* 上次打印至今被限速掉的条数(打印时读取) */
} LogRateLimit_t;

/**
 * @brief 热路径日志限速: 返回 1 表示这次允许打印(调用方随后自己 LOGxxx)
 */
uint8_t LogRateLimitAllow(LogRateLimit_t *rl, uint32_t period_ms);

/**
 * @brief 通过segger RTT打印日志,支持格式化输出,格式化输出的实现参考printf.
 * @attention !! 此函数不支持浮点格式化,若有需要,请使用Float2Str()函数进行转换后再打印 !!
 *
 * @param fmt 格式字符串
 * @param ... 参数列表
 * @return int 打印的log字符数
 */
int PrintLog(const char *fmt, ...);

/**
 * @brief 利用sprintf(),将float转换为字符串进行打印
 * @attention 浮点数需要转换为字符串后才能通过RTT打印
 *
 * @param str 转换后的字符串
 * @param va 待转换的float
 */
void Float2Str(char *str, float va);

#endif
