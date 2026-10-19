/**
 * @file bsp_log.h
 * @brief LK 主机回归的日志替身；测试期间不输出日志。
 */
#ifndef MC02_LK_TEST_LOG_H
#define MC02_LK_TEST_LOG_H
#include <stdarg.h>
#include <stdint.h>
typedef struct { uint32_t total, dropped; } LogRateLimit_t;
static inline uint8_t LogRateLimitAllow(LogRateLimit_t *limit, uint32_t period_ms)
{
    (void)limit;
    (void)period_ms;
    return 0u;
}
static inline void LKTestLogWarning(const char *format, ...)
{
    (void)format;
}
#define LOGERROR(...) ((void)0)
#define LOGWARNING(...) LKTestLogWarning(__VA_ARGS__)
#define LOGINFO(...) LKTestLogWarning(__VA_ARGS__)
#endif
