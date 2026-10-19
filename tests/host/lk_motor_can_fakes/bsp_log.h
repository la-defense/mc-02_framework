/**
 * @file bsp_log.h
 * @brief 生产 CAN 集成回归使用的静默日志替身。
 */
#ifndef MC02_LK_CAN_TEST_LOG_H
#define MC02_LK_CAN_TEST_LOG_H

#include <stdint.h>
typedef struct { uint32_t total, dropped; } LogRateLimit_t;
static inline uint8_t LogRateLimitAllow(LogRateLimit_t *limit, uint32_t period_ms)
{
    (void)limit;
    (void)period_ms;
    return 0u;
}
static inline void LKTestCanLog(const char *format, ...)
{
    (void)format;
}
#define LOGERROR(...) LKTestCanLog(__VA_ARGS__)
#define LOGWARNING(...) LKTestCanLog(__VA_ARGS__)
#define LOGINFO(...) LKTestCanLog(__VA_ARGS__)

#endif
