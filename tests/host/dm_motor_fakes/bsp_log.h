/** @file bsp_log.h @brief 主机生产集成测试关闭日志输出。 */
#ifndef MC02_DM_MOTOR_TEST_LOG_H
#define MC02_DM_MOTOR_TEST_LOG_H
#include <stdint.h>
typedef struct { uint32_t total, dropped; } LogRateLimit_t;
static inline uint8_t LogRateLimitAllow(LogRateLimit_t *limit, uint32_t period_ms)
{ (void)limit; (void)period_ms; return 0u; }
static inline void DMTestLog(const char *format, ...)
{ (void)format; }
#define LOGERROR(...) DMTestLog(__VA_ARGS__)
#define LOGWARNING(...) DMTestLog(__VA_ARGS__)
#define LOGINFO(...) DMTestLog(__VA_ARGS__)
#endif
