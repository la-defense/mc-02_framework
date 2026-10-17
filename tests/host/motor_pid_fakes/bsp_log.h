/** @file bsp_log.h
 * @brief 保留日志调用的类型，测试不向终端发送周期电机日志。
 */
#ifndef MC02_PID_TEST_LOG_H
#define MC02_PID_TEST_LOG_H
#include <stdint.h>
typedef struct { uint32_t total, dropped; } LogRateLimit_t;
static inline uint8_t LogRateLimitAllow(LogRateLimit_t *limit, uint32_t period)
{ (void)limit; (void)period; return 0u; }
#define LOGERROR(...) ((void)0)
#define LOGWARNING(...) ((void)0)
#define LOGINFO(...) ((void)0)
#endif
