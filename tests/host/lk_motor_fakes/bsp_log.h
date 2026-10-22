/* 测试桩沿用生产事件目录；此类调用方测试关闭日志副作用。 */
#undef DISABLE_LOG_SYSTEM
#define DISABLE_LOG_SYSTEM 1
#include "../../../bsp/log/bsp_log.h"
/* 这些模块测试只验证业务结果；logger 核心在独立生产接口测试中验证。 */
#undef BSP_LOG_EVENT_0
#undef BSP_LOG_EVENT_1
#undef BSP_LOG_EVENT_2
#undef BSP_LOG_EVENT_3
#undef BSP_LOG_EVENT_4
#undef BSP_LOG_EVENT_5
#undef BSP_LOG_EVENT_6
#undef BSP_LOG_EVENT_7
#undef BSP_LOG_EVENT_8
#define BSP_LOG_EVENT_0(...) ((void)0)
#define BSP_LOG_EVENT_1(...) ((void)0)
#define BSP_LOG_EVENT_2(event, field0, field1) \
    do { (void)(event); (void)sizeof(field0); (void)sizeof(field1); } while (0)
#define BSP_LOG_EVENT_3(...) ((void)0)
#define BSP_LOG_EVENT_4(...) ((void)0)
#define BSP_LOG_EVENT_5(...) ((void)0)
#define BSP_LOG_EVENT_6(...) ((void)0)
#define BSP_LOG_EVENT_7(...) ((void)0)
#define BSP_LOG_EVENT_8(...) ((void)0)
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
