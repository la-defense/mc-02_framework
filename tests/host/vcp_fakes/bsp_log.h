#ifndef MC02_VCP_FAKE_LOG_H
#define MC02_VCP_FAKE_LOG_H
#include <stdint.h>
typedef struct { uint32_t unused; } LogRateLimit_t;
static inline uint8_t LogRateLimitAllow(LogRateLimit_t *state, uint32_t interval)
{ (void)state; (void)interval; return 0u; }
#define LOGWARNING(...) ((void)0)
#define LOGINFO(...) ((void)0)
#endif
