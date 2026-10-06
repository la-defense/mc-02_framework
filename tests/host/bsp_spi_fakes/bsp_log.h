#ifndef MC02_HOST_SPI_FAKE_LOG_H
#define MC02_HOST_SPI_FAKE_LOG_H
#include <stdint.h>
typedef struct { uint32_t total; uint32_t dropped; } LogRateLimit_t;
static inline uint8_t LogRateLimitAllow(LogRateLimit_t *limit, uint32_t interval)
{ (void)limit; (void)interval; return 1u; }
#define LOGERROR(...) ((void)0)
#endif
