#include <stdint.h>
typedef struct { uint32_t unused; } LogRateLimit_t;
static inline uint8_t LogRateLimitAllow(LogRateLimit_t *p, uint32_t ms) { (void)p; (void)ms; return 0; }
#define LOGERROR(...) ((void)0)
#define LOGWARNING(...) ((void)0)
#define LOGINFO(...) ((void)0)
