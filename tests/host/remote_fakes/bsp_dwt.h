#include <stdint.h>
typedef struct { uint32_t unused; } DWT_Probe_t;
static inline uint32_t DWT_ProbeStart(void) { return 0; }
static inline void DWT_ProbeDone(volatile DWT_Probe_t *p, uint32_t start) { (void)p; (void)start; }
