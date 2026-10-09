#ifndef MC02_MOTOR_PROFILE_IO_H
#define MC02_MOTOR_PROFILE_IO_H
#include <stdint.h>
typedef struct { uint32_t unused; } DWT_Probe_t;
static inline uint32_t DWT_ProbeStart(void) { return 0u; }
static inline void DWT_ProbeDone(volatile DWT_Probe_t *probe, uint32_t start) { (void)probe; (void)start; }
void CANProcessRx(void);
void USARTProcessRx(void);
void VCPProcessRx(void);
void DJIMotorControl(void);
void LKMotorControl(void);
#endif
