/** @file profile_io.h @brief 电机任务配置测试的时间和硬件入口声明，仅主机使用。 */
#ifndef MC02_MOTOR_PROFILE_IO_H
#define MC02_MOTOR_PROFILE_IO_H
#include <stdint.h>
typedef struct { uint32_t unused; } DWT_Probe_t;
static inline uint32_t DWT_ProbeStart(void) { return 0u; }
static inline void DWT_ProbeDone(volatile DWT_Probe_t *probe, uint32_t start) { (void)probe; (void)start; }
void CANProcessRx(void);
void USARTProcessRx(void);
void DJIMotorControl(void);
void LKMotorControl(void);
#endif
