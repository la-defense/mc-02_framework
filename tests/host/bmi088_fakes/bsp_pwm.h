#ifndef MC02_HOST_BMI088_FAKE_BSP_PWM_H
#define MC02_HOST_BMI088_FAKE_BSP_PWM_H

#include <stdint.h>

typedef struct pwm_ins_temp
{
    void *htim;
    uint32_t channel;
    float period;
    float dutyratio;
    void (*callback)(struct pwm_ins_temp *);
    void *id;
} PWMInstance;
typedef struct
{
    void *htim;
    uint32_t channel;
    float period;
    float dutyratio;
    void (*callback)(PWMInstance *);
    void *id;
} PWM_Init_Config_s;
PWMInstance *PWMRegister(PWM_Init_Config_s *config);

#endif
