#ifndef MC02_HOST_HEATER_FAKE_CONTROLLER_H
#define MC02_HOST_HEATER_FAKE_CONTROLLER_H

typedef enum { PID_Integral_Limit = 1 } PID_Improvement_e;
typedef struct
{
    float Kp;
    float MaxOut;
    float Iout;
    float Output;
    float Last_Output;
} PIDInstance;
typedef struct
{
    float MaxOut;
    float IntegralLimit;
    float DeadBand;
    float Kp;
    float Ki;
    float Kd;
    PID_Improvement_e Improve;
} PID_Init_Config_s;
void PIDInit(PIDInstance *pid, PID_Init_Config_s *config);
float PIDCalculate(PIDInstance *pid, float measure, float ref);

#endif
