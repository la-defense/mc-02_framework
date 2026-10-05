#ifndef MC02_HOST_BMI088_FAKE_CONTROLLER_H
#define MC02_HOST_BMI088_FAKE_CONTROLLER_H

#include "bsp_dwt.h"
#include "main.h"

typedef struct { float output; } PIDInstance;
typedef struct { float kp; } PID_Init_Config_s;
void PIDInit(PIDInstance *pid, PID_Init_Config_s *config);

#endif
