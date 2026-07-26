#ifndef POWER_CONTROL_H
#define POWER_CONTROL_H

#include "general_def.h"
#include "stdint.h"

#define TOQUE_COEFFICIENT_3508 1.9067382e-5f     // 3508电机电流转换扭矩 Nm

#define ERROR_POWERDISTRIBUTE 15.0f
#define PROP_POWERDISTRIBUTE 5.0f

typedef struct
{
    uint8_t rlsEnabled;
    float k1, k2, k3;
} PowerManagerInstance;

/* powermanager初始化配置 */
typedef struct
{
    float k1, k2, k3;
} PowerManager_Init_Config_s;

PowerManagerInstance *PowerControlInit(PowerManager_Init_Config_s *power_manager_config);
void PowerManagerFeedback(PowerManagerInstance *instance, float maxPowerLimited);
void PowerManagerEnable(PowerManagerInstance *instance);
void PowerManagerUpdate(float power, float sample_a, float sample_b);

#endif // !POWER_CONTROL_H
