#include "power_manager.h"
#include "RLS.h"
#include "string.h"
#include "stdlib.h"

float k[2] = {100.2f, 1.2f};

static PowerManagerInstance *power_control_instance = NULL;

static RLS_t RLS;

PowerManagerInstance *PowerControlInit(PowerManager_Init_Config_s *power_manager_config)
{
    power_control_instance = (PowerManagerInstance *)malloc(sizeof(PowerManagerInstance));
    memset(power_control_instance, 0, sizeof(PowerManagerInstance));

    power_control_instance->k1 = power_manager_config->k1;
    power_control_instance->k2 = power_manager_config->k2;
    power_control_instance->k3 = power_manager_config->k3;

    // // 初始化RLS滤波器，用于解算k1，k2
    // RLS_Init(&RLS, 2, 1e-5f, 0.99999f);
    // RLS_Reset(&RLS);
    // RLS_SetParam(&RLS, k);

    return power_control_instance;
}

void PowerManagerFeedback(PowerManagerInstance *instance, float power)
{
    ;
}

void PowerManagerEnable(PowerManagerInstance *instance)
{
    instance->rlsEnabled = 1;
}

void PowerManagerUpdate(float power, float sample_a, float sample_b)
{
    // M[0] = sample_a;
    // M[1] = sample_b;

    // if (power > 5.0f)
    // {
    //     RLS_Update(&RLS, M, power);
    //     power_control_instance->k1 = fmax(RLS.params_data[0], 1e-5f);  // In case the k1 diverge to negative number
    //     power_control_instance->k2 = fmax(RLS.params_data[1], 1e-5f);  // In case the k2 diverge to negative number
    // }
}
