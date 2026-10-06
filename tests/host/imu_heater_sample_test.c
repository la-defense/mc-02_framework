#include "imu_heater.h"
#include "controller.h"
#include "main.h"
#include "tim.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>

static uint32_t now_ms;
static float vcc_in = 24.0f;
static uint32_t compare_value;
TIM_HandleTypeDef htim3;

static void fail(const char *message)
{
    fprintf(stderr, "%s\n", message);
    exit(EXIT_FAILURE);
}

uint32_t HAL_GetTick(void) { return now_ms; }
float BSP_ADCGetVccIn(void) { return vcc_in; }
void HostSetCompare(TIM_HandleTypeDef *timer, uint32_t channel, uint32_t value)
{ (void)channel; timer->compare = value; compare_value = value; }
int HAL_TIM_PWM_Start(TIM_HandleTypeDef *timer, uint32_t channel)
{ (void)timer; (void)channel; return 0; }
void PIDInit(PIDInstance *pid, PID_Init_Config_s *config)
{ pid->Kp = config->Kp; pid->MaxOut = config->MaxOut; }
float PIDCalculate(PIDInstance *pid, float measure, float ref)
{
    float output = (ref - measure) * pid->Kp;
    if (output > pid->MaxOut) output = pid->MaxOut;
    pid->Last_Output = pid->Output;
    pid->Output = output;
    return output;
}

int main(void)
{
    now_ms = 100u;
    IMUHeaterInit();
    BMI088_Data_t sample = {
        .temperature = 20.0f,
        .sequence = 1u,
        .timestamp_ms = 100u,
        .valid = 1u,
    };

    IMUHeaterUpdate(&sample, 0u);
    if (compare_value == 0u) fail("a fresh valid sample should exercise the enabled heater control path");

    now_ms = 104u;
    IMUHeaterUpdate(&sample, 0u);
    if (compare_value == 0u) fail("re-reading a still-fresh snapshot should not turn off before its age limit");

    now_ms = 106u;
    IMUHeaterUpdate(&sample, 0u);
    if (compare_value != 0u) fail("a snapshot older than 5ms must turn the production heater output off");
    IMUHeaterStatus_t status = {0};
    IMUHeaterGetStatus(&status);
    if (status.sensor_valid) fail("stale samples must be reported invalid");

    now_ms = 601u;
    IMUHeaterUpdate(&sample, 0u);
    IMUHeaterGetStatus(&status);
    if (!status.fault) fail("replaying one old sequence must not refresh the 500ms sensor timeout");

    sample.sequence = 2u;
    sample.timestamp_ms = 602u;
    now_ms = 602u;
    IMUHeaterUpdate(&sample, 0u);
    IMUHeaterGetStatus(&status);
    if (!status.fault || compare_value != 0u)
        fail("a new sample must not silently clear the latched sensor timeout");

    sample.sequence = 3u;
    sample.timestamp_ms = 603u;
    sample.temperature = NAN;
    now_ms = 603u;
    IMUHeaterUpdate(&sample, 0u);
    if (compare_value != 0u) fail("non-finite temperature must never enable heater output");
    return EXIT_SUCCESS;
}
