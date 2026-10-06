#include "imu_heater.h"
#include "controller.h"
#include "main.h"
#include "tim.h"
#include "bsp_adc.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>

static uint32_t now_ms;
static float vcc_in = 24.0f;
static uint32_t compare_value;
static BSP_ADC_Sample_t adc_sample = {
    .sequence = 1u,
    .timestamp_ms = 100u,
    .vcc_in_volts = 24.0f,
    .voltage_config_valid = 1u,
};
static BSP_ADC_Status_e adc_status = BSP_ADC_STATUS_VALID;
TIM_HandleTypeDef htim3;

static void fail(const char *message)
{
    fprintf(stderr, "%s\n", message);
    exit(EXIT_FAILURE);
}

uint32_t HAL_GetTick(void) { return now_ms; }
float BSP_ADCGetVccIn(void) { return vcc_in; }
BSP_ADC_Status_e BSP_ADCGetSample(BSP_ADC_Sample_t *sample)
{
    if (sample == NULL)
        return BSP_ADC_STATUS_INVALID_ARGUMENT;
    *sample = adc_sample;
    if (adc_status != BSP_ADC_STATUS_VALID)
        return adc_status;
    return ((uint32_t)(now_ms - sample->timestamp_ms) <= BSP_ADC_SAMPLE_MAX_AGE_MS)
               ? BSP_ADC_STATUS_VALID
               : BSP_ADC_STATUS_STALE;
}
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

    /* A cached 12V conversion cannot remain authoritative after its 20ms age
       limit, even when the physical supply changes and the IMU keeps updating. */
    adc_sample.vcc_in_volts = 12.1f;
    adc_sample.timestamp_ms = 100u;
    now_ms = 121u;
    sample.sequence = 2u;
    sample.timestamp_ms = 121u;
    IMUHeaterUpdate(&sample, 0u);
    if (compare_value != 0u) fail("stale cached VIN must force the production heater output off");

    /* A newly published 24V conversion can re-enter normal control. */
    adc_sample.vcc_in_volts = 24.2f;
    adc_sample.timestamp_ms = 122u;
    now_ms = 122u;
    sample.sequence = 3u;
    sample.timestamp_ms = 122u;
    IMUHeaterUpdate(&sample, 0u);
    if (compare_value == 0u) fail("a fresh VIN sample must permit normal control when the supply is valid");

    adc_status = BSP_ADC_STATUS_ERROR;
    now_ms = 123u;
    sample.sequence = 4u;
    sample.timestamp_ms = 123u;
    IMUHeaterUpdate(&sample, 0u);
    if (compare_value != 0u) fail("an ADC runtime error must force the heater output off");
    adc_status = BSP_ADC_STATUS_VALID;

    adc_sample.voltage_config_valid = 0u;
    adc_sample.timestamp_ms = 124u;
    now_ms = 124u;
    sample.sequence = 5u;
    sample.timestamp_ms = 124u;
    IMUHeaterUpdate(&sample, 0u);
    if (compare_value != 0u) fail("an invalid voltage reference/divider configuration must reject heating");
    adc_sample.voltage_config_valid = 1u;

    adc_sample.vcc_in_volts = 9.9f;
    adc_sample.timestamp_ms = 125u;
    now_ms = 125u;
    sample.sequence = 6u;
    sample.timestamp_ms = 125u;
    IMUHeaterUpdate(&sample, 0u);
    if (compare_value != 0u) fail("an out-of-range low supply must reject heating");

    adc_sample.vcc_in_volts = 24.2f;
    adc_sample.timestamp_ms = 127u;
    now_ms = 127u;
    sample.sequence = 7u;
    sample.timestamp_ms = 127u;
    IMUHeaterUpdate(&sample, 0u);
    if (compare_value == 0u) fail("fresh in-range voltage may resume normal control after a low supply");

    adc_sample.vcc_in_volts = 30.1f;
    adc_sample.timestamp_ms = 128u;
    now_ms = 128u;
    sample.sequence = 8u;
    sample.timestamp_ms = 128u;
    IMUHeaterUpdate(&sample, 0u);
    if (compare_value != 0u) fail("an out-of-range high supply must reject heating");
    IMUHeaterClearFault();

    adc_sample.vcc_in_volts = 24.2f;
    adc_sample.timestamp_ms = 129u;
    now_ms = 129u;
    sample.sequence = 9u;
    sample.timestamp_ms = 129u;
    IMUHeaterUpdate(&sample, 0u);
    if (compare_value == 0u) fail("valid in-range supply may resume after manual fault clear");

    if (!IMUHeaterBeginLongOperation(IMU_HEATER_LONG_OPERATION_CALIBRATION))
        fail("calibration should acquire the heater inhibit before blocking work");
    IMUHeaterUpdate(&sample, 0u);
    if (compare_value != 0u) fail("heater output must be forced off throughout calibration");
    IMUHeaterEndLongOperation(IMU_HEATER_LONG_OPERATION_CALIBRATION, 1u);
    IMUHeaterUpdate(&sample, 0u);
    if (compare_value != 0u) fail("successful maintenance must not restore PWM from the pre-operation sample");

    sample.sequence = 10u;
    sample.timestamp_ms = 130u;
    adc_sample.timestamp_ms = 130u;
    now_ms = 130u;
    IMUHeaterUpdate(&sample, 0u);
    if (compare_value == 0u) fail("a new fresh post-operation sample may re-enable normal heater control");

    now_ms = 130u;
    IMUHeaterUpdate(&sample, 0u);
    if (compare_value == 0u) fail("re-reading a still-fresh snapshot should not turn off before its age limit");

    now_ms = 136u;
    IMUHeaterUpdate(&sample, 0u);
    if (compare_value != 0u) fail("a snapshot older than 5ms must turn the production heater output off");
    IMUHeaterStatus_t status = {0};
    IMUHeaterGetStatus(&status);
    if (status.sensor_valid) fail("stale samples must be reported invalid");

    now_ms = 631u;
    IMUHeaterUpdate(&sample, 0u);
    IMUHeaterGetStatus(&status);
    if (!status.fault) fail("replaying one old sequence must not refresh the 500ms sensor timeout");

    sample.sequence = 11u;
    sample.timestamp_ms = 632u;
    now_ms = 632u;
    adc_sample.timestamp_ms = 632u;
    IMUHeaterUpdate(&sample, 0u);
    IMUHeaterGetStatus(&status);
    if (!status.fault || compare_value != 0u)
        fail("a new sample must not silently clear the latched sensor timeout");

    sample.sequence = 12u;
    sample.timestamp_ms = 633u;
    sample.temperature = NAN;
    now_ms = 633u;
    adc_sample.timestamp_ms = 633u;
    IMUHeaterUpdate(&sample, 0u);
    if (compare_value != 0u) fail("non-finite temperature must never enable heater output");

    if (!IMUHeaterBeginLongOperation(IMU_HEATER_LONG_OPERATION_FLASH))
        fail("flash operation should be able to acquire its independent heater inhibit");
    IMUHeaterEndLongOperation(IMU_HEATER_LONG_OPERATION_FLASH, 0u);
    sample.sequence = 13u;
    sample.timestamp_ms = 634u;
    sample.temperature = 20.0f;
    now_ms = 634u;
    adc_sample.timestamp_ms = 634u;
    IMUHeaterUpdate(&sample, 0u);
    if (compare_value != 0u) fail("failed flash work must leave the heater inhibited");
    return EXIT_SUCCESS;
}
