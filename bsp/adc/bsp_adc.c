#include "bsp_adc.h"
#include "adc.h"
#include "main.h"
#include "bsp_log.h"

#include <stddef.h>

/*
 * MC-02 电源电压采集:
 * VCC_IN -- R86 100K -- PC4(ADC1_INP4) -- R87 10K -- GND
 * 分压比 = 10K / (100K + 10K) = 1/11
 *
 * ADC1 配置为 scan + DMA circular:
 *   rank1 = ADC_CHANNEL_4  (PC4, VCC_IN)
 *   rank2 = ADC_CHANNEL_19 (PA5, 预留/其他采样)
 */

#define BSP_ADC_DMA_WORD_COUNT (BSP_ADC_DMA_BATCH_PAIR_COUNT * 2u)
#define BSP_ADC_DMA_HALF_WORD_COUNT (BSP_ADC_DMA_WORD_COUNT / 2u)
#define BSP_ADC_PAIRS_PER_HALF (BSP_ADC_DMA_HALF_WORD_COUNT / 2u)
#define BSP_ADC_CACHE_LINE_BYTES 32u

static uint16_t adc_dma_buffer[BSP_ADC_DMA_WORD_COUNT] __attribute__((aligned(BSP_ADC_CACHE_LINE_BYTES)));
static volatile BSP_ADC_Sample_t latest_sample;
static volatile uint32_t adc_error_flags = BSP_ADC_ERROR_NONE;
static volatile uint32_t adc_hal_error_code = 0u;
static volatile uint8_t adc_started = 0u;
static volatile uint8_t sample_ready = 0u;

static uint8_t BSP_ADCVoltageConfigValid(void)
{
    return (BSP_ADC_VREF_MV > 0u && BSP_ADC_VREF_MV <= 5000u &&
            BSP_ADC_FULL_SCALE == 65535u && BSP_ADC_VCC_IN_DIVIDER_MILLI > 0u &&
            BSP_ADC_VCC_IN_DIVIDER_MILLI <= 100000u)
               ? 1u
               : 0u;
}

static void BSP_ADCSetError(uint32_t error_flag, uint32_t hal_error_code)
{
    uint32_t previous_primask = __get_PRIMASK();
    __disable_irq();
    adc_error_flags |= error_flag;
    adc_hal_error_code = hal_error_code;
    __set_PRIMASK(previous_primask);
}

static void BSP_ADCResetPublishedSample(void)
{
    uint32_t previous_primask = __get_PRIMASK();
    __disable_irq();
    latest_sample.raw_vcc_in = 0u;
    latest_sample.raw_key = 0u;
    latest_sample.sequence = 0u;
    latest_sample.timestamp_ms = 0u;
    latest_sample.error_flags = BSP_ADC_ERROR_NONE;
    latest_sample.hal_error_code = 0u;
    latest_sample.vcc_in_volts = 0.0f;
    latest_sample.voltage_config_valid = 0u;
    adc_error_flags = BSP_ADC_ERROR_NONE;
    adc_hal_error_code = 0u;
    sample_ready = 0u;
    adc_started = 0u;
    __set_PRIMASK(previous_primask);
}

static void BSP_ADCProcessCompletedHalf(uint32_t first_word)
{
    if (!adc_started || adc_error_flags != BSP_ADC_ERROR_NONE)
        return;

    uint16_t *half_buffer = &adc_dma_buffer[first_word];
    SCB_InvalidateDCache_by_Addr((uint32_t *)half_buffer,
                                 (int32_t)(BSP_ADC_DMA_HALF_WORD_COUNT * sizeof(uint16_t)));

    uint32_t vcc_sum = 0u;
    uint32_t key_sum = 0u;
    for (uint32_t pair = 0u; pair < BSP_ADC_PAIRS_PER_HALF; ++pair)
    {
        vcc_sum += half_buffer[pair * 2u];
        key_sum += half_buffer[pair * 2u + 1u];
    }

    uint32_t previous_primask = __get_PRIMASK();
    __disable_irq();
    latest_sample.raw_vcc_in = (uint16_t)(vcc_sum / BSP_ADC_PAIRS_PER_HALF);
    latest_sample.raw_key = (uint16_t)(key_sum / BSP_ADC_PAIRS_PER_HALF);
    latest_sample.sequence++;
    latest_sample.timestamp_ms = HAL_GetTick();
    latest_sample.error_flags = adc_error_flags;
    latest_sample.hal_error_code = adc_hal_error_code;
    sample_ready = 1u;
    __set_PRIMASK(previous_primask);
}

void BSP_ADCInit(void)
{
    BSP_ADCResetPublishedSample();

    HAL_StatusTypeDef calibration_status =
        HAL_ADCEx_Calibration_Start(&hadc1, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED);
    if (calibration_status != HAL_OK)
    {
        BSP_ADCSetError(BSP_ADC_ERROR_CALIBRATION, (uint32_t)calibration_status);
        LOGERROR("[adc] calibration failed");
        return;
    }

    SCB_CleanInvalidateDCache_by_Addr((uint32_t *)adc_dma_buffer,
                                     (int32_t)(BSP_ADC_DMA_WORD_COUNT * sizeof(uint16_t)));

    HAL_StatusTypeDef dma_start_status =
        HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc_dma_buffer, BSP_ADC_DMA_WORD_COUNT);
    if (dma_start_status != HAL_OK)
    {
        BSP_ADCSetError(BSP_ADC_ERROR_DMA_START, (uint32_t)dma_start_status);
        LOGERROR("[adc] start DMA failed");
        return;
    }

    adc_started = 1u;
    LOGINFO("[adc] VCC_IN sense started (divider=1/11)");
}

BSP_ADC_Status_e BSP_ADCGetSample(BSP_ADC_Sample_t *sample)
{
    if (sample == NULL)
        return BSP_ADC_STATUS_INVALID_ARGUMENT;

    uint32_t previous_primask = __get_PRIMASK();
    __disable_irq();
    sample->raw_vcc_in = latest_sample.raw_vcc_in;
    sample->raw_key = latest_sample.raw_key;
    sample->sequence = latest_sample.sequence;
    sample->timestamp_ms = latest_sample.timestamp_ms;
    sample->error_flags = adc_error_flags;
    sample->hal_error_code = adc_hal_error_code;
    uint8_t started = adc_started;
    uint8_t ready = sample_ready;
    __set_PRIMASK(previous_primask);

    sample->voltage_config_valid = BSP_ADCVoltageConfigValid();
    sample->vcc_in_volts = 0.0f;
    if (sample->voltage_config_valid)
    {
        sample->vcc_in_volts = ((float)sample->raw_vcc_in * (float)BSP_ADC_VREF_MV *
                                (float)BSP_ADC_VCC_IN_DIVIDER_MILLI) /
                               ((float)BSP_ADC_FULL_SCALE * 1000000.0f);
    }

    if (sample->error_flags != BSP_ADC_ERROR_NONE)
        return BSP_ADC_STATUS_ERROR;
    if (!started || !ready)
        return BSP_ADC_STATUS_NOT_READY;
    if ((uint32_t)(HAL_GetTick() - sample->timestamp_ms) > BSP_ADC_SAMPLE_MAX_AGE_MS)
        return BSP_ADC_STATUS_STALE;
    return BSP_ADC_STATUS_VALID;
}

void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc == &hadc1)
        BSP_ADCProcessCompletedHalf(0u);
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc == &hadc1)
        BSP_ADCProcessCompletedHalf(BSP_ADC_DMA_HALF_WORD_COUNT);
}

void HAL_ADC_ErrorCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc != &hadc1)
        return;

    BSP_ADCSetError(BSP_ADC_ERROR_RUNTIME, hadc->ErrorCode);
}
