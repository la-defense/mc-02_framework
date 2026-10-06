#include "adc.h"
#include "bsp_adc.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#define CHECK(condition)                                                         \
    do                                                                           \
    {                                                                            \
        if (!(condition))                                                        \
        {                                                                        \
            fprintf(stderr, "FAIL:%d: %s\n", __LINE__, #condition);             \
            return EXIT_FAILURE;                                                \
        }                                                                        \
    } while (0)

#define EXPECTED_DMA_WORD_COUNT 64u
#define EXPECTED_DMA_HALF_WORD_COUNT (EXPECTED_DMA_WORD_COUNT / 2u)
#define EXPECTED_PAIR_COUNT (EXPECTED_DMA_HALF_WORD_COUNT / 2u)

ADC_HandleTypeDef hadc1 = {.Instance = ADC1};
static HAL_StatusTypeDef calibration_status = HAL_OK;
static HAL_StatusTypeDef dma_start_status = HAL_OK;
static uint16_t *dma_words;
static uint32_t dma_word_count;
static uint32_t calibration_call_count;
static uint32_t dma_start_call_count;
static uint32_t now_ms;
static uint32_t primask;
static uint32_t cache_prepare_count;
static uintptr_t last_prepared_address;
static int32_t last_prepared_size;
static uint32_t dma_start_cache_order_error_count;
static uint32_t cache_invalidate_count;
static uintptr_t last_invalidated_address;
static int32_t last_invalidated_size;

uint32_t HAL_GetTick(void) { return now_ms; }
uint32_t __get_PRIMASK(void) { return primask; }
void __disable_irq(void) { primask = 1u; }
void __set_PRIMASK(uint32_t value) { primask = value; }

void SCB_CleanInvalidateDCache_by_Addr(uint32_t *address, int32_t size)
{
    cache_prepare_count++;
    last_prepared_address = (uintptr_t)address;
    last_prepared_size = size;
}

void SCB_InvalidateDCache_by_Addr(uint32_t *address, int32_t size)
{
    cache_invalidate_count++;
    last_invalidated_address = (uintptr_t)address;
    last_invalidated_size = size;
}

HAL_StatusTypeDef HAL_ADCEx_Calibration_Start(ADC_HandleTypeDef *hadc, uint32_t calibration_mode,
                                             uint32_t single_diff)
{
    (void)hadc;
    (void)calibration_mode;
    (void)single_diff;
    calibration_call_count++;
    return calibration_status;
}

HAL_StatusTypeDef HAL_ADC_Start_DMA(ADC_HandleTypeDef *hadc, uint32_t *data, uint32_t length)
{
    (void)hadc;
    if (cache_prepare_count <= dma_start_call_count)
        dma_start_cache_order_error_count++;
    dma_start_call_count++;
    dma_words = (uint16_t *)data;
    dma_word_count = length;
    return dma_start_status;
}

static void fill_pairs(uint32_t start_word, uint16_t raw_vcc, uint16_t raw_key)
{
    for (uint32_t pair = 0u; pair < EXPECTED_PAIR_COUNT; ++pair)
    {
        dma_words[start_word + pair * 2u] = raw_vcc;
        dma_words[start_word + pair * 2u + 1u] = raw_key;
    }
}

#ifdef BSP_ADC_EXPECT_INVALID_CONFIG
int main(void)
{
    BSP_ADC_Sample_t sample = {0};
    BSP_ADCInit();
    CHECK(dma_word_count == EXPECTED_DMA_WORD_COUNT);
    fill_pairs(0u, 21845u, 0u);
    HAL_ADC_ConvHalfCpltCallback(&hadc1);
    CHECK(BSP_ADCGetSample(&sample) == BSP_ADC_STATUS_VALID);
    CHECK(sample.sequence == 1u);
    CHECK(sample.error_flags == 0u);
    CHECK(sample.voltage_config_valid == 0u);
    CHECK(sample.vcc_in_volts == 0.0f);
    return EXIT_SUCCESS;
}
#else
int main(void)
{
    BSP_ADC_Sample_t sample = {0};
    CHECK(BSP_ADCGetSample(NULL) == BSP_ADC_STATUS_INVALID_ARGUMENT);
    CHECK(BSP_ADCGetSample(&sample) == BSP_ADC_STATUS_NOT_READY);

    calibration_status = HAL_ERROR;
    BSP_ADCInit();
    CHECK(calibration_call_count == 1u);
    CHECK(dma_start_call_count == 0u);
    CHECK(BSP_ADCGetSample(&sample) == BSP_ADC_STATUS_ERROR);
    CHECK((sample.error_flags & BSP_ADC_ERROR_CALIBRATION) != 0u);
    CHECK(sample.hal_error_code == (uint32_t)HAL_ERROR);

    calibration_status = HAL_OK;
    dma_start_status = HAL_ERROR;
    BSP_ADCInit();
    CHECK(dma_start_call_count == 1u);
    CHECK(dma_start_cache_order_error_count == 0u);
    CHECK((last_prepared_address % 32u) == 0u);
    CHECK(last_prepared_size == (int32_t)(EXPECTED_DMA_WORD_COUNT * sizeof(uint16_t)));
    CHECK(BSP_ADCGetSample(&sample) == BSP_ADC_STATUS_ERROR);
    CHECK((sample.error_flags & BSP_ADC_ERROR_DMA_START) != 0u);
    CHECK(sample.hal_error_code == (uint32_t)HAL_ERROR);

    dma_start_status = HAL_OK;
    now_ms = 100u;
    BSP_ADCInit();
    CHECK(dma_word_count == EXPECTED_DMA_WORD_COUNT);
    CHECK(cache_prepare_count == 2u);
    CHECK(dma_start_cache_order_error_count == 0u);
    CHECK(BSP_ADCGetSample(&sample) == BSP_ADC_STATUS_NOT_READY);

    fill_pairs(0u, 21845u, 0u);
    HAL_ADC_ConvHalfCpltCallback(&hadc1);
    CHECK(cache_invalidate_count == 1u);
    CHECK((last_invalidated_address % 32u) == 0u);
    CHECK(last_invalidated_size == (int32_t)(EXPECTED_DMA_HALF_WORD_COUNT * sizeof(uint16_t)));
    CHECK(BSP_ADCGetSample(&sample) == BSP_ADC_STATUS_VALID);
    CHECK(sample.raw_vcc_in == 21845u);
    CHECK(sample.raw_key == 0u);
    CHECK(sample.sequence == 1u && sample.timestamp_ms == 100u);
    CHECK(fabsf(sample.vcc_in_volts - 12.1f) < 0.01f);

    now_ms = 120u;
    CHECK(BSP_ADCGetSample(&sample) == BSP_ADC_STATUS_VALID);
    now_ms = 121u;
    CHECK(BSP_ADCGetSample(&sample) == BSP_ADC_STATUS_STALE);
    CHECK(sample.sequence == 1u);
    CHECK(fabsf(sample.vcc_in_volts - 12.1f) < 0.01f);

    fill_pairs(EXPECTED_DMA_HALF_WORD_COUNT, 43690u, 32000u);
    now_ms = 122u;
    HAL_ADC_ConvCpltCallback(&hadc1);
    CHECK(cache_invalidate_count == 2u);
    CHECK(last_invalidated_size == (int32_t)(EXPECTED_DMA_HALF_WORD_COUNT * sizeof(uint16_t)));
    CHECK(BSP_ADCGetSample(&sample) == BSP_ADC_STATUS_VALID);
    CHECK(sample.raw_vcc_in == 43690u && sample.raw_key == 32000u);
    CHECK(sample.sequence == 2u && sample.timestamp_ms == 122u);
    CHECK(fabsf(sample.vcc_in_volts - 24.2f) < 0.01f);

    ADC_HandleTypeDef other_adc = {.Instance = (void *)(uintptr_t)2u};
    other_adc.ErrorCode = 7u;
    HAL_ADC_ErrorCallback(&other_adc);
    CHECK(BSP_ADCGetSample(&sample) == BSP_ADC_STATUS_VALID);

    hadc1.ErrorCode = 0x35u;
    HAL_ADC_ErrorCallback(&hadc1);
    CHECK(BSP_ADCGetSample(&sample) == BSP_ADC_STATUS_ERROR);
    CHECK((sample.error_flags & BSP_ADC_ERROR_RUNTIME) != 0u);
    CHECK(sample.hal_error_code == 0x35u);
    CHECK(sample.sequence == 2u);

    hadc1.ErrorCode = 0u;
    now_ms = UINT32_MAX - 10u;
    BSP_ADCInit();
    fill_pairs(0u, 21845u, 1234u);
    HAL_ADC_ConvHalfCpltCallback(&hadc1);
    now_ms = 9u;
    CHECK(BSP_ADCGetSample(&sample) == BSP_ADC_STATUS_VALID);
    now_ms = 10u;
    CHECK(BSP_ADCGetSample(&sample) == BSP_ADC_STATUS_STALE);

    puts("PASS: ADC snapshots publish complete DMA batches and reject stale/error data");
    return EXIT_SUCCESS;
}
#endif
