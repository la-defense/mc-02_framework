#include "bsp_adc.h"
#include "adc.h"
#include "main.h"
#include "bsp_log.h"

/*
 * MC-02 电源电压采集:
 * VCC_IN -- R86 100K -- PC4(ADC1_INP4) -- R87 10K -- GND
 * 分压比 = 10K / (100K + 10K) = 1/11
 *
 * ADC1 配置为 scan + DMA circular:
 *   rank1 = ADC_CHANNEL_4  (PC4, VCC_IN)
 *   rank2 = ADC_CHANNEL_19 (PA5, 预留/其他采样)
 */

#define BSP_ADC_VREF 3.3f
#define BSP_ADC_FULL_SCALE 65535.0f
#define BSP_ADC_VCC_IN_DIVIDER 11.0f

static uint16_t adc_dma_buffer[2] = {0};
static uint8_t adc_started = 0;

void BSP_ADCInit(void)
{
    if (HAL_ADCEx_Calibration_Start(&hadc1, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED) != HAL_OK)
    {
        LOGERROR("[adc] calibration failed");
    }

    if (HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc_dma_buffer, 2) != HAL_OK)
    {
        LOGERROR("[adc] start DMA failed");
        return;
    }

    adc_started = 1;
    LOGINFO("[adc] VCC_IN sense started (divider=1/11)");
}

uint16_t BSP_ADCGetRawVccIn(void)
{
    if (!adc_started)
        return 0;
    return adc_dma_buffer[0];
}

float BSP_ADCGetVccIn(void)
{
    if (!adc_started)
        return 0.0f;

    float pin_voltage = (float)adc_dma_buffer[0] * (BSP_ADC_VREF / BSP_ADC_FULL_SCALE);
    return pin_voltage * BSP_ADC_VCC_IN_DIVIDER;
}
