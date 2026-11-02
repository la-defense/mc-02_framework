/** @file platform_adc_stm32.c
 * @brief 将逻辑电源输入映射到现有 ADC 样本读取接口。
 */
#include "platform_analog.h"
#include "bsp_adc.h"
#include <math.h>
#include <stddef.h>

uint8_t PlatformAdcIsBound(PlatformAdcChannel_t channel)
{
    return (uint8_t)(channel == PLATFORM_ADC_VCC_IN);
}

uint8_t PlatformAdcReadVolts(PlatformAdcChannel_t channel, float *volts)
{
    if (!PlatformAdcIsBound(channel) || volts == NULL) return 0u; // 非法参数不能改写输出。
    BSP_ADC_Sample_t sample;
    if (BSP_ADCGetSample(&sample) != BSP_ADC_STATUS_VALID || !sample.voltage_config_valid ||
        !isfinite(sample.vcc_in_volts))
        return 0u; // 样本无效或换算不可用时不交付电压。
    *volts = sample.vcc_in_volts; // 保留原 ADC 换算结果，调用者继续检查有限值和量程。
    return 1u;
}
