/** @file platform_adc_stm32.c
 * @brief 将逻辑电源输入映射到现有 ADC 样本读取接口。
 */
#include "platform_analog.h"
#include "bsp_adc.h"
#include <math.h>
#include <stddef.h>
#if defined(MC02_CONFIG_GENERATED)
#include "mc02_build_config.h"
#endif

uint8_t PlatformAdcIsBound(PlatformAdcChannel_t channel)
{
#if defined(MC02_CONFIG_GENERATED) && !MC02_FEATURE_ADC
    (void)channel;
    return 0u; // 当前板未选择 ADC 时不声明可用输入。
#else
    return (uint8_t)(channel == PLATFORM_ADC_VCC_IN);
#endif
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
