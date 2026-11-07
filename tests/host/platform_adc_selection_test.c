/** @file platform_adc_selection_test.c
 * @brief 验证未选择 ADC 时不暴露绑定，也不会读取采样。
 */
#include "test_check.h"
#include "platform_analog.h"
#include "bsp_adc.h"

static unsigned sample_calls;
BSP_ADC_Status_e BSP_ADCGetSample(BSP_ADC_Sample_t *sample)
{
    ++sample_calls; // 记录真实适配器是否访问采样。
    sample->voltage_config_valid = 1u; // 模拟换算配置有效。
    sample->vcc_in_volts = 12.0f; // 固定测试样本单位为 V。
    return BSP_ADC_STATUS_VALID;
}

int main(void)
{
    float volts = -1.0f; // 失败时用于检查调用者的旧值。
    TEST_CHECK(PlatformAdcIsBound(PLATFORM_ADC_VCC_IN) == TEST_ADC_SELECTED);
    TEST_CHECK(PlatformAdcReadVolts(PLATFORM_ADC_VCC_IN, &volts) == TEST_ADC_SELECTED);
    TEST_CHECK(sample_calls == TEST_ADC_SELECTED);
    TEST_CHECK(volts == (TEST_ADC_SELECTED ? 12.0f : -1.0f));
    TEST_CHECK(!PlatformAdcReadVolts(PLATFORM_ADC_VCC_IN, NULL));
    return 0;
}
