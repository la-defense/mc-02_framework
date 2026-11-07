/** @file adc_calibration_test.c
 * @brief 运行生产 ADC 校准入口，注入硬件状态和期限变化。
 */
#include "bsp_adc_calibration.h"
#include "test_check.h"
#include <string.h>
unsigned char fake_adc_device;
ADC_HandleTypeDef hadc1;
static unsigned enabled, old_calibration, calibration_started, remaining_polls, check_count, check_limit;

uint32_t LL_ADC_IsEnabled(void *adc) { TEST_CHECK(adc == ADC1); return enabled; }
uint32_t LL_ADC_IsCalibrationOnGoing(void *adc)
{
    TEST_CHECK(adc == ADC1);
    if (!calibration_started) return old_calibration; // 启动前检查上次硬件状态。
    if (remaining_polls) { --remaining_polls; return 1u; } // 模拟硬件尚未结束。
    return 0u;
}
void LL_ADC_StartCalibration(void *adc, uint32_t mode, uint32_t single)
{
    TEST_CHECK(adc == ADC1 && mode == ADC_CALIB_OFFSET && single == ADC_SINGLE_ENDED);
    TEST_CHECK(hadc1.Lock == HAL_LOCKED && (hadc1.State & HAL_ADC_STATE_BUSY_INTERNAL));
    ++calibration_started; // 记录一次真正发给硬件的校准命令。
}
uint8_t PlatformPeripheralStartupCheck(void) { return (uint8_t)(++check_count <= check_limit); }

static void Reset(void)
{
    memset(&hadc1, 0, sizeof(hadc1));
    hadc1.Instance = ADC1;
    hadc1.State = HAL_ADC_STATE_READY;
    enabled = old_calibration = calibration_started = remaining_polls = check_count = 0u;
    check_limit = 10u;
}

int main(void)
{
    Reset(); hadc1.Instance = NULL;
    TEST_CHECK(BSP_ADCCalibrate() == HAL_ERROR && !calibration_started);
    Reset(); hadc1.Lock = HAL_LOCKED;
    TEST_CHECK(BSP_ADCCalibrate() == HAL_BUSY && !calibration_started);
    Reset(); hadc1.State = HAL_ADC_STATE_ERROR_INTERNAL;
    TEST_CHECK(BSP_ADCCalibrate() == HAL_ERROR && !calibration_started);
    Reset(); enabled = 1u;
    TEST_CHECK(BSP_ADCCalibrate() == HAL_ERROR && !calibration_started);
    Reset(); old_calibration = 1u;
    TEST_CHECK(BSP_ADCCalibrate() == HAL_ERROR && !calibration_started);
    Reset(); check_limit = 0u;
    TEST_CHECK(BSP_ADCCalibrate() == HAL_TIMEOUT && !calibration_started);
    Reset(); remaining_polls = 3u;
    TEST_CHECK(BSP_ADCCalibrate() == HAL_OK && calibration_started == 1u);
    TEST_CHECK(hadc1.Lock == HAL_UNLOCKED && hadc1.State == HAL_ADC_STATE_READY && check_count == 5u);
    Reset(); remaining_polls = 20u; check_limit = 3u;
    TEST_CHECK(BSP_ADCCalibrate() == HAL_TIMEOUT && calibration_started == 1u);
    TEST_CHECK(hadc1.Lock == HAL_UNLOCKED && (hadc1.State & HAL_ADC_STATE_ERROR_INTERNAL));
    TEST_CHECK(remaining_polls > 0u); // 硬件未完成时，软件仍按期限退出。
    Reset(); check_limit = 1u; // 硬件立即结束，但最终检查发生在期限外。
    TEST_CHECK(BSP_ADCCalibrate() == HAL_TIMEOUT && calibration_started == 1u);
    TEST_CHECK(hadc1.Lock == HAL_UNLOCKED && (hadc1.State & HAL_ADC_STATE_ERROR_INTERNAL));
    return 0;
}
