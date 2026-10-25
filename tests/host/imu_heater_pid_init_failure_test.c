/** @file imu_heater_pid_init_failure_test.c
 * @brief 在 PID 初始化返回失败时，检查生产温控保留故障且不启动 PWM。
 * 链接替换仅用于注入失败，采样和 PWM 操作均由主机替身承担。
 */
#include "imu_heater.h"
#include "controller.h"
#include "main.h"
#include "tim.h"
#include "bsp_adc.h"
#include "test_check.h"

TIM_HandleTypeDef htim3;
static uint32_t primask;
static unsigned pwm_starts;
uint32_t HAL_GetTick(void) { return 100u; }
uint32_t __get_PRIMASK(void) { return primask; }
void __disable_irq(void) { primask = 1u; }
void __set_PRIMASK(uint32_t value) { primask = value; }
BSP_ADC_Status_e BSP_ADCGetSample(BSP_ADC_Sample_t *sample)
{
    *sample = (BSP_ADC_Sample_t){.sequence = 1u, .timestamp_ms = 100u,
        .vcc_in_volts = 12.5f, .voltage_config_valid = 1u};
    return BSP_ADC_STATUS_VALID;
}
void HostSetCompare(TIM_HandleTypeDef *timer, uint32_t channel, uint32_t value)
{ (void)channel; timer->compare = value; }
int HAL_TIM_PWM_Start(TIM_HandleTypeDef *timer, uint32_t channel)
{ (void)timer; (void)channel; ++pwm_starts; return 0; }
PID_Status_e __wrap_PIDInit(PIDInstance *pid, const PID_Init_Config_s *config)
{ (void)pid; (void)config; return PID_BAD_CONFIG; }

/** @brief 初始化失败后，正常温度也不能产生加热输出。 */
int main(void)
{
    IMUHeaterInit();
    BMI088_Data_t sample = {.sequence = 1u, .timestamp_ms = 100u, .valid = 1u, .temperature = 20.0f};
    IMUHeaterUpdate(&sample, 0u);
    IMUHeaterStatus_t status;
    IMUHeaterGetStatus(&status);
    TEST_CHECK(status.fault == 1u);
    TEST_CHECK(status.duty == 0u && htim3.compare == 0u);
    TEST_CHECK(pwm_starts == 0u);
    return EXIT_SUCCESS;
}
