/** @file bsp_adc_calibration.c
 * @brief 使用固定启动期限等待 H723 ADC1 单端偏移校准。
 */
#include "bsp_adc_calibration.h"
#include "platform_clock_startup.h"
#include "stm32h7xx_ll_adc.h"

HAL_StatusTypeDef BSP_ADCCalibrate(void)
{
    if (hadc1.Instance != ADC1) return HAL_ERROR; // 板级准备失败时不能访问 ADC 寄存器。
    if (hadc1.Lock != HAL_UNLOCKED) return HAL_BUSY; // 其他操作正在使用句柄时不能覆盖其状态。
    if (!(hadc1.State & HAL_ADC_STATE_READY) ||
        (hadc1.State & (HAL_ADC_STATE_BUSY_INTERNAL | HAL_ADC_STATE_REG_BUSY | HAL_ADC_STATE_INJ_BUSY |
                        HAL_ADC_STATE_ERROR_INTERNAL))) return HAL_ERROR; // 只有初始化完成且没有忙碌或内部错误时才能校准。
    if (LL_ADC_IsEnabled(hadc1.Instance) || LL_ADC_IsCalibrationOnGoing(hadc1.Instance))
        return HAL_ERROR; // 启动校准要求 ADC 已禁用，且没有旧校准在运行。
    if (!PlatformPeripheralStartupCheck()) return HAL_TIMEOUT; // 已超期时不能开始新的硬件动作。
    __HAL_LOCK(&hadc1); // 与 HAL 的 ADC 状态机使用同一把锁。
    ADC_STATE_CLR_SET(hadc1.State, HAL_ADC_STATE_REG_BUSY | HAL_ADC_STATE_INJ_BUSY,
                      HAL_ADC_STATE_BUSY_INTERNAL); // 标记正在校准，禁止当作就绪使用。
    LL_ADC_StartCalibration(hadc1.Instance, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED); // 沿用原有单端偏移校准。
    while (LL_ADC_IsCalibrationOnGoing(hadc1.Instance)) // 等硬件结束，而不是等待固定次数。
    {
        if (!PlatformPeripheralStartupCheck()) // 使用整个外设启动阶段的原始起点。
        {
            ADC_STATE_CLR_SET(hadc1.State, HAL_ADC_STATE_BUSY_INTERNAL, HAL_ADC_STATE_ERROR_INTERNAL); // 超期不能报告就绪。
            __HAL_UNLOCK(&hadc1); // 与 HAL 失败路径一样释放软件锁。
            return HAL_TIMEOUT; // 硬件可能仍在校准；外层将关断并复位。
        }
    }
    if (!PlatformPeripheralStartupCheck()) // 硬件刚完成时也不能接受期限外的结果。
    {
        ADC_STATE_CLR_SET(hadc1.State, HAL_ADC_STATE_BUSY_INTERNAL, HAL_ADC_STATE_ERROR_INTERNAL); // 已过期的结果不能设为就绪。
        __HAL_UNLOCK(&hadc1); // 保持后续错误处理可读取句柄。
        return HAL_TIMEOUT;
    }
    ADC_STATE_CLR_SET(hadc1.State, HAL_ADC_STATE_BUSY_INTERNAL, HAL_ADC_STATE_READY); // 保留 HAL 原有成功状态。
    __HAL_UNLOCK(&hadc1); // 校准结束后允许 DMA 启动。
    return HAL_OK;
}
