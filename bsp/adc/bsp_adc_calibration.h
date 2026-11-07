/** @file bsp_adc_calibration.h
 * @brief 在启动期限内完成板载 ADC 的单端偏移校准。
 */
#ifndef MC02_BSP_ADC_CALIBRATION_H
#define MC02_BSP_ADC_CALIBRATION_H
#include "adc.h"

/** @brief 校准已初始化、尚未启用的 ADC1。
 * @return 成功返回 HAL_OK；设备忙、状态无效或总期限用尽时返回相应 HAL 状态。
 * @note 只在启动线程的外设期限保护内调用；超期后 ADC 可能仍在硬件校准，不能启动采样。
 */
HAL_StatusTypeDef BSP_ADCCalibrate(void);
#endif
