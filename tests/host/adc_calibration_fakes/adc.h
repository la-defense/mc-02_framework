/** @file adc.h
 * @brief 供 ADC 校准期限测试使用的 HAL 最小替身。
 */
#pragma once
#include <stdint.h>
typedef enum { HAL_OK = 0, HAL_ERROR = 1, HAL_BUSY = 2, HAL_TIMEOUT = 3 } HAL_StatusTypeDef;
typedef struct { void *Instance; uint32_t State, Lock; } ADC_HandleTypeDef;
extern unsigned char fake_adc_device;
extern ADC_HandleTypeDef hadc1;
#define ADC1 (&fake_adc_device)
#define HAL_UNLOCKED 0u
#define HAL_LOCKED 1u
#define HAL_ADC_STATE_READY (1u << 0)
#define HAL_ADC_STATE_BUSY_INTERNAL (1u << 1)
#define HAL_ADC_STATE_REG_BUSY (1u << 2)
#define HAL_ADC_STATE_INJ_BUSY (1u << 3)
#define HAL_ADC_STATE_ERROR_INTERNAL (1u << 4)
#define ADC_CALIB_OFFSET 0u
#define ADC_SINGLE_ENDED 0u
#define __HAL_LOCK(handle) do { if ((handle)->Lock == HAL_LOCKED) return HAL_BUSY; (handle)->Lock = HAL_LOCKED; } while (0)
#define __HAL_UNLOCK(handle) ((handle)->Lock = HAL_UNLOCKED)
#define ADC_STATE_CLR_SET(state, clear, set) ((state) = ((state) & ~(clear)) | (set))
