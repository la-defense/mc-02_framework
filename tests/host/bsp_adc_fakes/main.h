/**
 * @file main.h
 * @brief 为主机 ADC 测试替换 STM32 主头文件中的 HAL 与内核接口。
 * @note 仅由 tests/host/bsp_adc_snapshot_test.c 使用，不进入固件目标。
 */
#ifndef MC02_HOST_ADC_FAKE_MAIN_H
#define MC02_HOST_ADC_FAKE_MAIN_H

#include <stdint.h>

typedef enum
{
    HAL_OK = 0,
    HAL_ERROR = 1
} HAL_StatusTypeDef;

uint32_t HAL_GetTick(void);
uint32_t __get_PRIMASK(void);
void __disable_irq(void);
void __set_PRIMASK(uint32_t value);
static inline void __DMB(void) {} // 主机没有 SWD，版本顺序由接口测试检查。
void SCB_CleanInvalidateDCache_by_Addr(uint32_t *address, int32_t size);
void SCB_InvalidateDCache_by_Addr(uint32_t *address, int32_t size);

#endif
