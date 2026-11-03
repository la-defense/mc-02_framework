/** @file bsp_spi.h
 * @brief 逻辑 SPI 注册测试中使用的最小板级设备替身。
 */
#ifndef MC02_TEST_SPI_REGISTRATION_H
#define MC02_TEST_SPI_REGISTRATION_H
#include "ins_hardware.h"
typedef struct { int token; } SPIInstance;
typedef struct {
    SPI_HandleTypeDef *spi_handle;
    GPIO_TypeDef *GPIOx;
    uint16_t cs_pin;
    int spi_work_mode;
    void *id;
} SPI_Init_Config_s;
SPIInstance *SPIRegister(const SPI_Init_Config_s *config);
uint8_t SPIUnregister(SPIInstance *instance);
#endif
