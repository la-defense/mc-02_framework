/**
 * @file i2c.h
 * @brief 提供 I²C 注册和可选驱动测试使用的最小 HAL 类型与函数声明。
 * @author MC-02 project contributors
 * @copyright Copyright (c) HNU YueLu EC 2022 all rights reserved
 */
#ifndef MC02_IIC_TEST_I2C_H
#define MC02_IIC_TEST_I2C_H

#include <stdint.h>

typedef enum {
    HAL_OK = 0,
    HAL_ERROR,
    HAL_BUSY,
    HAL_TIMEOUT
} HAL_StatusTypeDef;

typedef struct {
    uint16_t Devaddress;
} I2C_HandleTypeDef;

#define I2C_OTHER_AND_LAST_FRAME 0u
#define I2C_OTHER_FRAME 1u
#define I2C_MEMADD_SIZE_8BIT 1u
#define I2C_MEMADD_SIZE_16BIT 2u

uint32_t IICTestGetPrimask(void);
void IICTestDisableIrq(void);
void IICTestDmb(void);
void IICTestSetPrimask(uint32_t value);

#define __get_PRIMASK() IICTestGetPrimask()
#define __disable_irq() IICTestDisableIrq()
#define __DMB() IICTestDmb()
#define __set_PRIMASK(value) IICTestSetPrimask(value)

void HAL_Delay(uint32_t delay_ms);
HAL_StatusTypeDef HAL_I2C_Master_Transmit(I2C_HandleTypeDef *handle, uint16_t address,
                                         uint8_t *data, uint16_t size, uint32_t timeout);
HAL_StatusTypeDef HAL_I2C_Master_Receive(I2C_HandleTypeDef *handle, uint16_t address,
                                         uint8_t *data, uint16_t size, uint32_t timeout);
HAL_StatusTypeDef HAL_I2C_Master_Seq_Transmit_IT(I2C_HandleTypeDef *handle, uint16_t address,
                                                 uint8_t *data, uint16_t size, uint32_t options);
HAL_StatusTypeDef HAL_I2C_Master_Seq_Transmit_DMA(I2C_HandleTypeDef *handle, uint16_t address,
                                                  uint8_t *data, uint16_t size, uint32_t options);
HAL_StatusTypeDef HAL_I2C_Master_Seq_Receive_IT(I2C_HandleTypeDef *handle, uint16_t address,
                                                uint8_t *data, uint16_t size, uint32_t options);
HAL_StatusTypeDef HAL_I2C_Master_Seq_Receive_DMA(I2C_HandleTypeDef *handle, uint16_t address,
                                                 uint8_t *data, uint16_t size, uint32_t options);
HAL_StatusTypeDef HAL_I2C_Mem_Write(I2C_HandleTypeDef *handle, uint16_t address,
                                    uint16_t register_address, uint16_t register_size,
                                    uint8_t *data, uint16_t size, uint32_t timeout);
HAL_StatusTypeDef HAL_I2C_Mem_Read(I2C_HandleTypeDef *handle, uint16_t address,
                                   uint16_t register_address, uint16_t register_size,
                                   uint8_t *data, uint16_t size, uint32_t timeout);

#endif
