/** @file main.h
 * @brief OLED主机测试只替换I2C发送；不连接硬件，不替换绘图和字模。
 */
#ifndef MC02_OLED_TEST_MAIN_H
#define MC02_OLED_TEST_MAIN_H

#include <stdint.h>

typedef struct { unsigned unused; } I2C_HandleTypeDef;
typedef enum { HAL_OK = 0, HAL_ERROR = 1 } HAL_StatusTypeDef;
HAL_StatusTypeDef HAL_I2C_Master_Transmit(I2C_HandleTypeDef *handle, uint16_t address,
                                        uint8_t *data, uint16_t length, uint32_t timeout);
#endif
