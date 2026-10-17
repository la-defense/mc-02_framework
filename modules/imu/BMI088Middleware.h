/**
* @file BMI088Middleware.h
* @brief 保留旧版 BMI088 SPI 片选和单字节读写接口。
* 当前驱动位于 modules/BMI088；调用字节读写前须先给 BMI088_SPI 设置有效 SPI 句柄。
 */
#ifndef BMI088MIDDLEWARE_H
#define BMI088MIDDLEWARE_H

#include "main.h"

#define BMI088_USE_SPI
//#define BMI088_USE_IIC

/*
#define CS1_ACCEL_GPIO_Port ACCEL_NSS_GPIO_Port
#define CS1_ACCEL_Pin ACCEL_NSS_Pin
#define CS1_GYRO_GPIO_Port GYRO_NSS_GPIO_Port
#define CS1_GYRO_Pin GYRO_NSS_Pin
*/


#if defined(BMI088_USE_SPI)
/** @brief 拉低或拉高旧版加速度计的片选引脚。 */
extern void BMI088_ACCEL_NS_L(void);
extern void BMI088_ACCEL_NS_H(void);

/** @brief 拉低或拉高旧版陀螺仪的片选引脚。 */
extern void BMI088_GYRO_NS_L(void);
extern void BMI088_GYRO_NS_H(void);

/**
* @brief 通过 BMI088_SPI 同步交换一个字节。
* @param reg 要发送的寄存器地址或数据字节。
* @return HAL 收到的字节；本接口没有返回 HAL 传输状态。
* @note 调用前必须设置有效的 BMI088_SPI。
 */
extern uint8_t BMI088_read_write_byte(uint8_t reg);

/** @brief 由旧版驱动调用方在使用前设置的 SPI 句柄。 */
extern SPI_HandleTypeDef *BMI088_SPI;

#elif defined(BMI088_USE_IIC)

#endif

#endif
