/** @file BMI088Middleware.c
* @brief 实现旧版 BMI088 SPI 片选控制和单字节收发。
* 该接口保留给旧代码使用，当前传感器驱动见 modules/BMI088。
*/
#include "BMI088Middleware.h"
#include "main.h"

SPI_HandleTypeDef *BMI088_SPI;

void BMI088_ACCEL_NS_L(void)
{
    // SPI 事务开始前选中加速度计。
    HAL_GPIO_WritePin(CS2_ACCEL_GPIO_Port, CS2_ACCEL_Pin, GPIO_PIN_RESET);
}
void BMI088_ACCEL_NS_H(void)
{
    // SPI 事务结束后取消选择加速度计。
    HAL_GPIO_WritePin(CS2_ACCEL_GPIO_Port, CS2_ACCEL_Pin, GPIO_PIN_SET);
}

void BMI088_GYRO_NS_L(void)
{
    // SPI 事务开始前选中陀螺仪。
    HAL_GPIO_WritePin(CS2_GYRO_GPIO_Port, CS2_GYRO_Pin, GPIO_PIN_RESET);
}
void BMI088_GYRO_NS_H(void)
{
    // SPI 事务结束后取消选择陀螺仪。
    HAL_GPIO_WritePin(CS2_GYRO_GPIO_Port, CS2_GYRO_Pin, GPIO_PIN_SET);
}

uint8_t BMI088_read_write_byte(uint8_t txdata)
{
    // 阻塞交换一个字节；HAL 状态未向调用者返回。
    uint8_t rx_data;
    HAL_SPI_TransmitReceive(BMI088_SPI, &txdata, &rx_data, 1, 1000);
    return rx_data;
}
