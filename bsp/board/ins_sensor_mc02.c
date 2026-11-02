/** @file ins_sensor_mc02.c
 * @brief 将 INS 样本入口绑定到 MC-02 板载 BMI088 与 SPI2。
 */
#include "../../modules/imu/ins_sensor_port.h"
#include "bmi088.h"
#include "spi.h"
#include <stddef.h>

uint8_t InsSensorIsBound(PlatformImuPort_t port)
{
    return (uint8_t)(port == PLATFORM_IMU_BOARD_1); // 当前板卡只声明板载 BMI088。
}

InsSensor *InsSensorRegister(PlatformImuPort_t port)
{
    if (!InsSensorIsBound(port)) return NULL; // 未绑定时不申请 SPI 资源或启动标定。
    BMI088_Init_Config_s config = {
        .work_mode = BMI088_BLOCK_PERIODIC_MODE, // 沿用任务周期内阻塞采样。
        .cali_mode = BMI088_CALIBRATE_ONLINE_MODE, // 启动时沿用在线标定。
        .spi_gyro_config = {
            .spi_handle = &hspi2, // 陀螺使用板载 SPI2。
            .GPIOx = CS2_GYRO_GPIO_Port, // 陀螺片选所在端口。
            .cs_pin = CS2_GYRO_Pin, // 陀螺片选引脚。
            .spi_work_mode = SPI_BLOCK_MODE, // 采样沿用阻塞传输。
        },
        .spi_acc_config = {
            .spi_handle = &hspi2, // 加速度计与陀螺共享 SPI2。
            .GPIOx = CS2_ACCEL_GPIO_Port, // 加速度计片选所在端口。
            .cs_pin = CS2_ACCEL_Pin, // 加速度计片选引脚。
            .spi_work_mode = SPI_BLOCK_MODE, // 采样沿用阻塞传输。
        },
        // 加热 PWM 仍由温控模块单独管理，注册时不配置。
    };
    return BMI088Register(&config); // 保留原有注册失败和资源处理。
}

uint8_t InsSensorAcquire(InsSensor *sensor, BMI088_Data_t *sample)
{
    return (uint8_t)(BMI088Acquire((BMI088Instance *)sensor, sample) == BMI088_ACQUIRE_OK);
}

uint8_t InsSensorCalibrationService(void)
{
    return BMI088CalibService();
}

void InsSensorGetInitDiagnostics(uint32_t *error, uint32_t *retries)
{
    if (error != NULL) *error = bmi088_init_error;
    if (retries != NULL) *retries = bmi088_init_retry_count;
}
