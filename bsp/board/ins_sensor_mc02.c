/** @file ins_sensor_mc02.c
 * @brief 将 INS 样本入口绑定到 MC-02 板载 BMI088 与 SPI2。
 */
#include "../../modules/imu/ins_sensor_port.h"
#include "bmi088.h"
#include "board_spi_stm32.h"
#include "board_gpio_stm32.h"
#include <stddef.h>

typedef struct {
    SPI_HandleTypeDef *bus; // 两个传感器芯片共享的 SPI 控制器。
    BoardGpioBinding_t gyro_cs; // 陀螺片选。
    BoardGpioBinding_t accel_cs; // 加速度计片选。
} InsBoardBinding;

// 将设备编号对应的逻辑总线和片选转换为本板资源，不操作硬件。
static uint8_t ResolveInsBinding(PlatformImuPort_t port, InsBoardBinding *output)
{
    if (port != PLATFORM_IMU_BOARD_1 || output == NULL) return 0u; // 未声明的设备不启动。
    InsBoardBinding candidate = {0}; // 仅在全部资源存在时公布结果。
    candidate.bus = BoardSpiResolve(PLATFORM_SPI_BUS_2); // 板载 IMU 使用逻辑 SPI2。
    if (candidate.bus == NULL ||
        !BoardGpioResolve(PLATFORM_GPIO_IMU_GYRO_CS, &candidate.gyro_cs) ||
        !BoardGpioResolve(PLATFORM_GPIO_IMU_ACCEL_CS, &candidate.accel_cs))
        return 0u; // 缺少任一资源时不进入驱动注册。
    *output = candidate; // 资源完整后一次交付。
    return 1u;
}

uint8_t InsSensorIsBound(PlatformImuPort_t port)
{
    InsBoardBinding binding;
    return ResolveInsBinding(port, &binding);
}

InsSensor *InsSensorRegister(PlatformImuPort_t port)
{
    InsBoardBinding binding;
    if (!ResolveInsBinding(port, &binding)) return NULL; // 未绑定时不申请 SPI 资源或启动标定。
    BMI088_Init_Config_s config = {
        .work_mode = BMI088_BLOCK_PERIODIC_MODE, // 沿用任务周期内阻塞采样。
        .cali_mode = BMI088_CALIBRATE_ONLINE_MODE, // 启动时沿用在线标定。
        .spi_gyro_config = {
            .bus = PLATFORM_SPI_BUS_2, // 陀螺使用本板第二条 SPI 总线。
            .chip_select = PLATFORM_GPIO_IMU_GYRO_CS, // 陀螺片选由板级表绑定。
        },
        .spi_acc_config = {
            .bus = PLATFORM_SPI_BUS_2, // 加速度计与陀螺共享总线。
            .chip_select = PLATFORM_GPIO_IMU_ACCEL_CS, // 加速度计使用独立片选。
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
