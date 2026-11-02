/** @file platform_resources.h
 * @brief 车型使用的逻辑资源编号；不暴露芯片或外设句柄。
 */
#ifndef MC02_PLATFORM_RESOURCES_H
#define MC02_PLATFORM_RESOURCES_H
#include <stdint.h>

/** @brief 车型使用的经典 CAN 总线编号，由板级适配决定实际控制器。 */
typedef enum {
    PLATFORM_CAN_INVALID = 0, /**< 未指定总线，不可注册设备。 */
    PLATFORM_CAN_BUS_1,       /**< 板级声明的第一条 CAN 总线。 */
    PLATFORM_CAN_BUS_2,       /**< 板级声明的第二条 CAN 总线。 */
    PLATFORM_CAN_BUS_3        /**< 板级声明的第三条 CAN 总线。 */
} PlatformCanBus_t;

/** @brief 车型声明的逻辑 UART 编号，实际端口由板级适配绑定。 */
typedef enum {
    PLATFORM_UART_INVALID = 0, /**< 未指定串口。 */
    PLATFORM_UART_1 = 1,       /**< 板级声明的串口 1。 */
    PLATFORM_UART_2 = 2,       /**< 板级声明的串口 2。 */
    PLATFORM_UART_3 = 3,       /**< 板级声明的串口 3。 */
    PLATFORM_UART_5 = 5,       /**< 板级声明的串口 5。 */
    PLATFORM_UART_7 = 7,       /**< 板级声明的串口 7。 */
    PLATFORM_UART_8 = 8,       /**< 板级声明的串口 8。 */
    PLATFORM_UART_9 = 9,       /**< 板级声明的串口 9。 */
    PLATFORM_UART_10 = 10      /**< 板级声明的串口 10。 */
} PlatformUartPort_t;

/** @brief 车型声明的 IMU 编号；实际传感器与总线由板级适配绑定。 */
typedef enum {
    PLATFORM_IMU_INVALID = 0, /**< 未选择传感器。 */
    PLATFORM_IMU_BOARD_1      /**< 本板声明的第一个板载 IMU。 */
} PlatformImuPort_t;

/** @brief 车型或设备使用的逻辑 SPI 总线编号。 */
typedef enum {
    PLATFORM_SPI_INVALID = 0, /**< 未选择总线。 */
    PLATFORM_SPI_BUS_1,       /**< 板级声明的 SPI 总线 1。 */
    PLATFORM_SPI_BUS_2        /**< 板级声明的 SPI 总线 2。 */
} PlatformSpiBus_t;

/** @brief 设备使用的逻辑 GPIO 引脚编号。 */
typedef enum {
    PLATFORM_GPIO_INVALID = 0, /**< 未选择引脚。 */
    PLATFORM_GPIO_IMU_GYRO_CS, /**< 板载 IMU 陀螺片选。 */
    PLATFORM_GPIO_IMU_ACCEL_CS /**< 板载 IMU 加速度计片选。 */
} PlatformGpioPin_t;

/** @brief 业务使用的逻辑 PWM 通道。 */
typedef enum {
    PLATFORM_PWM_INVALID = 0, /**< 未选择通道。 */
    PLATFORM_PWM_IMU_HEATER  /**< 本板 IMU 加热输出。 */
} PlatformPwmChannel_t;

/** @brief 业务使用的逻辑模拟输入。 */
typedef enum {
    PLATFORM_ADC_INVALID = 0, /**< 未选择通道。 */
    PLATFORM_ADC_VCC_IN       /**< 本板电源输入电压。 */
} PlatformAdcChannel_t;

/** @brief 业务声明的经典 CAN 端点，不包含芯片句柄和回调指针。 */
typedef struct {
    PlatformCanBus_t can_bus; /**< 本板逻辑总线编号。 */
    uint32_t tx_id; /**< 待发送的标准帧 ID；部分驱动按电机编号解释并重新计算。 */
    uint32_t rx_id; /**< 待接收的标准帧 ID；部分驱动从电机编号计算。 */
} PlatformCanEndpoint_t;
#endif
