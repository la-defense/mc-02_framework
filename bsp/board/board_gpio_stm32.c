/** @file board_gpio_stm32.c
 * @brief MC-02 的逻辑 GPIO 绑定。
 */
#include "board_gpio_stm32.h"
#include <stddef.h>

uint8_t BoardGpioResolve(PlatformGpioPin_t id, BoardGpioBinding_t *output)
{
    if (output == NULL) return 0u; // 不访问空输出地址。
    BoardGpioBinding_t candidate;
    switch (id) {
    case PLATFORM_GPIO_IMU_GYRO_CS:
        candidate = (BoardGpioBinding_t){CS2_GYRO_GPIO_Port, CS2_GYRO_Pin}; // 陀螺片选。
        break;
    case PLATFORM_GPIO_IMU_ACCEL_CS:
        candidate = (BoardGpioBinding_t){CS2_ACCEL_GPIO_Port, CS2_ACCEL_Pin}; // 加速度计片选。
        break;
    default:
        return 0u; // 未绑定时不改调用者已有结果。
    }
    *output = candidate; // 检查完成后一次交付整份绑定。
    return 1u;
}
