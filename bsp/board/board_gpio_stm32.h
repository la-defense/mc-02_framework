/** @file board_gpio_stm32.h
 * @brief MC-02 板级 GPIO 编号到 STM32 引脚的映射，仅供平台后端使用。
 */
#ifndef MC02_BOARD_GPIO_STM32_H
#define MC02_BOARD_GPIO_STM32_H
#include "platform_resources.h"
#include "main.h"
#include <stdint.h>

typedef struct {
    GPIO_TypeDef *port; /**< GPIO 所属控制器。 */
    uint16_t pin; /**< HAL 使用的单引脚位掩码。 */
} BoardGpioBinding_t;

/** @brief 查询板级逻辑 GPIO 对应的端口与引脚。
 * @param[in] id 逻辑引脚编号。
 * @param[out] output 成功时写入绑定；失败时保持原值。
 * @return 1 为已绑定，0 为无效参数或未绑定；不改变引脚电平。
 */
uint8_t BoardGpioResolve(PlatformGpioPin_t id, BoardGpioBinding_t *output);
#endif
