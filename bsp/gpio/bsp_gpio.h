/** @file bsp_gpio.h
* @brief 保存 GPIO 实例和 EXTI 回调；GPIO 的引脚模式与中断触发方式仍须提前由板级代码配置。
 */
#include "gpio.h"
#include "stdint.h"

#define GPIO_MX_DEVICE_NUM 10

/**
 * @brief 用于判断中断来源,注意和CUBEMX中配置一致
 *
 */
typedef enum
{
    GPIO_EXTI_MODE_RISING,
    GPIO_EXTI_MODE_FALLING,
    GPIO_EXTI_MODE_RISING_FALLING,
    GPIO_EXTI_MODE_NONE,
} GPIO_EXTI_MODE_e;

/**
 * @brief GPIO实例结构体定义
 *
 */
typedef struct tmpgpio
{
    GPIO_TypeDef *GPIOx;        // GPIOA,GPIOB,GPIOC...
    GPIO_PinState pin_state;    // 引脚状态,Set,Reset;not frequently used
    GPIO_EXTI_MODE_e exti_mode; // 外部中断模式 not frequently used
    uint16_t GPIO_Pin;          // 引脚号,
    // 使用当前 H7 HAL 的 GPIO_PIN_0、GPIO_PIN_1 等引脚位值，不是数字序号。
    // 随便取个名字当临时声明
    void (*gpio_model_callback)(struct tmpgpio *); // exti中断回调函数
    void *id;                                      // 区分不同的GPIO实例

} GPIOInstance;

/**
 * @brief GPIO初始化配置结构体定义
 *
 */
typedef struct
{
    GPIO_TypeDef *GPIOx;        // GPIOA,GPIOB,GPIOC...
    GPIO_PinState pin_state;    // 引脚状态,Set,Reset not frequently used
    GPIO_EXTI_MODE_e exti_mode; // 外部中断模式 not frequently used
    uint16_t GPIO_Pin;          // 引脚号,@note 这里的引脚号是GPIO_PIN_0,GPIO_PIN_1...
    // 使用当前 H7 HAL 的 GPIO_PIN_0、GPIO_PIN_1 等引脚位值，不是数字序号。

    void (*gpio_model_callback)(GPIOInstance *); // exti中断回调函数
    void *id;                                    // 区分不同的GPIO实例

} GPIO_Init_Config_s;

/** @brief 复制配置并保存到 GPIO 实例表，不在此处修改引脚模式。
* @param[in] GPIO_config 有效配置，端口、引脚及回调地址须在使用期间有效，不能为 NULL。
* @return 分配的内部对象，没有检查内存申请失败；目前最多容纳 10 项，但注册没有检查容量。
* @note 仅在启动阶段注册，不可自行释放；同一 EXTI 引脚号应只对应一个回调对象。
 */
GPIOInstance *GPIORegister(GPIO_Init_Config_s *GPIO_config);

/** @brief 翻转指定引脚的输出电平。
* @param[in] _instance 注册得到的有效对象，不能为 NULL。
 */
void GPIOToggel(GPIOInstance *_instance);

/** @brief 将指定引脚输出设为高电平。
* @param[in] _instance 注册得到的有效对象，不能为 NULL。
 */
void GPIOSet(GPIOInstance *_instance);

/** @brief 将指定引脚输出设为低电平。
* @param[in] _instance 注册得到的有效对象，不能为 NULL。
 */
void GPIOReset(GPIOInstance *_instance);

/** @brief 读取引脚当前电平。
* @param[in] _instance 注册得到的有效对象，不能为 NULL。
* @return GPIO_PIN_SET 为高电平，GPIO_PIN_RESET 为低电平。
 */
GPIO_PinState GPIORead(GPIOInstance *_instance);