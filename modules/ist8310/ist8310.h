/**
 * @file ist8310.h
 * @brief 定义可选 IST8310 磁力计的配置、初始化入口和三轴磁场数据。
 *
 * IST8310Init 登记 I²C 和 GPIO；数据就绪中断触发读取，再更新实例中的 mag。
 * 当前应用未调用此入口。接入前需配置外设、复位引脚和数据就绪中断。
 */
#pragma once // 防止重复包含

#include "bsp_iic.h"
#include "bsp_gpio.h"
#include "stdint.h"

// 三轴 int16_t 读数乘此值，换算为 µT（微特斯拉）。
#define MAG_SEN 0.3f

// 已定义的状态编号；当前 IST8310Init 返回指针或 NULL，未用这两个编号作为返回值。
#define IST8310_NO_ERROR 0x00
#define IST8310_NO_SENSOR 0x40
#define IST8310_IIC_ADDRESS 0x0E // 7位地址，由配置传入；底层 IICRegister 负责左移

/**
 * @brief 保存底层登记对象和最近解析的磁场数据；这些对象继续供回调使用。
 */
typedef struct tempist8310
{
    IICInstance *iic;        /**< I²C 登记对象，其 id 指向本实例。 */
    GPIOInstance *gpio_rst;  /**< 复位引脚登记对象。 */
    GPIOInstance *gpio_exti; /**< 数据就绪引脚的外部中断登记对象。 */
    uint8_t iic_buffer[8];   /**< 接收空间，本模块每次读取使用前6字节。 */
    float mag[3];            /**< 传感器 x/y/z 轴磁场，单位 µT；本模块没有单独的数据有效标记。 */

    void (*ist_module_callback)(IICInstance *); /**< 当前文件未设置或调用此函数指针。 */
} IST8310Instance;

/**
 * @brief 已配置好的 I²C、复位和数据就绪引脚信息；GPIO_Pin 使用 GPIO_PIN_x 宏。
 */
typedef struct
{
    IIC_Init_Config_s iic_config;      /**< HAL 句柄及未左移的0x0E地址；id 和 callback 会被改写。 */
    GPIO_Init_Config_s gpio_conf_rst;  /**< 复位输出引脚；id 会被改写。 */
    GPIO_Init_Config_s gpio_conf_exti; /**< 数据就绪外部中断引脚；id 和 gpio_model_callback 会被改写。 */
} IST8310_Init_Config_s;

/**
 * @brief 分配实例、登记 I²C 和 GPIO、复位传感器并检查 ID 与配置读回。
 * @param[in,out] config 非空配置；HAL 句柄和 GPIO 配置须已准备好。I²C 登记失败前不改写该结构。
 * @return ID 和配置读回通过时返回实例；空配置、分配失败、I²C 登记失败或 ID 不符返回 NULL。
 * @note I²C 登记成功后会写入 id 和回调字段；调用方须保持 GPIO、HAL 句柄及已登记对象有效。
 *       登记后的 ID 不符会返回 NULL 但不释放对象；配置读回不符会持续打印并等待，不会返回。
 */
IST8310Instance *IST8310Init(IST8310_Init_Config_s *config);
