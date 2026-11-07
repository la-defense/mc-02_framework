/** @file platform_feature_startup.h
 * @brief 按固件启用的功能准备 MC-02 板载 SPI 和 PWM 外设。
 */
#ifndef MC02_PLATFORM_FEATURE_STARTUP_H
#define MC02_PLATFORM_FEATURE_STARTUP_H

/** @brief 在 GPIO 和 DMA 初始化之后、业务启动之前准备所需外设。
 * @note 当前生成的初始化函数遇到硬件错误时仍进入 Error_Handler；辅助降级尚未接入。
 */
void PlatformFeaturePeripheralsInit(void);
#endif
