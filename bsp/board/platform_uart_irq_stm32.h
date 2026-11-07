/** @file platform_uart_irq_stm32.h
 * @brief 管理 MC-02 各 UART 及其专用 DMA 的中断。
 */
#pragma once
#include "platform_resources.h"

/** @brief 清除旧中断并启用指定串口及其 DMA 中断。
 * @param[in] port 板级逻辑串口编号。
 * @return 已识别编号返回 1，否则返回 0 且不操作中断。
 * @note 应在 GPIO 与 DMA 配置完成后调用。
 */
unsigned char PlatformUartInterruptEnable(PlatformUartPort_t port);

/** @brief 禁止并清除指定串口及其 DMA 中断。
 * @param[in] port 板级逻辑串口编号。
 * @return 已识别编号返回 1，否则返回 0 且不操作中断。
 * @note 应在释放 DMA、引脚和时钟之前调用；允许重复调用。
 */
unsigned char PlatformUartInterruptDisable(PlatformUartPort_t port);
