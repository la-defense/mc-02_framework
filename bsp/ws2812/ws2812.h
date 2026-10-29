#ifndef WS2812_H
#define WS2812_H

#include "main.h"

/** @file ws2812.h
* @brief MC02 板载彩灯，使用 SPI6 从 PA7 发送 GRB 编码。
* 来源：达妙 MC02 例程（资料/例程/CtrBoard-H7_WS2812），保留本工程移植实现。
* 每个颜色位用一个 SPI 字节发送；代码把 0xC0、0xF0 右移一位后分别作为 0、1 的编码。
* PA5 保留给 ADC1_INP19 按键采样；SPI6 时钟留在外设内部，不占用该引脚。
* 代码说明当前编码，不代替实际灯珠波形和时序测量。
 */

/** @brief 配置 SPI6 和数据引脚；重复调用保持已初始化状态。
 * @return HAL_OK 表示成功；时钟配置或 SPI 初始化失败时返回 HAL 状态，不进入 Error_Handler。
 */
HAL_StatusTypeDef WS2812_Init(void);
/** @brief 发送颜色及复位低电平字节；未初始化时返回 HAL_ERROR。
* @param[in] r,g,b 红、绿、蓝分量，各为 0～255，实际按绿、红、蓝顺序发送，高位在前。
* @return HAL_OK 表示本机阻塞发送完成；时基不可用返回 BUSY，其余 HAL 状态由调用方处理。
* @note 普通任务中调用，一次发送 124 字节，HAL 超时参数为 2 ms；实际耗时受调度影响，灯珠波形仍需测量。
 */
HAL_StatusTypeDef WS2812_SetRGB(uint8_t r, uint8_t g, uint8_t b);

#endif // !WS2812_H
