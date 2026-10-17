#ifndef WS2812_H
#define WS2812_H

#include "main.h"

/** @file ws2812.h
* @brief MC02 板载彩灯，使用 SPI6（PA5 时钟、PA7 数据）发送 GRB 编码。
* 来源：达妙 MC02 例程（资料/例程/CtrBoard-H7_WS2812），保留本工程移植实现。
* 每个颜色位用一个 SPI 字节发送；代码把 0xC0、0xF0 右移一位后分别作为 0、1 的编码。
* SPI6 以 HSE 24 MHz 四分频工作；这里只说明当前编码，不代替实际电平时间测量。
 */

/** @brief 首次调用时配置 SPI6 和引脚，重复调用不再次配置；失败进入 Error_Handler()。 */
void WS2812_Init(void);
/** @brief 发送颜色，结束后发送零字节使灯带接收本次颜色；未初始化时不处理。
* @param[in] r,g,b 红、绿、蓝分量，各为 0～255，实际按绿、红、蓝顺序发送，高位在前。
* @note 同步发送，不返回 HAL 状态；等待 SPI 空闲的循环没有单独时限，HAL 毫秒计时须能运行。
 */
void WS2812_SetRGB(uint8_t r, uint8_t g, uint8_t b);

#endif // !WS2812_H
