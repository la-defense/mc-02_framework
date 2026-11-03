/** @file platform_rgb_mc02.c
 * @brief 将 RGB 接口连接到 MC-02 的 WS2812 异步驱动。
 */
#include "platform_rgb.h"
#include "ws2812.h"

static PlatformRgbStatus_t MapStatus(HAL_StatusTypeDef status)
{
    if (status == HAL_OK)
        return PLATFORM_RGB_OK; // 保留本机发送完成的含义。
    if (status == HAL_BUSY)
        return PLATFORM_RGB_BUSY; // 仍需任务推进事务。
    return PLATFORM_RGB_ERROR; // 错误和超时均报告通道不可用。
}

PlatformRgbStatus_t PlatformRgbInit(PlatformRgbPort_t port)
{
    if (port != PLATFORM_RGB_BOARD_1)
        return PLATFORM_RGB_ERROR; // 不访问未绑定到本板的指示灯。
    return MapStatus(WS2812_Init());
}

PlatformRgbStatus_t PlatformRgbSet(uint8_t r, uint8_t g, uint8_t b)
{
    return MapStatus(WS2812_SetRGB(r, g, b));
}

void PlatformRgbProcess(void)
{
    (void)WS2812_Process(); // 按原有任务周期推进完成与超时处理。
}
