/** @file referee_ui_platform_port.c
 * @brief 将裁判 UI 主机测试的平台调用接到各测试原有的时钟和中断模型。
 */
#include "bsp_usart.h"
#include "../../bsp/os/platform_irq.h"
#include "../../bsp/os/platform_sync.h"

PlatformIrqState_t PlatformIrqSaveAndDisable(void)
{
    const PlatformIrqState_t saved = __get_PRIMASK(); // 读取此测试配置的原始屏蔽状态。
    __disable_irq(); // 复用测试已有的中断模型。
    return saved; // 供调用方在退出时恢复。
}

void PlatformIrqRestore(PlatformIrqState_t saved)
{
    __set_PRIMASK(saved); // 恢复进入前的状态，保留嵌套语义。
}

uint32_t PlatformTimeNowMs(void)
{
    return HAL_GetTick(); // 沿用各测试控制的毫秒计数。
}
