/** @file platform_irq_stm32.c
 * @brief 用 Cortex-M 的 PRIMASK 实现本地中断状态保存与恢复。
 */
#include "platform_irq.h"
#include "main.h"

PlatformIrqState_t PlatformIrqSaveAndDisable(void)
{
    const PlatformIrqState_t saved = __get_PRIMASK(); // 保留进入前的屏蔽状态。
    __disable_irq(); // 屏蔽可屏蔽中断，防止共享数据在复制期间被中断改写。
    return saved; // 交给调用方配对恢复，支持进入前已经屏蔽的情形。
}

void PlatformIrqRestore(PlatformIrqState_t saved)
{
    __set_PRIMASK(saved); // 恢复调用前状态，不无条件开启中断。
}
