/**
 * @file bsp_safety_error.c
 * @brief 提供 Error_Handler()，处理外设初始化等流程中报告的错误。
 * 入口声明在 main.h，通过 BSP_SafetyLatchOutputsOff() 关闭已知输出。
 */
#include "main.h"

#include "bsp_safety.h"
#include "bsp_reset_recovery.h"

/**
 * @brief 关闭输出、记录致命故障并按剩余恢复额度处理。
 * @note 定义 MC02_HOST_TEST 时关断并记录后返回；固件不返回。
 */
void Error_Handler(void)
{
    __disable_irq();
    // 模块初始化可能尚未完成，使用直接操作寄存器的关闭入口。
    BSP_SafetyLatchOutputsOff();
    BSP_ResetRecoveryMarkFault(RESET_RECOVERY_CAUSE_FATAL_HANDLER); // 在关断之后保存本次故障。

#if defined(MC02_HOST_TEST)
    // 主机测试需要返回，才能检查中断、引脚和定时器是否已按要求设置。
    return;
#else
    if (!BSP_ResetRecoveryIsLockedOut())
    {
        __HAL_RCC_CLEAR_RESET_FLAGS(); // 避免旧上电标志在软件复位后清掉恢复次数。
        NVIC_SystemReset(); // 有恢复额度时重新启动并检查全部核心条件。
    }
    // 已锁定或复位意外返回时保持关断；看门狗跨复位仍运行时阻止无限复位。
    for (;;)
    {
        IWDG1->KR = 0xAAAAu; // 重载可能仍在运行的独立看门狗。
        __NOP(); // 不执行任何初始化或业务控制。
    }
#endif
}
