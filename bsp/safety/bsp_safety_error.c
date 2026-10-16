/**
 * @file bsp_safety_error.c
 * @brief 提供 Error_Handler()，处理外设初始化等流程中报告的错误。
 * 入口声明在 main.h，通过 BSP_SafetyLatchOutputsOff() 关闭已知输出。
 */
#include "main.h"

#include "bsp_safety.h"

/**
 * @brief 关闭普通中断和已知输出，避免错误发生后继续执行原来的流程。
 * @note 定义 MC02_HOST_TEST 时关断后返回；固件中保持普通中断关闭并持续等待。
 */
void Error_Handler(void)
{
    __disable_irq();
    // 模块初始化可能尚未完成，使用直接操作寄存器的关闭入口。
    BSP_SafetyLatchOutputsOff();

#if defined(MC02_HOST_TEST)
    // 主机测试需要返回，才能检查中断、引脚和定时器是否已按要求设置。
    return;
#else
    // 留在这里，避免返回出错的调用处继续执行；等待看门狗或外部操作复位。
    for (;;)
        ;
#endif
}
