/** @file platform_irq.h
 * @brief 保存并恢复本地中断屏蔽状态，不依赖任务调度器。
 */
#ifndef MC02_PLATFORM_IRQ_H
#define MC02_PLATFORM_IRQ_H
#include <stdint.h>

typedef uint32_t PlatformIrqState_t; /**< 后端保存的状态，只能原样传回恢复接口。 */

/** @brief 保存当前中断屏蔽状态，然后屏蔽本地可屏蔽中断。
 * @return 调用前的状态；嵌套使用时分别保存每次返回值。
 * @note 可在调度器启动前、任务或中断中使用；受保护的代码须短且不能阻塞。
 */
PlatformIrqState_t PlatformIrqSaveAndDisable(void);

/** @brief 恢复对应进入操作之前的中断屏蔽状态。
 * @param[in] saved 对应 PlatformIrqSaveAndDisable 返回的原始状态。
 * @note 必须在同一执行上下文按照相反顺序配对，不得伪造状态。
 */
void PlatformIrqRestore(PlatformIrqState_t saved);
#endif
