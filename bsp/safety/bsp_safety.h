/**
 * @file bsp_safety.h
 * @brief 将 MC-02 的加热和三路电源使能引脚设为低电平，停止 TIM3 输出。
 * 启动及初始化失败时用 BSP_SafetyStartupLatchOutputsOff()，
 * 故障处理中需要保持普通中断关闭时用 BSP_SafetyLatchOutputsOff()。
 */
#ifndef BSP_SAFETY_H
#define BSP_SAFETY_H

/**
 * @brief 关闭普通中断，将 PB1 和 PC13～PC15 设为低电平输出，关闭 TIM3_CH4 并停止 TIM3 计数。
 * @note 返回后普通中断继续关闭，可从异常或已关闭普通中断的代码中调用。
 *       直接操作寄存器，不依赖 HAL、RTOS、tick、日志或模块初始化。
 *       本次设置会保留，后续代码仍可重新配置这些引脚和定时器。
 */
void BSP_SafetyLatchOutputsOff(void);

/**
 * @brief 执行相同的输出关断，随后恢复调用前的普通中断开关状态。
 * @note 可在 HAL 和模块初始化前调用；当前也用于初始化失败后的处理。
 *       保存并恢复 PRIMASK，调用前已关闭普通中断时仍保持关闭，输出保持本次设置。
 */
void BSP_SafetyStartupLatchOutputsOff(void);

#endif
