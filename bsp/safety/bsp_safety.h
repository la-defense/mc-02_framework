#ifndef BSP_SAFETY_H
#define BSP_SAFETY_H

/* 禁止中断并立即关闭由 MC-02 直接控制的加热与电源使能输出。
 * 不依赖 HAL、RTOS、tick、日志或模块初始化；可从异常和中断关闭上下文调用。
 */
void BSP_SafetyLatchOutputsOff(void);

/** @brief Latch outputs off at startup, then restore the incoming PRIMASK. */
void BSP_SafetyStartupLatchOutputsOff(void);

#endif
