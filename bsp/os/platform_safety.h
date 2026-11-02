/** @file platform_safety.h
 * @brief 平台提供的核心任务故障记录、立即关断和看门狗喂养入口。
 */
#ifndef MC02_PLATFORM_SAFETY_H
#define MC02_PLATFORM_SAFETY_H

/** @brief 在复位恢复记录中标记核心任务故障。
 * @note 不复位、不解除已有锁存；记录实现由平台提供。
 */
void PlatformRecordCoreTaskFault(void);

/** @brief 立即关断平台控制的安全输出并保持普通中断关闭。
 * @note MC-02 后端直接访问寄存器，不依赖 HAL、调度器、堆或日志。
 */
void PlatformSafetyLatchOutputsOff(void);

/** @brief 在任务监控确认允许时重载硬件看门狗。 */
void PlatformWatchdogFeed(void);
#endif
