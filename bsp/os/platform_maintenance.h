/** @file platform_maintenance.h
 * @brief 普通任务周期推进板级通信和模拟输入维护。
 */
#ifndef MC02_PLATFORM_MAINTENANCE_H
#define MC02_PLATFORM_MAINTENANCE_H

/** @brief 检查异步 SPI 事务期限并推进出错事务的停止处理。
 * @note 由单一普通任务周期调用；停止仍按原后端规则异步完成。
 */
void PlatformMaintainSpi(void);

/** @brief 推进板级模拟电源输入诊断。
 * @note 后端控制采样周期；诊断结果不直接改变整车许可。
 */
void PlatformMaintainAnalog(void);

/** @brief 检查 CAN 总线状态并推进已有恢复流程。
 * @note 由单一普通任务周期调用，不保证本次调用完成恢复。
 */
void PlatformMaintainCan(void);
#endif
