/** @file chassis_supercap_legacy.h
 * @brief 旧电容设备初始化入口；协议与普通供电回退尚待确认。
 */
#ifndef MC02_CHASSIS_SUPERCAP_LEGACY_H
#define MC02_CHASSIS_SUPERCAP_LEGACY_H
#include <stdint.h>
/** @brief 初始化当前旧电容驱动。
 * @return 注册成功为 1，失败为 0；调用前需完成板级 CAN 初始化。
 * @note 设备未选型，不能据此认为普通供电功率限制已实现。
 */
uint8_t ChassisLegacySuperCapInit(void);
/** @brief 回退底盘在启动阶段创建的旧电容实例。
 * @return 已回退或没有实例返回 1；底层拒绝回退返回 0。
 * @note 仅在接收派发、发送及业务任务启动之前调用。
 */
uint8_t ChassisLegacySuperCapRollbackStartup(void);
#endif
