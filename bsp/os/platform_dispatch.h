/** @file platform_dispatch.h
 * @brief 由单一任务推进当前平台的 CAN 和串口队列。
 */
#ifndef MC02_PLATFORM_DISPATCH_H
#define MC02_PLATFORM_DISPATCH_H

/** @brief 派发当前平台各 CAN 接收队列中本轮允许处理的数据。
 * @note 仅普通任务调用；全工程须保持唯一的周期接收派发者。
 */
void PlatformDispatchCanRx(void);

/** @brief 派发当前平台各串口接收队列并处理既有错误恢复。
 * @note 仅普通任务调用；全工程须保持唯一的周期接收派发者。
 */
void PlatformDispatchUartRx(void);

/** @brief 推进当前平台各 CAN 总线的发送队列。
 * @note 仅普通任务调用；返回不代表硬件或总线完成发送。
 */
void PlatformDispatchCanTx(void);
#endif
