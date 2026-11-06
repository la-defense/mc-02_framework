/**
 * @file bsp_watchdog.h
 * @brief 配置 IWDG1 超时、重载计数，并保存启动时读取到的复位来源。
 * BSPInit() 先初始化 DWT，再调用本模块；运行中由 TaskMonitor 决定是否喂狗。
 * 时限按 LSI 为 32kHz 估算，查询接口返回软件保存的信息。
 */
#ifndef BSP_WATCHDOG_H
#define BSP_WATCHDOG_H

#include <stdint.h>

/**
 * @brief 启动 IWDG1，设置请求时限并重载计数；再次调用会重新配置。
 * @param[in] timeout_ms 请求时限，单位 ms；传 0 时内部按 1ms 换算，软件仍保存原值。
 * @note 兼容入口忽略结果；需要确认配置的调用者应使用 BSP_WatchdogInitChecked()。
 */
void BSP_WatchdogInit(uint32_t timeout_ms);

/** @brief 启动并确认 IWDG 配置寄存器更新完成。
 * @param[in] timeout_ms 请求时限，单位 ms；0 按 1 ms 换算，最大 32760 ms。
 * @return 1 表示寄存器更新完成；0 表示参数非法或等待失败。等待失败时软件禁止后续喂狗。
 * @note IWDG 启动后无法由软件停止；调用前须准备 DWT，失败时上层应关断并停止启动。
 */
uint8_t BSP_WatchdogInitChecked(uint32_t timeout_ms);

/**
 * @brief 初始化完成后重载 IWDG1 计数，未初始化时不写寄存器。
 * @note 正常运行时由 TaskMonitor 确认任务状态后调用，避免任务失去响应时仍不断喂狗。
 */
void BSP_WatchdogFeed(void);

/**
 * @brief 兼容入口，尝试更新已初始化看门狗的时限。
 * @param[in] timeout_ms 请求时限，单位 ms；0 的处理及等待条件同 BSP_WatchdogInit()。
 * @note 本入口忽略结果；需要确认维护窗口的调用者应使用检查结果的接口。
 */
void BSP_WatchdogSetTimeout(uint32_t timeout_ms);

/** @brief 更新已启动 IWDG 的时限并确认寄存器更新。
 * @param[in] timeout_ms 请求时限，单位 ms；0 按 1 ms 换算，最大 32760 ms。
 * @return 1 表示更新完成；参数非法、未启动或等待失败返回 0。等待失败时禁止继续喂狗。
 * @note 参数非法时保留现有配置；任何失败都不发布新请求时限，调用方须处理失败。
 */
uint8_t BSP_WatchdogSetTimeoutChecked(uint32_t timeout_ms);

/**
 * @brief 查询软件保存的最近一次请求时限。
 * @return 请求时限，单位 ms；初始为 200，之后为最近一次确认配置成功的原始实参。
 * @note 不读取硬件配置；实际时限受 LSI 频率和分频影响。
 */
uint32_t BSP_WatchdogGetTimeout(void);

/**
 * @brief 读取并打印 RCC 复位标志，保存 IWDG1 复位信息，最后清除硬件复位标志。
 * @note 启动时在日志初始化后、其他代码清除复位标志前调用。多种来源可同时出现；
 *       已记录的 IWDG1 信息保留到本次启动结束，再次读取不会将它清零。
 */
void BSP_WatchdogLogResetReason(void);

/**
 * @brief 查询本次启动是否记录过 IWDG1 复位标志，不改写记录。
 * @return 1 表示 BSP_WatchdogLogResetReason() 曾读到该标志；0 表示尚未记录，包括尚未读取的情况。
 */
uint8_t BSP_WatchdogWasIwdgReset(void);

/**
 * @brief 查询本次启动的软件初始化标志。
 * @return 1 表示最近一次配置已完成，0 表示未完成或更新失败；不读取硬件是否正在运行。
 */
uint8_t BSP_WatchdogIsRunning(void);

#endif // !BSP_WATCHDOG_H
