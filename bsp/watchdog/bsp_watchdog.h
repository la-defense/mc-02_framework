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
 * @note 先调用 DWT_Init()，确保 CYCCNT 正在计数且频率参数正确。每次等待寄存器更新
 *       分别按 20ms 检查，超时尝试输出错误日志后继续，函数没有成功或失败返回值。
 *       时限按 32kHz 估算，没有完整的参数范围检查；当前工程使用 200、4000 和 8000ms。
 */
void BSP_WatchdogInit(uint32_t timeout_ms);

/**
 * @brief 初始化完成后重载 IWDG1 计数，未初始化时不写寄存器。
 * @note 正常运行时由 TaskMonitor 确认任务状态后调用，避免任务失去响应时仍不断喂狗。
 */
void BSP_WatchdogFeed(void);

/**
 * @brief 保存新的请求时限，已初始化时同时更新硬件并重载计数。
 * @param[in] timeout_ms 请求时限，单位 ms；0 的处理及等待条件同 BSP_WatchdogInit()。
 * @note 未初始化时只保存数值；之后仍由 BSP_WatchdogInit() 的实参决定配置。
 */
void BSP_WatchdogSetTimeout(uint32_t timeout_ms);

/**
 * @brief 查询软件保存的最近一次请求时限。
 * @return 请求时限，单位 ms；初始为 200，之后为最近传给 BSP_WatchdogInit()
 *         或 BSP_WatchdogSetTimeout() 的原始实参。
 * @note 不读取硬件配置，传 0 后也会返回 0，不能据此确认实际超时或写入是否成功。
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
 * @return 1 表示已执行完 BSP_WatchdogInit()，0 表示尚未完成；不读取硬件是否正在运行。
 */
uint8_t BSP_WatchdogIsRunning(void);

#endif // !BSP_WATCHDOG_H
