/** @file platform_startup_wait.h
 * @brief 调度器启动前使用的有界条件等待，不依赖 HAL 或 RTOS。
 */
#ifndef MC02_PLATFORM_STARTUP_WAIT_H
#define MC02_PLATFORM_STARTUP_WAIT_H
#include <stdint.h>

/** @brief 启动前等待的结果。 */
typedef enum {
    STARTUP_WAIT_READY = 0, /**< 条件在期限内成立。 */
    STARTUP_WAIT_TIMEOUT, /**< 周期计数达到固定期限。 */
    STARTUP_WAIT_POLL_LIMIT, /**< 检查次数用尽，计时未能先结束等待。 */
    STARTUP_WAIT_ARGUMENT /**< 参数无效，未执行等待。 */
} PlatformStartupWaitResult_t;

typedef uint8_t (*PlatformStartupReadyFn)(void *context); /**< 非零表示条件成立。 */
typedef uint32_t (*PlatformStartupCyclesFn)(void *context); /**< 返回同一 32 位周期计数。 */

/** @brief 在固定周期数及检查次数内等待一个启动条件。
 * @param[in] ready 非空条件检查函数，必须有界返回，不得更改等待期限。
 * @param[in] cycles 非空周期计数读取函数，使用同一计数来源，允许回绕。
 * @param[in] context 原样交给两个函数；由调用者保证有效，允许为 NULL。
 * @param[in] timeout_cycles 固定期限，范围 1～INT32_MAX 个周期。
 * @param[in] max_polls 条件最多检查次数，必须大于零；计数停止时也会退出。
 * @return 就绪、周期超时、检查次数用尽或参数错误。
 * @note 仅用于短暂启动等待；不调用 HAL、RTOS、日志或看门狗，不测量硬件实际耗时。
 */
PlatformStartupWaitResult_t PlatformStartupWait(PlatformStartupReadyFn ready,
    PlatformStartupCyclesFn cycles, void *context, uint32_t timeout_cycles, uint32_t max_polls);
#endif
