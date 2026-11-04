/** @file platform_signal.h
 * @brief 提供可合并信号位的任务唤醒接口；不保存事件数据或次数。
 */
#ifndef MC02_PLATFORM_SIGNAL_H
#define MC02_PLATFORM_SIGNAL_H
#include "platform_os.h"
#include <stdint.h>

#define PLATFORM_SIGNAL_MAX_MASK 0x7fffffffu /**< 可使用的信号位，最高位留作后端错误标记。 */
#define PLATFORM_SIGNAL_WAIT_FOREVER UINT32_MAX /**< 等待直到收到通知。 */

/** @brief 等待当前任务信号的结果。 */
typedef enum {
    PLATFORM_SIGNAL_RECEIVED, /**< 返回了信号位。 */
    PLATFORM_SIGNAL_TIMEOUT, /**< 指定时间内没有信号。 */
    PLATFORM_SIGNAL_INVALID, /**< 输入参数无效。 */
    PLATFORM_SIGNAL_ERROR /**< 后端或调用上下文错误。 */
} PlatformSignalResult_t;

/** @brief 向已有任务合并设置一个或多个信号位。
 * @param[in] task 创建成功后取得的任务句柄，不能为 NULL；调用期间任务须存在。
 * @param[in] bits 非零，且只使用 PLATFORM_SIGNAL_MAX_MASK 内的位。
 * @return 1 表示后端接受，0 表示参数或后端错误。
 * @note 可从任务或允许调用 RTOS 接口的中断发出；相同位重复发送会合并。
 * 不复制数据，也不能据此计算发送次数；中断优先级须满足当前 RTOS 要求。
 */
uint8_t PlatformSignalSet(PlatformTaskHandle_t task, uint32_t bits);

/** @brief 等待当前任务的信号，并在成功时清除指定的位。
 * @param[in] clear_bits 非零，且只使用 PLATFORM_SIGNAL_MAX_MASK 内的位。
 * @param[in] timeout_ms 等待毫秒数；0 为立即检查，PLATFORM_SIGNAL_WAIT_FOREVER 为无限等待。
 * @param[out] observed 成功时写入后端报告的全部信号位；其他结果不改变原值。
 * @return 收到、超时、参数错误或后端错误。
 * @note 只能由普通任务调用；后端可能因其他位唤醒，未清除的位会保持待处理。
 * 调用者应处理返回的全部位，或在继续等待前明确清除不需要的位，避免反复立即唤醒。
 * 后端无法重新标记未清除位时返回错误，不修改 observed。
 * H723 保持当前 CMSIS 等待时间到 RTOS tick 的换算，不承诺亚毫秒分辨率。
 */
PlatformSignalResult_t PlatformSignalWait(uint32_t clear_bits, uint32_t timeout_ms, uint32_t *observed);
#endif
