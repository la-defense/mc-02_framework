/** @file platform_signal_freertos.c
 * @brief 将框架的合并信号位映射到本工程 CMSIS-RTOS 后端。
 */
#include "platform_signal.h"
#include "cmsis_os.h"
#include <stddef.h>

uint8_t PlatformSignalSet(PlatformTaskHandle_t task, uint32_t bits)
{
    if (task == NULL || bits == 0u || (bits & ~PLATFORM_SIGNAL_MAX_MASK) != 0u)
        return 0u; // 不把无效句柄或保留位传给 RTOS。
    const int32_t previous = osSignalSet((osThreadId)task, (int32_t)bits); // 后端合并信号位。
    return (uint8_t)(previous >= 0); // 最高位置位的返回值表示后端错误。
}

PlatformSignalResult_t PlatformSignalWait(uint32_t clear_bits, uint32_t timeout_ms, uint32_t *observed)
{
    if (observed == NULL || clear_bits == 0u || (clear_bits & ~PLATFORM_SIGNAL_MAX_MASK) != 0u)
        return PLATFORM_SIGNAL_INVALID; // 非法输入时保留调用者原值。
    const osEvent event = osSignalWait((int32_t)clear_bits, timeout_ms); // 沿用 CMSIS 等待与清位行为。
    if (event.status == osEventSignal) {
        const uint32_t pending = (uint32_t)event.value.signals & ~clear_bits; // 取出未被本次消费的位。
        if (pending != 0u) {
            const osThreadId current = osThreadGetId(); // 获取刚完成等待的当前任务。
            if (current == NULL || osSignalSet(current, (int32_t)pending) < 0)
                return PLATFORM_SIGNAL_ERROR; // 无法恢复待处理标志时不能报告成功。
        }
        *observed = (uint32_t)event.value.signals; // 只在收到信号后交付后端返回值。
        return PLATFORM_SIGNAL_RECEIVED;
    }
    if (event.status == osEventTimeout || event.status == osOK)
        return PLATFORM_SIGNAL_TIMEOUT; // osOK 对应立即检查时没有信号。
    return PLATFORM_SIGNAL_ERROR; // 例如中断中等待或后端不支持通知。
}
