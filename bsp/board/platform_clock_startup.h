/** @file platform_clock_startup.h
 * @brief H723 主时钟初始化期间的有界等待和失败处理。
 */
#ifndef MC02_PLATFORM_CLOCK_STARTUP_H
#define MC02_PLATFORM_CLOCK_STARTUP_H
#include <stdint.h>

/** @brief 主时钟启动失败的位置。 */
typedef enum {
    PLATFORM_CLOCK_FAILURE_SUPPLY = 1, /**< 电源供给配置失败。 */
    PLATFORM_CLOCK_FAILURE_VOS, /**< 电压调节器未在期限内就绪。 */
    PLATFORM_CLOCK_FAILURE_OSCILLATOR, /**< HSE／HSI 或 PLL 配置失败。 */
    PLATFORM_CLOCK_FAILURE_SWITCH, /**< 主时钟或分频切换失败。 */
    PLATFORM_CLOCK_FAILURE_DEADLINE, /**< 周期期限或查询次数已用尽。 */
    PLATFORM_CLOCK_FAILURE_HAL, /**< HAL 基础初始化失败。 */
    PLATFORM_CLOCK_FAILURE_PERIPHERAL_DEADLINE, /**< 早期外设准备达到总期限。 */
} PlatformClockFailure_t;

/** @brief 初始化启动期限并启用周期计数；只能在调度器启动前调用。 */
void PlatformClockStartupBegin(void);
/** @brief 检查 VOS 就绪并限定等待；成功返回 1，失败返回 0。 */
uint8_t PlatformClockStartupWaitVos(void);
/** @brief 检查原始 HAL 时基前进并结束保护；时基正常且总期限未耗尽返回 1。 */
uint8_t PlatformClockStartupFinish(void);
/** @brief 开始调度器启动前外设准备的固定总期限。
 * @return 成功返回 1；主时钟保护尚未结束或本次上电已开始过外设期限时返回 0。
 */
uint8_t PlatformPeripheralStartupBegin(void);
/** @brief 在启动线程中进入一次 HAL 等待保护；不重置总期限。
 * @return 可执行本次操作返回 1；未开始、已到期、嵌套或中断中调用返回 0。
 */
uint8_t PlatformPeripheralStartupEnter(void);
/** @brief 退出本次 HAL 等待保护并检查总期限。
 * @return 未到期返回 1；未进入或已到期返回 0。
 */
uint8_t PlatformPeripheralStartupLeave(void);
/** @brief 结束外设准备并检查同一总期限；不得在保护内部调用。
 * @return 成功结束且未到期返回 1，否则返回 0。
 */
uint8_t PlatformPeripheralStartupFinish(void);
/** @brief 获取未经启动等待保护修改的 HAL 毫秒计数。
 * @return 原始毫秒计数，允许 32 位回绕。
 */
uint32_t PlatformClockRawTick(void);
/** @brief 关断输出、记录具体原因并按恢复额度请求复位；不能返回。 */
__attribute__((noreturn)) void PlatformClockStartupFail(PlatformClockFailure_t failure);
#endif
