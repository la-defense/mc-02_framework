/** @file platform_clock_startup.h
 * @brief BSP 启动测试使用的外设期限声明。
 */
#include <stdint.h>
uint8_t PlatformPeripheralStartupEnter(void);
uint8_t PlatformPeripheralStartupLeave(void);
typedef enum { PLATFORM_CLOCK_FAILURE_PERIPHERAL_DEADLINE = 7 } PlatformClockFailure_t;
_Noreturn void PlatformClockStartupFail(PlatformClockFailure_t failure);
