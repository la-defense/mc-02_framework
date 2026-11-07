/** @file platform_can_diagnostic_stm32.c
 * @brief 将显式启用的回环诊断交给 MC-02 CAN 驱动。
 */
#include "platform_can_diagnostic.h"
#if CAN_SELFTEST_LOOPBACK
#include "bsp_can.h"
#include "platform_feature_startup.h"

uint8_t PlatformCanRunLoopbackDiagnostic(void)
{
    if (!PlatformFeatureCanReady(PLATFORM_CAN_BUS_1) ||
        !PlatformFeatureCanReady(PLATFORM_CAN_BUS_2) ||
        !PlatformFeatureCanReady(PLATFORM_CAN_BUS_3)) return 0u; // 自测需要三条已准备的总线。
    return CANRunLoopbackSelfTest(); // 延续原有模式切换、测试和恢复行为。
}
#endif
