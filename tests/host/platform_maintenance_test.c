/** @file platform_maintenance_test.c
 * @brief 检查周期维护平台适配调用各自的生产入口。
 */
#include "test_check.h"
#include "platform_maintenance.h"

static unsigned calls[3];
int SPIProcessTimeouts(void) { ++calls[0]; return 1; }
void BSP_ADCUpdateDiagnostics(void) { ++calls[1]; }
void CANHealthMonitor(void) { ++calls[2]; }

int main(void)
{
    PlatformMaintainSpi();
    TEST_CHECK(calls[0] == 1u && calls[1] == 0u && calls[2] == 0u);
    PlatformMaintainAnalog();
    TEST_CHECK(calls[0] == 1u && calls[1] == 1u && calls[2] == 0u);
    PlatformMaintainCan();
    TEST_CHECK(calls[0] == 1u && calls[1] == 1u && calls[2] == 1u);
    return 0;
}
