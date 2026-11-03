/** @file platform_maintenance_stm32.c
 * @brief 将周期维护入口绑定到 MC-02 现有 SPI、ADC 和 CAN 实现。
 */
#include "platform_maintenance.h"
#include "bsp_spi.h"
#include "bsp_adc.h"
#include "bsp_can.h"

void PlatformMaintainSpi(void)
{
    (void)SPIProcessTimeouts(); // 保持原有事务期限、停止和隔离规则。
}

void PlatformMaintainAnalog(void)
{
    BSP_ADCUpdateDiagnostics(); // 保持原有采样频率和诊断结果。
}

void PlatformMaintainCan(void)
{
    CANHealthMonitor(); // 保持原有总线故障检查与恢复流程。
}
