/** @file init_diagnostic.h
 * @brief BSP 启动测试使用的降级事件声明。
 */
#include <stdint.h>
enum { INIT_STAGE_BSP = 1, INIT_MODULE_ADC = 13, INIT_MODULE_LOG = 14, INIT_FAILURE_RESOURCE = 1,
       INIT_RESOURCE_LOG_RTT = 1, INIT_RESOURCE_LOG_DIAGNOSTIC_UART = 2, INIT_RESOURCE_LOG_STATUS = 3 };
void InitDiagnosticReportDegraded(int stage, uint8_t module, int reason, uint16_t resource_id);
