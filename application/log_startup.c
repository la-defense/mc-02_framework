/** @file log_startup.c
 * @brief 将 RTT 日志状态与车型核心依赖关联。
 */
#include "log_startup.h"
#include "bsp_log.h"
#include <stddef.h>

uint8_t RobotStartLogConfigured(const RobotModulePlan_t *plan)
{
    if (plan == NULL) return 0u; // 缺少依赖声明时不能判断日志是否必要。
    BSPLogState_t state = {0}; // 无法查询时不能把未知状态当作已就绪。
    const uint8_t available = (uint8_t)(BSPLogGetState(&state) == BSP_LOG_STATUS_OK &&
                                         state.backend_available != 0u); // RTT 是日志正文的输出通道。
    if (available) return 1u; // 出口已就绪，任务创建在后续阶段处理。
    const uint32_t log_bit = ROBOT_MODULE_BIT(ROBOT_MODULE_LOG); // 按完整依赖闭包判断影响范围。
    return (uint8_t)((RobotModuleCoreClosure(plan) & log_bit) == 0u); // 辅助降级已由基础启动记录。
}
