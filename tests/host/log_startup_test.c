/** @file log_startup_test.c
 * @brief 用生产模块计划检查日志出口失败的影响范围。
 */
#include "log_startup.h"
#include "bsp_log.h"
#include "test_check.h"
#include <stddef.h>

static BSPLogStatus_t status = BSP_LOG_STATUS_OK;
static BSPLogState_t log_state;
static RobotModulePlan_t plan;

BSPLogStatus_t BSPLogGetState(BSPLogState_t *output)
{
    TEST_CHECK(output != NULL); // 生产入口必须提供状态存储。
    if (status == BSP_LOG_STATUS_OK) *output = log_state; // 查询失败时保留调用者原值。
    return status;
}

static uint8_t StartModule(void *context, unsigned module)
{
    (void)context;
    return module == ROBOT_MODULE_LOG ? RobotStartLogConfigured(&plan) : 1u;
}

int main(void)
{
    const uint32_t log = ROBOT_MODULE_BIT(ROBOT_MODULE_LOG); // 日志模块位。
    const uint32_t chassis = ROBOT_MODULE_BIT(ROBOT_MODULE_CHASSIS); // 模拟核心使用者。
    RobotModuleStartup_t state;
    plan.enabled = log | chassis; // 两个模块均被车型启用。
    plan.required = chassis; // 最初日志仅是辅助功能。
    TEST_CHECK(RobotModuleStart(&plan, StartModule, NULL, &state) == ROBOT_MODULE_OK);
    TEST_CHECK((state.started & log) != 0u); // RTT 不可用时仍保留辅助日志状态。
    plan.dependencies[ROBOT_MODULE_CHASSIS] = log; // 核心底盘显式依赖日志。
    TEST_CHECK(RobotModuleStart(&plan, StartModule, NULL, &state) == ROBOT_MODULE_CORE_FAILURE);
    TEST_CHECK((state.failed & log) != 0u); // 依赖失败由统一计划报告。
    TEST_CHECK((state.started & chassis) == 0u); // 核心依赖失败时不能启动底盘。
    log_state.backend_available = 1u; // RTT 成功时核心依赖满足。
    TEST_CHECK(RobotModuleStart(&plan, StartModule, NULL, &state) == ROBOT_MODULE_OK);
    TEST_CHECK((state.started & log) != 0u);
    status = BSP_LOG_STATUS_NOT_INITIALIZED; // 状态未知不能满足核心依赖。
    TEST_CHECK(RobotModuleStart(&plan, StartModule, NULL, &state) == ROBOT_MODULE_CORE_FAILURE);
    TEST_CHECK(!RobotStartLogConfigured(NULL));
    return 0;
}
