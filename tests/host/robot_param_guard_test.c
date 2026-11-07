/** @file robot_param_guard_test.c
 * @brief 检查生产 Flash 维护保护的配对、降级和期限结果。
 */
#include "robot_param_guard.h"
#include "config/vehicle_selection.h"
#include "platform_irq.h"
#include "imu_heater.h"
#include "task_monitor.h"
#include "test_check.h"
#include <string.h>

static uint8_t available, heater_ok = 1u, monitor_ok = 1u, monitor_end_ok = 1u;
static unsigned heater_begin, heater_end, monitor_begin, monitor_end, mask;
static uint8_t heater_result, test_reentrant;
PlatformIrqState_t PlatformIrqSaveAndDisable(void) { const unsigned old = mask; mask = 1u; return old; }
void PlatformIrqRestore(PlatformIrqState_t saved) { mask = saved; }
uint8_t RobotBuildModuleStarted(RobotModule_t module)
{ TEST_CHECK(module == ROBOT_MODULE_HEATER); return available; }
uint8_t IMUHeaterBeginLongOperation(IMUHeater_LongOperation_e op)
{ TEST_CHECK(op == IMU_HEATER_LONG_OPERATION_FLASH && !mask); ++heater_begin; return heater_ok; }
void IMUHeaterEndLongOperation(IMUHeater_LongOperation_e op, uint8_t success)
{ TEST_CHECK(op == IMU_HEATER_LONG_OPERATION_FLASH && !mask); ++heater_end; heater_result = success; }
uint8_t TaskMonitorBeginLongOperation(TaskMonitor_Id_e id, TaskMonitor_LongOperation_e op)
{
    TEST_CHECK(id == TASK_MONITOR_INS && op == TASK_MONITOR_LONG_OPERATION_FLASH && !mask);
    ++monitor_begin;
    if (test_reentrant)
    {
        TEST_CHECK(!RobotParamFlashOperationBegin()); // 准备阶段不能接受第二次开始。
        TEST_CHECK(!RobotParamFlashOperationEnd(1u)); // 尚未取得监控窗口时不能结束。
    }
    return monitor_ok;
}
uint8_t TaskMonitorEndLongOperation(TaskMonitor_Id_e id, TaskMonitor_LongOperation_e op)
{ TEST_CHECK(id == TASK_MONITOR_INS && op == TASK_MONITOR_LONG_OPERATION_FLASH && !mask); ++monitor_end; return monitor_end_ok; }

int main(int argc, char **argv)
{
    TEST_CHECK(argc == 2);
    available = (uint8_t)(strcmp(argv[1], "degraded") != 0); // 降级场景跳过温控。
    if (!strcmp(argv[1], "heater-refused")) heater_ok = 0u;
    if (!strcmp(argv[1], "monitor-refused")) monitor_ok = 0u;
    if (!strcmp(argv[1], "deadline")) monitor_end_ok = 0u;
    if (!strcmp(argv[1], "reentrant")) test_reentrant = 1u;
    TEST_CHECK(!RobotParamFlashOperationEnd(1u)); // 没有开始时不得调用结束保护。
    const uint8_t begun = RobotParamFlashOperationBegin();
    if (!heater_ok || !monitor_ok)
    {
        TEST_CHECK(!begun && monitor_end == 0u);
        TEST_CHECK(heater_end == (unsigned)(heater_ok && !monitor_ok));
        TEST_CHECK(!heater_end || heater_result == 0u); // 失败撤销保持禁热。
        return 0;
    }
    TEST_CHECK(begun && !RobotParamFlashOperationBegin()); // 重复开始不能覆盖已有保护。
    available = 0u; // 已取得保护时模拟模块在操作中降级。
    const uint8_t expected = (uint8_t)(monitor_end_ok && strcmp(argv[1], "write-failed"));
    TEST_CHECK(RobotParamFlashOperationEnd((uint8_t)(strcmp(argv[1], "write-failed") != 0)) == expected);
    TEST_CHECK(monitor_begin == 1u && monitor_end == 1u);
    TEST_CHECK(heater_begin == heater_end);
    TEST_CHECK(heater_end == (unsigned)(strcmp(argv[1], "degraded") != 0));
    if (heater_end) TEST_CHECK(heater_result == expected);
    TEST_CHECK(!RobotParamFlashOperationEnd(1u) && !mask); // 收尾不可重复且恢复原中断状态。
    return 0;
}
