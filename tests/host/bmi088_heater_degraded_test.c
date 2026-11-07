/** @file bmi088_heater_degraded_test.c
 * @brief 运行生产标定流程，验证辅助温控降级不会阻止 IMU 标定。
 */
#include "bmi088_host_fixture.h"
#include "config/vehicle_selection.h"
#include "test_check.h"
#include <string.h>

static uint8_t available, drop_after_query;

uint8_t RobotBuildModuleStarted(RobotModule_t module)
{
    TEST_CHECK(module == ROBOT_MODULE_HEATER); // 本测试只查询温控模块。
    const uint8_t result = available; // 返回请求开始时的可用状态。
    if (drop_after_query) available = 0u; // 模拟保护取得后模块发生降级。
    return result;
}

int main(int argc, char **argv)
{
    if (argc > 1)
    {
        TEST_CHECK(strcmp(argv[1], "--lost-during-operation") == 0);
        available = 1u; // 首次查询时温控仍可用。
        drop_after_query = 1u; // 后续查询反映运行中降级。
    }
    BMI088HostReset(); // 清除采样与长操作替身状态。
    BMI088HostSetKernelRunning(1u); // 标定在调度器运行后执行。
    TEST_CHECK(BMI088HostRegisterOnline() != NULL); // 没有温控仍可完成静止标定。
    TEST_CHECK(bmi088_host_fixture.calibration_monitor_begin_count == 1u);
    TEST_CHECK(bmi088_host_fixture.calibration_monitor_end_count == 1u);
    TEST_CHECK(bmi088_host_fixture.calibration_heater_begin_count == (unsigned)(argc > 1));
    TEST_CHECK(bmi088_host_fixture.calibration_heater_end_count == (unsigned)(argc > 1));
    TEST_CHECK(BMI088CalibRequest() && BMI088CalibService()); // 人工请求仍走有界任务监控。
    TEST_CHECK(bmi088_host_fixture.calibration_monitor_begin_count == 2u);
    TEST_CHECK(bmi088_host_fixture.calibration_monitor_end_count == 2u);
    TEST_CHECK(bmi088_host_fixture.calibration_heater_begin_count == (unsigned)(argc > 1));
    TEST_CHECK(bmi088_host_fixture.calibration_heater_end_count == (unsigned)(argc > 1));
    bmi088_host_fixture.calibration_monitor_allow = 0u; // 模拟任务监控拒绝新的标定窗口。
    const uint32_t before = bmi088_host_fixture.transfer_count;
    TEST_CHECK(BMI088CalibRequest() && BMI088CalibService());
    TEST_CHECK(BMI088CalibGetState() == BMI088_RECALIB_FAIL);
    TEST_CHECK(bmi088_host_fixture.transfer_count == before); // 拒绝后不读取传感器。
    TEST_CHECK(bmi088_host_fixture.calibration_heater_begin_count == (unsigned)(argc > 1));
    return 0;
}
