/** @file ins_startup_test.c
 * @brief 使用生产 INS 启动与温控失败处理检查车型核心和辅助依赖。
 */
#include "ins_startup.h"
#include "config/vehicle_selection.h"
#include "init_diagnostic.h"
#include "test_check.h"
#include <stdint.h>
#include <string.h>

static uint8_t ins_ok = 1u, heater_enabled = 1u, heater_ok = 1u;
static RobotModule_t core_failure = ROBOT_MODULE_COUNT;
static uint32_t ins_calls, force_off, faults, disable, shutdowns, failures, degraded;
static RobotModule_t failed_module = ROBOT_MODULE_COUNT;
static InitDiagnosticStage_e reported_stage;
static uint8_t reported_module;
static InitDiagnosticReason_e reported_reason;
static uint16_t reported_resource;

void *INS_Init(void) { ++ins_calls; return ins_ok ? &ins_calls : NULL; }
uint8_t IMUHeaterInitSucceeded(void) { return heater_ok; }
void IMUHeaterForceOff(void) { ++force_off; }
uint8_t RobotBuildModuleStarted(RobotModule_t module)
{
    return (uint8_t)(module == ROBOT_MODULE_HEATER && heater_enabled); // 只模拟本测试需要的启用状态。
}
uint8_t RobotBuildMarkFailure(RobotModule_t module)
{
    failed_module = module; // 保留依赖图收到的故障来源。
    return (uint8_t)(module == core_failure); // 模拟该故障是否影响核心依赖。
}
void InitDiagnosticReportFailure(InitDiagnosticStage_e stage, uint8_t module,
                                 InitDiagnosticReason_e reason, uint16_t resource)
{
    ++failures; reported_stage = stage; reported_module = module;
    reported_reason = reason; reported_resource = resource; // 保留完整诊断供断言。
}
void InitDiagnosticReportDegraded(InitDiagnosticStage_e stage, uint8_t module,
                                  InitDiagnosticReason_e reason, uint16_t resource)
{
    ++degraded; reported_stage = stage; reported_module = module;
    reported_reason = reason; reported_resource = resource; // 保留降级原因供断言。
}
void RobotFaultReport(unsigned fault) { TEST_CHECK(fault == 1u); ++faults; }
void RobotSafetyRequestDisable(void) { ++disable; }
void BSP_SafetyStartupLatchOutputsOff(void) { ++shutdowns; }

int main(int argc, char **argv)
{
    TEST_CHECK(argc == 2); // 每个场景独立进程，避免静态状态相互影响。
    RobotInsStartupResult_e expected = ROBOT_INS_STARTUP_READY;
    if (strcmp(argv[1], "imu-core") == 0)
    { ins_ok = 0u; core_failure = ROBOT_MODULE_IMU; expected = ROBOT_INS_STARTUP_CORE_FAILURE; }
    else if (strcmp(argv[1], "imu-aux") == 0)
    { ins_ok = 0u; expected = ROBOT_INS_STARTUP_AUXILIARY_FAILURE; }
    else if (strcmp(argv[1], "heater-core") == 0)
    { heater_ok = 0u; core_failure = ROBOT_MODULE_HEATER; expected = ROBOT_INS_STARTUP_CORE_FAILURE; }
    else if (strcmp(argv[1], "heater-aux") == 0) heater_ok = 0u;
    else if (strcmp(argv[1], "heater-disabled") == 0) { heater_enabled = 0u; heater_ok = 0u; }
    else TEST_CHECK(strcmp(argv[1], "success") == 0);

    TEST_CHECK(RobotInitializeIns() == expected && ins_calls == 1u); // 每次启动只初始化一次。
    const uint8_t is_core = (uint8_t)(expected == ROBOT_INS_STARTUP_CORE_FAILURE);
    const uint8_t is_aux = (uint8_t)(!ins_ok ? !is_core : heater_enabled && !heater_ok && !is_core);
    TEST_CHECK(failures == is_core && degraded == is_aux); // 核心与辅助结果不能混淆。
    TEST_CHECK(faults == is_core && disable == is_core && shutdowns == is_core); // 只有核心失败关断整车。
    TEST_CHECK(force_off == (unsigned)(is_core || (heater_enabled && !heater_ok))); // 温控失败必须关闭加热。
    if (is_core || is_aux)
    {
        TEST_CHECK(reported_stage == INIT_STAGE_APPLICATION && reported_module == INIT_MODULE_INS);
        TEST_CHECK(reported_reason == INIT_FAILURE_RESOURCE && reported_resource == 0u);
        TEST_CHECK(failed_module == (ins_ok ? ROBOT_MODULE_HEATER : ROBOT_MODULE_IMU));
    }
    return 0;
}
