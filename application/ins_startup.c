/** @file ins_startup.c
 * @brief 将 INS 初始化结果映射到车型依赖、启动诊断和核心关断。
 */
#include "ins_startup.h"
#include "config/build_selection.h"
#if defined(MC02_CONFIG_GENERATED)
#include "config/vehicle_selection.h"
#endif
#include "ins_task.h"
#include "init_diagnostic.h"
#include "robot_safety.h"
#include "bsp_safety.h"
#if MC02_FEATURE_HEATER
#include "imu_heater.h"
#include "robot_task_create.h"
#endif
#include <stddef.h>

// 核心初始化失败统一记录整车故障，再直接关闭板级受控输出。
static void RobotInsLatchCoreFailure(void)
{
    RobotFaultReport(ROBOT_FAULT_INIT_FAILURE); // 保存核心初始化失败。
    RobotSafetyRequestDisable(); // 撤销业务控制许可。
    BSP_SafetyStartupLatchOutputsOff(); // 不依赖任务调度器关断受控输出。
}

RobotInsStartupResult_e RobotInitializeIns(void)
{
    if (INS_Init() == NULL)
    {
#if defined(MC02_CONFIG_GENERATED)
        if (!RobotBuildMarkFailure(ROBOT_MODULE_IMU))
        {
            InitDiagnosticReportDegraded(INIT_STAGE_APPLICATION, INIT_MODULE_INS,
                                         INIT_FAILURE_RESOURCE, 0u); // 辅助 INS 失败不关断其他核心功能。
            return ROBOT_INS_STARTUP_AUXILIARY_FAILURE; // 调用者不得创建失效的姿态任务。
        }
#endif
        InitDiagnosticReportFailure(INIT_STAGE_APPLICATION, INIT_MODULE_INS,
                                    INIT_FAILURE_RESOURCE, 0u); // 记录核心 INS 失败。
#if MC02_FEATURE_HEATER
        IMUHeaterForceOff(); // 温控存在时先撤销加热输出。
#endif
        RobotInsLatchCoreFailure(); // 使用统一的核心失能路径。
        return ROBOT_INS_STARTUP_CORE_FAILURE; // 调用者不得放行控制任务。
    }

#if MC02_FEATURE_HEATER
#if defined(MC02_CONFIG_GENERATED)
    if (RobotBuildModuleStarted(ROBOT_MODULE_HEATER) && !IMUHeaterInitSucceeded())
#else
    if (!IMUHeaterInitSucceeded())
#endif
    {
        IMUHeaterForceOff(); // 温控失败时立即关闭该输出。
        if (RobotReportHeaterInitFailure())
        {
            RobotInsLatchCoreFailure(); // 温控影响核心依赖时关闭整车。
            return ROBOT_INS_STARTUP_CORE_FAILURE; // 不将该状态当作 INS 可运行。
        }
    }
#endif
    return ROBOT_INS_STARTUP_READY; // INS 已初始化；运行许可仍由安全管理决定。
}
