/** @file robot_monitor_plan_test.c
 * @brief 用生产车型依赖算法验证姿态任务监控的核心、辅助和禁用分类。
 */
#include "robot_monitor_plan.h"
#include "test_check.h"
#include <string.h>

static TaskMonitorTaskConfig_t captured[TASK_MONITOR_MAX_TASKS]; // 保存传入监控模块的完整配置。
static uint32_t captured_count, init_calls;
static uint32_t degraded_calls;
static RobotModule_t degraded_module = ROBOT_MODULE_COUNT;

static PlatformPeriodicCapabilities_t fake_periodic_capabilities = {
    .rtos_quantum_us = 1000u, // 模拟现有 RTOS 的 1 ms 刻度。
    .high_res_quantum_us = 1u, // 模拟 TIM5 的 1 μs 刻度。
    .high_res_channels = 4u, // 模拟裸板启动时空闲的四个比较通道。
};

PlatformPeriodicCapabilities_t PlatformTaskPeriodicCapabilities(void)
{
    return fake_periodic_capabilities;
}

#if !MC02_TASK_CHASSIS
static void TestDiagnosticAdmission(void)
{
    RobotModulePlan_t modules = {.enabled = ROBOT_MODULE_BIT(ROBOT_MODULE_IMU)};
    RobotPeriodicTask_t failed = ROBOT_PERIODIC_COUNT;
    TEST_CHECK(RobotValidateTaskPeriods(&modules, &failed) == PLATFORM_PERIODIC_READY && failed == ROBOT_PERIODIC_COUNT);
    TEST_CHECK(RobotGetTaskPeriod(ROBOT_PERIODIC_DIAGNOSTIC)->period_us == 5000u);
    TEST_CHECK(RobotGetTaskPeriod(ROBOT_PERIODIC_INS)->period_us == 1000u);
    TEST_CHECK(RobotGetTaskPeriod(ROBOT_PERIODIC_COMM)->period_us == 1000u);
    TEST_CHECK(RobotGetTaskPeriod(ROBOT_PERIODIC_DAEMON)->period_us == 10000u);
    TEST_CHECK(RobotGetTaskPeriod(ROBOT_PERIODIC_TIM5_PROBE)->period_us == 500u);
    TEST_CHECK(RobotGetTaskPeriod(ROBOT_PERIODIC_COUNT) == NULL);

    fake_periodic_capabilities.high_res_channels = 0u; // 板级资源被占用，显式定时自测不能悄悄跳过。
    fake_periodic_capabilities.rtos_quantum_us = 100u; // 即使 RTOS 也能表示自测周期，仍需检验 TIM5。
    TEST_CHECK(RobotValidateTaskPeriods(&modules, &failed) == PLATFORM_PERIODIC_RESOURCE && failed == ROBOT_PERIODIC_TIM5_PROBE);
    fake_periodic_capabilities.high_res_channels = 4u;
    fake_periodic_capabilities.rtos_quantum_us = 2000u; // 普通任务必须使用 RTOS，不能用空闲 TIM5 掩盖配置错误。
    TEST_CHECK(RobotValidateTaskPeriods(&modules, &failed) == PLATFORM_PERIODIC_CONFIG && failed == ROBOT_PERIODIC_DIAGNOSTIC);
    fake_periodic_capabilities.rtos_quantum_us = 5000u; // 此刻度可表示 5 ms，不能表示 1 ms。
    TEST_CHECK(RobotValidateTaskPeriods(&modules, &failed) == PLATFORM_PERIODIC_CONFIG && failed == ROBOT_PERIODIC_INS);
    modules.enabled = 0u; // 车型禁用姿态时不校验姿态的拍点，压缩索引仍须指向通信任务。
    TEST_CHECK(RobotValidateTaskPeriods(&modules, &failed) == PLATFORM_PERIODIC_CONFIG && failed == ROBOT_PERIODIC_COMM);
}
#endif

uint8_t RobotBuildMarkFailure(RobotModule_t module)
{
    ++degraded_calls; degraded_module = module; // 模拟依赖图接收任务故障。
    return 0u; // 辅助场景不会触及核心依赖。
}

TaskMonitor_Result_e TaskMonitorInitConfigured(const TaskMonitorTaskConfig_t *configs, uint32_t count)
{
    TEST_CHECK(configs != NULL && count <= TASK_MONITOR_MAX_TASKS); // 复制前检查容量。
    memcpy(captured, configs, count * sizeof(captured[0])); // 不保留调用者的栈地址。
    captured_count = count; // 记录实际提交项数。
    ++init_calls; // 记录是否调用了监控初始化。
    return TASK_MONITOR_RESULT_OK;
}

#if MC02_TASK_CHASSIS
static void TestChassisAdmission(void)
{
    RobotModulePlan_t plan = {
        .enabled = ROBOT_MODULE_BIT(ROBOT_MODULE_SAFETY) | ROBOT_MODULE_BIT(ROBOT_MODULE_CHASSIS),
        .required = ROBOT_MODULE_BIT(ROBOT_MODULE_SAFETY) | ROBOT_MODULE_BIT(ROBOT_MODULE_CHASSIS),
    };
    RobotControlTaskConfig_t configs[ROBOT_CONTROL_TASK_COUNT] = {{
        .timing = {.period_us = 4000u, .budget_us = 2000u}, // 主机假时钟的周期和单轮预算。
        .source = PLATFORM_PERIODIC_SOURCE_RTOS, // 先检查仅允许 RTOS 的配置。
        .deadline_ms = 12u, // 主机试验用监控期限。
        .startup_grace_ms = 50u, // 主机试验用首次工作宽限。
        .stack_words = 192u, // 替身不分配或运行真实栈。
    }};
    RobotControlTaskConfig_t *chassis = &configs[ROBOT_CONTROL_TASK_CHASSIS];
    RobotControlTimingFailure_t failure;
    TEST_CHECK(RobotConfigureTaskMonitor(&plan, NULL, &failure) == TASK_MONITOR_RESULT_BAD_CONFIGURATION);
    TEST_CHECK(failure.task == ROBOT_CONTROL_TASK_CHASSIS && failure.reason == PLATFORM_PERIODIC_CONFIG);
    TEST_CHECK(init_calls == 0u && RobotGetControlTaskConfig(ROBOT_CONTROL_TASK_CHASSIS) == NULL);
    chassis->timing.budget_us = 2100u; // 配置超出 RTOS 可表示的刻度。
    TEST_CHECK(RobotConfigureTaskMonitor(&plan, configs, NULL) == TASK_MONITOR_RESULT_BAD_CONFIGURATION);
    chassis->timing.budget_us = 2000u;
    chassis->deadline_ms = 3u; // 监控期限短于一个计划周期。
    TEST_CHECK(RobotConfigureTaskMonitor(&plan, configs, NULL) == TASK_MONITOR_RESULT_BAD_CONFIGURATION);
    TEST_CHECK(init_calls == 0u);
    chassis->deadline_ms = 12u;

    TEST_CHECK(RobotConfigureTaskMonitor(&plan, configs, NULL) == TASK_MONITOR_RESULT_OK);
    TEST_CHECK(init_calls == 1u && captured_count == 5u);
    const TaskMonitorTaskConfig_t *item = &captured[captured_count - 1u];
    TEST_CHECK(item->id == TASK_MONITOR_CHASSIS && item->role == TASK_MONITOR_ROLE_CORE);
    TEST_CHECK(item->period_ms == 4u && item->deadline_ms == 12u && item->startup_grace_ms == 50u);
    const RobotControlTaskConfig_t *prepared = RobotGetControlTaskConfig(ROBOT_CONTROL_TASK_CHASSIS);
    TEST_CHECK(prepared != NULL && prepared != chassis && prepared->stack_words == 192u);
    chassis->timing.period_us = 0u;
    TEST_CHECK(prepared->timing.period_us == 4000u); // 调用者临时数据改写后，任务仍使用已经接受的值。

    chassis->timing = (PlatformPeriodicConfig_t){.period_us = 500u, .budget_us = 250u}; // 主机测试精确到微秒的拍点。
    TEST_CHECK(RobotConfigureTaskMonitor(&plan, configs, &failure) == TASK_MONITOR_RESULT_BAD_CONFIGURATION);
    TEST_CHECK(failure.reason == PLATFORM_PERIODIC_CONFIG); // 强制 RTOS 时不改用硬件。
    chassis->source = PLATFORM_PERIODIC_SOURCE_AUTO; // 自动模式才允许后备的硬件来源。
    fake_periodic_capabilities.high_res_channels = 0u; // 模拟定时器已被占用。
    TEST_CHECK(RobotConfigureTaskMonitor(&plan, configs, &failure) == TASK_MONITOR_RESULT_BAD_CONFIGURATION);
    TEST_CHECK(failure.reason == PLATFORM_PERIODIC_RESOURCE);
    TEST_CHECK(init_calls == 1u && prepared->source == PLATFORM_PERIODIC_SOURCE_RTOS); // 保留上次成功的副本。
    fake_periodic_capabilities.high_res_channels = 1u;
    TEST_CHECK(RobotConfigureTaskMonitor(&plan, configs, NULL) == TASK_MONITOR_RESULT_OK);
    TEST_CHECK(prepared->source == PLATFORM_PERIODIC_SOURCE_HIGH_RES && prepared->timing.period_us == 500u);
    TEST_CHECK(captured[captured_count - 1u].period_ms == 1u); // 只在监控中向上取整。
    chassis->timing = (PlatformPeriodicConfig_t){.period_us = 4000u, .budget_us = 2000u};
    TEST_CHECK(RobotConfigureTaskMonitor(&plan, configs, NULL) == TASK_MONITOR_RESULT_OK);
    TEST_CHECK(prepared->source == PLATFORM_PERIODIC_SOURCE_RTOS); // 两种来源都可用时不占硬件。

    plan.enabled = plan.required = ROBOT_MODULE_BIT(ROBOT_MODULE_SAFETY);
    TEST_CHECK(RobotConfigureTaskMonitor(&plan, NULL, NULL) == TASK_MONITOR_RESULT_OK);
    TEST_CHECK(captured_count == 4u && RobotGetControlTaskConfig(ROBOT_CONTROL_TASK_CHASSIS) == NULL);
}
#endif

#if MC02_TASK_GIMBAL && MC02_TASK_SHOOTER
static void TestSharedControlChannels(void)
{
    RobotModulePlan_t plan = {
        .enabled = ROBOT_MODULE_BIT(ROBOT_MODULE_SAFETY) | ROBOT_MODULE_BIT(ROBOT_MODULE_CHASSIS) |
            ROBOT_MODULE_BIT(ROBOT_MODULE_GIMBAL) | ROBOT_MODULE_BIT(ROBOT_MODULE_SHOOTER),
        .required = ROBOT_MODULE_BIT(ROBOT_MODULE_SAFETY),
    };
    RobotControlTaskConfig_t configs[ROBOT_CONTROL_TASK_COUNT];
    for (unsigned i = 0u; i < ROBOT_CONTROL_TASK_COUNT; ++i)
        configs[i] = (RobotControlTaskConfig_t){
            .timing = {500u, 200u}, // 亚毫秒测试需求只能由 TIM5 表示。
            .source = PLATFORM_PERIODIC_SOURCE_AUTO, // 对同一资源表检查允许的来源。
            .deadline_ms = 3u, // 毫秒监控期限与微秒拍点分开。
            .startup_grace_ms = 20u, // 主机测试的放行宽限。
            .stack_words = 256u, // 测试配置，仅用于验证准入。
        };
    fake_periodic_capabilities.high_res_channels = 2u;
    RobotControlTimingFailure_t failure;
    TEST_CHECK(RobotConfigureTaskMonitor(&plan, configs, &failure) == TASK_MONITOR_RESULT_BAD_CONFIGURATION);
    TEST_CHECK(failure.task == ROBOT_CONTROL_TASK_SHOOTER && failure.reason == PLATFORM_PERIODIC_RESOURCE);
    TEST_CHECK(init_calls == 0u && RobotGetControlTaskConfig(ROBOT_CONTROL_TASK_GIMBAL) == NULL);

    configs[ROBOT_CONTROL_TASK_CHASSIS].timing = (PlatformPeriodicConfig_t){2000u, 1000u}; // 底盘可用 RTOS，为另外两项让出通道。
    TEST_CHECK(RobotConfigureTaskMonitor(&plan, configs, &failure) == TASK_MONITOR_RESULT_OK);
    TEST_CHECK(captured_count == 7u && captured[4].id == TASK_MONITOR_CHASSIS &&
        captured[5].id == TASK_MONITOR_GIMBAL && captured[6].id == TASK_MONITOR_SHOOTER);
    TEST_CHECK(RobotGetControlTaskConfig(ROBOT_CONTROL_TASK_CHASSIS)->source == PLATFORM_PERIODIC_SOURCE_RTOS);
    TEST_CHECK(RobotGetControlTaskConfig(ROBOT_CONTROL_TASK_GIMBAL)->source == PLATFORM_PERIODIC_SOURCE_HIGH_RES);
    TEST_CHECK(RobotGetControlTaskConfig(ROBOT_CONTROL_TASK_SHOOTER)->source == PLATFORM_PERIODIC_SOURCE_HIGH_RES);
    configs[ROBOT_CONTROL_TASK_GIMBAL].stack_words = 0u; // 重新检查失败不能破坏已公布的整组结果。
    TEST_CHECK(RobotConfigureTaskMonitor(&plan, configs, &failure) == TASK_MONITOR_RESULT_BAD_CONFIGURATION);
    TEST_CHECK(failure.task == ROBOT_CONTROL_TASK_GIMBAL && RobotGetControlTaskConfig(ROBOT_CONTROL_TASK_GIMBAL)->stack_words == 256u);
}
#endif

int main(int argc, char **argv)
{
    TEST_CHECK(argc == 2); // 每个车型依赖场景分别执行。
#if MC02_TASK_GIMBAL && MC02_TASK_SHOOTER
    if (strcmp(argv[1], "shared-control") == 0)
    {
        TestSharedControlChannels();
        return 0;
    }
#endif
#if MC02_TASK_CHASSIS
    if (strcmp(argv[1], "chassis") == 0)
    {
        TestChassisAdmission();
        return 0;
    }
#else
    if (strcmp(argv[1], "timing") == 0)
    {
        TestDiagnosticAdmission(); // 核对实际固件使用的配置和资源分配规则。
        return 0;
    }
#endif
    const uint32_t safety = ROBOT_MODULE_BIT(ROBOT_MODULE_SAFETY);
    const uint32_t imu = ROBOT_MODULE_BIT(ROBOT_MODULE_IMU);
    const uint32_t gimbal = ROBOT_MODULE_BIT(ROBOT_MODULE_GIMBAL);
    RobotModulePlan_t plan = {0}; // 未声明的模块和依赖均为空。
    plan.enabled = safety | imu; // 默认启用核心安全与姿态。
    plan.required = safety; // 默认只要求安全模块。
    plan.dependencies[ROBOT_MODULE_IMU] = safety; // 姿态依赖安全模块。
    uint8_t expected_ins = 1u, expected_core = 0u, invalid = 0u;
    if (strcmp(argv[1], "core") == 0) { plan.required |= imu; expected_core = 1u; }
    else if (strcmp(argv[1], "transitive") == 0)
    {
        plan.enabled |= gimbal; plan.required |= gimbal; // 核心云台传递依赖姿态。
        plan.dependencies[ROBOT_MODULE_GIMBAL] = imu; expected_core = 1u;
    }
    else if (strcmp(argv[1], "disabled") == 0)
    { plan.enabled = safety; plan.dependencies[ROBOT_MODULE_IMU] = 0u; expected_ins = 0u; }
    else if (strcmp(argv[1], "missing") == 0)
    { plan.enabled = imu; plan.required = imu; invalid = 1u; }
    else if (strcmp(argv[1], "cycle") == 0)
    { plan.dependencies[ROBOT_MODULE_SAFETY] = imu; invalid = 1u; }
    else TEST_CHECK(strcmp(argv[1], "aux") == 0);

    const TaskMonitor_Result_e result = RobotConfigureTaskMonitor(&plan, NULL, NULL); // 执行生产映射。
    if (invalid)
    {
        TEST_CHECK(result == TASK_MONITOR_RESULT_BAD_CONFIGURATION && init_calls == 0u);
        return 0; // 无效计划不能提交部分监控配置。
    }
    TEST_CHECK(result == TASK_MONITOR_RESULT_OK && init_calls == 1u);
    TEST_CHECK(captured_count == (uint32_t)(4u + expected_ins)); // 其他四项核心监控始终保留。
    uint8_t found_ins = 0u;
    for (uint32_t index = 0u; index < captured_count; ++index)
    {
        const TaskMonitorTaskConfig_t *item = &captured[index]; // 逐项检查复制后的配置。
        if (item->id == TASK_MONITOR_INS) TEST_CHECK(item->period_ms * 1000u == RobotGetTaskPeriod(ROBOT_PERIODIC_INS)->period_us);
        if (item->id == TASK_MONITOR_MOTOR) TEST_CHECK(item->period_ms * 1000u == RobotGetTaskPeriod(ROBOT_PERIODIC_COMM)->period_us);
        if (item->id == TASK_MONITOR_ROBOT) TEST_CHECK(item->period_ms * 1000u == RobotGetTaskPeriod(ROBOT_PERIODIC_DIAGNOSTIC)->period_us);
        if (item->id == TASK_MONITOR_DAEMON) TEST_CHECK(item->period_ms * 1000u == RobotGetTaskPeriod(ROBOT_PERIODIC_DAEMON)->period_us);
        if (item->id != TASK_MONITOR_INS) { TEST_CHECK(item->role == TASK_MONITOR_ROLE_CORE); continue; }
        found_ins = 1u; // 姿态任务是否存在取决于车型启用状态。
        TEST_CHECK(item->role == (expected_core ? TASK_MONITOR_ROLE_CORE : TASK_MONITOR_ROLE_AUXILIARY));
        TEST_CHECK(item->dependency_mask == (expected_core ? 0u : imu)); // 辅助任务记录受影响模块。
        if (!expected_core)
        {
            TEST_CHECK(item->degrade_callback != NULL); // 辅助任务超时须传递到车型依赖图。
            item->degrade_callback(TASK_MONITOR_INS, imu, item->callback_context);
            TEST_CHECK(degraded_calls == 1u && degraded_module == ROBOT_MODULE_IMU);
        }
    }
    TEST_CHECK(found_ins == expected_ins);
    return 0;
}
