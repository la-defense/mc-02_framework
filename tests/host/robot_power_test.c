/** @file robot_power_test.c
 * @brief 使用生产电源和模块计划，测试阶段、故障影响和安全检查。
 */
#include "robot_power.h"
#include "robot_can_capacity.h"
#include "platform_irq.h"
#include "robot_safety.h"
#include "init_diagnostic.h"
#include "main.h"
#include "test_check.h"
#include <string.h>

GPIO_TypeDef power_test_gpio;
static uint32_t irq_mask, faults, writes, module_calls, failed_reports, degraded_reports;
static uint32_t shutdowns;
static uint32_t fail_modules;
static uint8_t estop;
static uint8_t reject_gpio_write;
static uint32_t writes_at_barrier;
PlatformIrqState_t PlatformIrqSaveAndDisable(void) { const uint32_t old = irq_mask; irq_mask = 1u; return old; }
void PlatformIrqRestore(PlatformIrqState_t saved) { irq_mask = saved; }
uint32_t RobotSafetyGetFaults(void) { TEST_CHECK(irq_mask); return faults; }
uint8_t EstopIsLatched(void) { TEST_CHECK(irq_mask); return estop; }
void BSP_SafetyStartupLatchOutputsOff(void)
{
    TEST_CHECK(irq_mask); // 核心失败必须在恢复中断前直接关断。
    power_test_gpio.ODR = 0u;
    ++shutdowns;
}
void RobotFaultReport(Robot_Fault_e fault) { TEST_CHECK(irq_mask); faults |= (uint32_t)fault; }
void RobotSafetyRequestDisable(void) { TEST_CHECK(irq_mask); }
void HAL_GPIO_WritePin(GPIO_TypeDef *port, uint16_t pin, uint32_t value)
{
    TEST_CHECK(irq_mask && port == GPIOC && value == GPIO_PIN_SET); // 开启必须与安全检查在同一临界区。
    if (!reject_gpio_write) port->ODR |= pin; // 模拟输出锁存，不能代替实际电压测量。
    writes |= pin;
}
void InitDiagnosticReportFailure(InitDiagnosticStage_e stage, uint8_t module,
                                 InitDiagnosticReason_e reason, uint16_t resource)
{ TEST_CHECK(stage == INIT_STAGE_APPLICATION && module == INIT_MODULE_ROBOT && reason == INIT_FAILURE_RESOURCE && resource > 0u); ++failed_reports; }
void InitDiagnosticReportDegraded(InitDiagnosticStage_e stage, uint8_t module,
                                  InitDiagnosticReason_e reason, uint16_t resource)
{ TEST_CHECK(stage == INIT_STAGE_APPLICATION && module == INIT_MODULE_ROBOT && reason == INIT_FAILURE_RESOURCE && resource > 0u); ++degraded_reports; }
static uint8_t start(void *context, unsigned module)
{
    (void)context;
    if (!RobotPowerModuleReady(module)) return 0u; // 模拟整车入口拒绝尚未供电的模块。
    module_calls |= ROBOT_MODULE_BIT(module);
    return (uint8_t)((fail_modules & ROBOT_MODULE_BIT(module)) == 0u);
}
static uint8_t barrier(void *context, const RobotModuleStartup_t *prior_state, uint32_t *failed)
{
    writes_at_barrier = writes; // 检查后阶段开启前的输出状态。
    return RobotPowerAfterDevices(context, prior_state, failed);
}

int main(int argc, char **argv)
{
    TEST_CHECK(argc == 2);
    power_test_gpio.MODER = (1u << 26u) | (1u << 28u) | (1u << 30u); // 三路均模拟为输出模式。
    RobotModulePlan_t plan = {.enabled = 15u, .required = 1u}; // 模块 0 为核心，3 为辅助。
    plan.dependencies[0] = ROBOT_MODULE_BIT(ROBOT_MODULE_IMU); // 核心依赖模块 1。
    RobotPowerRailConfig_t rails[PLATFORM_POWER_COUNT] = {
        {ROBOT_POWER_BEFORE_DEVICES, ROBOT_MODULE_BIT(ROBOT_MODULE_COMMAND)}, // 第一条供辅助模块。
        {ROBOT_POWER_AFTER_DEVICES, ROBOT_MODULE_BIT(ROBOT_MODULE_IMU)}, // 第二条供核心依赖。
        {ROBOT_POWER_OFF, 0u}, // 第三条保持关闭。
    };
    if (!strcmp(argv[1], "preflight-rejected") || !strcmp(argv[1], "preflight-shared"))
    {
        const RobotCanFilterDemand_t demand = {ROBOT_MODULE_COMMAND, PLATFORM_CAN_BUS_1, 1u, 0u}; // 辅助模块需要一个标准帧端点。
        const RobotCanFilterCapacity_t capacity = {0}; // 模拟没有剩余过滤器容量。
        RobotCanCapacityResult_t capacity_result;
        TEST_CHECK(RobotCanCapacityPlan(&plan, &demand, 1u, &capacity, &capacity_result) == ROBOT_MODULE_OK);
        const uint32_t rejected = capacity_result.rejected_modules; // 使用实际容量计划的拒绝结果。
        TEST_CHECK(rejected == ROBOT_MODULE_BIT(ROBOT_MODULE_COMMAND));
        if (!strcmp(argv[1], "preflight-shared"))
            rails[0].modules |= ROBOT_MODULE_BIT(ROBOT_MODULE_IMU); // 核心依赖与辅助模块共享电源。
        TEST_CHECK(RobotPowerBeginWithUnavailable(rails, &plan, rejected));
        TEST_CHECK(!RobotPowerModuleReady(ROBOT_MODULE_COMMAND)); // 共享电源也不能让被拒绝模块就绪。
        TEST_CHECK((writes & POWER_24V_1_Pin) ==
                   (!strcmp(argv[1], "preflight-shared") ? POWER_24V_1_Pin : 0u));
        RobotModuleStartup_t state;
        TEST_CHECK(RobotModuleStartPhased(&plan, RobotPowerDeferredModules(), barrier, start, NULL,
                                          &state) == ROBOT_MODULE_OK);
        TEST_CHECK((state.failed & rejected) && (state.started & ROBOT_MODULE_BIT(ROBOT_MODULE_SAFETY)));
        return 0;
    }
    if (!strcmp(argv[1], "preflight-invalid"))
    {
        TEST_CHECK(!RobotPowerBeginWithUnavailable(rails, &plan, ROBOT_MODULE_BIT(ROBOT_MODULE_IMU)));
        TEST_CHECK(!RobotPowerBeginWithUnavailable(rails, &plan, 1u << 30u));
        TEST_CHECK(writes == 0u && RobotPowerBegin(rails, &plan)); // 无效请求不能写输出或占用计划。
        return 0;
    }
    if (!strcmp(argv[1], "before-dependent-failure"))
    {
        plan.dependencies[ROBOT_MODULE_COMMUNICATION] = ROBOT_MODULE_BIT(ROBOT_MODULE_COMMAND);
        rails[1] = (RobotPowerRailConfig_t){ROBOT_POWER_BEFORE_DEVICES,
                                          ROBOT_MODULE_BIT(ROBOT_MODULE_COMMUNICATION)};
        power_test_gpio.MODER &= ~(3u << 28u); // 第一条辅助电源不可用。
        TEST_CHECK(RobotPowerBegin(rails, &plan));
        TEST_CHECK(writes == 0u && degraded_reports == 1u); // 下游独占电源不得开启。
        TEST_CHECK(!RobotPowerModuleReady(ROBOT_MODULE_COMMUNICATION));
        return 0;
    }
    if (!strcmp(argv[1], "all-off"))
    {
        memset(rails, 0, sizeof(rails)); // 模拟裸板生成的三路关闭计划。
        TEST_CHECK(RobotPowerBegin(rails, &plan) && RobotPowerDeferredModules() == 0u);
        RobotModuleStartup_t off_state;
        TEST_CHECK(RobotModuleStartPhased(&plan, 0u, barrier, start, NULL, &off_state) == ROBOT_MODULE_OK);
        TEST_CHECK(off_state.started == plan.enabled && writes == 0u); // 所有模块照常启动，不写电源。
        return 0;
    }
    if (!strcmp(argv[1], "before-core-failure"))
    {
        rails[0].modules |= ROBOT_MODULE_BIT(ROBOT_MODULE_IMU); // 同一路同时供核心和辅助。
        power_test_gpio.MODER &= ~(3u << 28u); // 模拟第一路输出未准备。
        TEST_CHECK(!RobotPowerBegin(rails, &plan)); // 共享电源影响核心时停止。
        TEST_CHECK(failed_reports == 1u && shutdowns == 1u && writes == 0u && !irq_mask);
        TEST_CHECK(!RobotPowerBegin(rails, &plan)); // 核心失败后不允许重试。
        return 0;
    }
    if (!strcmp(argv[1], "readback-failure"))
    {
        rails[0].modules |= ROBOT_MODULE_BIT(ROBOT_MODULE_IMU); // 此路同时影响核心。
        reject_gpio_write = 1u; // 模拟写入后输出锁存没有变高。
        TEST_CHECK(!RobotPowerBegin(rails, &plan));
        TEST_CHECK(failed_reports == 1u && shutdowns == 1u && power_test_gpio.ODR == 0u);
        TEST_CHECK(writes == POWER_24V_1_Pin); // 检测到核心失败后不能继续开启其他电源。
        return 0;
    }
    if (!strcmp(argv[1], "invalid-plan"))
    {
        rails[2].stage = ROBOT_POWER_AFTER_DEVICES; // 未声明使用者，不能开启。
        TEST_CHECK(!RobotPowerBegin(rails, &plan) && writes == 0u);
        return 0;
    }
    if (!strcmp(argv[1], "after-aux-failure"))
    {
        rails[0].modules = ROBOT_MODULE_BIT(ROBOT_MODULE_IMU); // 核心先取得设备前电源。
        rails[1].modules = ROBOT_MODULE_BIT(ROBOT_MODULE_COMMAND); // 辅助模块等待设备后电源。
        power_test_gpio.MODER &= ~(3u << 26u); // 模拟第二路输出不可用。
        TEST_CHECK(RobotPowerBegin(rails, &plan));
        RobotModuleStartup_t aux_state;
        TEST_CHECK(RobotModuleStartPhased(&plan, RobotPowerDeferredModules(), barrier, start, NULL,
                                          &aux_state) == ROBOT_MODULE_OK);
        TEST_CHECK(degraded_reports == 1u && (aux_state.failed & ROBOT_MODULE_BIT(ROBOT_MODULE_COMMAND)));
        TEST_CHECK((aux_state.started & ROBOT_MODULE_BIT(ROBOT_MODULE_SAFETY)) != 0u);
        return 0;
    }
    if (!strcmp(argv[1], "prior-aux-failure"))
    {
        rails[0] = (RobotPowerRailConfig_t){ROBOT_POWER_OFF, 0u}; // 第一阶段不需要电源。
        rails[1].modules = ROBOT_MODULE_BIT(ROBOT_MODULE_COMMUNICATION); // 后阶段只供模块 2。
        plan.dependencies[ROBOT_MODULE_COMMUNICATION] = ROBOT_MODULE_BIT(ROBOT_MODULE_COMMAND);
        fail_modules = ROBOT_MODULE_BIT(ROBOT_MODULE_COMMAND); // 独立辅助前置模块先失败。
        TEST_CHECK(RobotPowerBegin(rails, &plan));
        RobotModuleStartup_t aux_state;
        TEST_CHECK(RobotModuleStartPhased(&plan, RobotPowerDeferredModules(), barrier, start, NULL,
                                          &aux_state) == ROBOT_MODULE_OK);
        TEST_CHECK((aux_state.blocked & ROBOT_MODULE_BIT(ROBOT_MODULE_COMMUNICATION)) != 0u);
        TEST_CHECK((writes & POWER_24V_2_Pin) == 0u); // 全部使用者受阻时不打开后阶段电源。
        return 0;
    }
    if (!strcmp(argv[1], "before-aux-failure")) power_test_gpio.MODER &= ~(3u << 28u);
    if (!strcmp(argv[1], "after-core-failure")) power_test_gpio.MODER &= ~(3u << 26u);
    if (!strcmp(argv[1], "estop")) estop = 1u;
    if (!strcmp(argv[1], "fault")) faults = ROBOT_FAULT_RESET_LOCKOUT;
    TEST_CHECK(RobotPowerBegin(rails, &plan)); // 设备前辅助失败不会阻止其他模块。
    TEST_CHECK(!RobotPowerBegin(rails, &plan)); // 启动期间不自动重试电源。
    TEST_CHECK(RobotPowerDeferredModules() == ROBOT_MODULE_BIT(ROBOT_MODULE_IMU));
    if (!strcmp(argv[1], "shutdown-before-module"))
    {
        power_test_gpio.ODR &= ~POWER_24V_1_Pin; // 模拟安全关断发生在模块启动前。
        TEST_CHECK(!RobotPowerModuleReady(ROBOT_MODULE_COMMAND)); // 不能沿用之前的成功状态。
        RobotModuleStartup_t state;
        TEST_CHECK(RobotModuleStartPhased(&plan, RobotPowerDeferredModules(), barrier, start, NULL,
                                          &state) == ROBOT_MODULE_OK);
        TEST_CHECK((state.failed & ROBOT_MODULE_BIT(ROBOT_MODULE_COMMAND)) != 0u);
        TEST_CHECK((state.started & ROBOT_MODULE_BIT(ROBOT_MODULE_SAFETY)) != 0u);
        return 0;
    }
    if (estop || faults)
    {
        TEST_CHECK(writes == 0u && degraded_reports == 1u); // 禁止状态下没有电源写入。
    }
    RobotModuleStartup_t state;
    const RobotModuleStatus_t result = RobotModuleStartPhased(&plan, RobotPowerDeferredModules(),
                                                               barrier, start, NULL, &state);
    if (!strcmp(argv[1], "after-core-failure") || estop || faults)
    {
        TEST_CHECK(result == ROBOT_MODULE_CORE_FAILURE && state.failed_module == ROBOT_MODULE_IMU);
        TEST_CHECK(failed_reports == 1u && shutdowns == 1u && power_test_gpio.ODR == 0u); // 核心电源失败先关断并报告。
    }
    else
    {
        TEST_CHECK(result == ROBOT_MODULE_OK && (writes & POWER_24V_2_Pin));
        TEST_CHECK((writes_at_barrier & POWER_24V_2_Pin) == 0u); // 后阶段不会提前开启。
        if (!strcmp(argv[1], "before-aux-failure"))
            TEST_CHECK(degraded_reports == 1u && (state.failed & ROBOT_MODULE_BIT(ROBOT_MODULE_COMMAND)));
        else
            TEST_CHECK(state.started == 15u && writes_at_barrier == POWER_24V_1_Pin);
    }
    uint32_t ignored = 0u;
    TEST_CHECK(!RobotPowerAfterDevices(NULL, &state, &ignored)); // 阶段失败也不得自动重试。
    TEST_CHECK(!irq_mask && (writes & POWER_5V_Pin) == 0u); // 恢复中断，关闭的电源未写入。
    return 0;
}
