/** @file robot_can_runtime_test.c
 * @brief 用生产安全管理验证核心与辅助 CAN 故障的不同影响。
 */
#include "robot_can_runtime.h"
#include "platform_can.h"
#include "robot_safety.h"
#include "test_check.h"

static PlatformCanFaultObserver_t installed_observer; // 保存平台安装的观察者供故障注入。
static void *installed_context; // 回调时传回相同上下文。
static uint8_t reject_observer_install; // 模拟平台无法安装观察者。
static uint32_t now_ms; // 本机测试时钟。

uint32_t HAL_GetTick(void) { return now_ms; } // 平台时基使用主机测试时间。
uint8_t PlatformCanBusIsBound(PlatformCanBus_t bus)
{
    return (uint8_t)(bus >= PLATFORM_CAN_BUS_1 && bus <= PLATFORM_CAN_BUS_3); // 测试板声明三条逻辑总线。
}
uint8_t PlatformCanInstallFaultObserver(PlatformCanFaultObserver_t observer, void *context)
{
    if (observer == NULL || installed_observer != NULL || reject_observer_install) return 0u; // 不接受空、重复或被注入拒绝的安装。
    installed_context = context; // 先保存上下文。
    installed_observer = observer; // 使故障注入能调用生产观察者。
    return 1u;
}

int main(void)
{
    RobotSafetyInit(); // 建立真实安全管理状态。
    RobotSafetySetRcOnline(1u); // 人工控制所需遥控在线。
    RobotSafetySetImuValid(1u); // 核心姿态有效。
    RobotSafetySetCalibValid(1u); // 核心标定有效。
    RobotSafetySetTaskHealthy(1u); // 核心任务健康。
    RobotSafetyRequestEnable(); RobotSafetyUpdate(); // 操作者请求使能。
    TEST_CHECK(RobotSafetyIsReady()); // 故障前整车具备运行条件。
    RobotModulePlan_t plan = {0}; // 构造隔离的测试车型依赖。
    plan.enabled = ROBOT_MODULE_BIT(ROBOT_MODULE_CHASSIS) | ROBOT_MODULE_BIT(ROBOT_MODULE_ALARM); // 底盘和辅助模块均启用。
    plan.required = ROBOT_MODULE_BIT(ROBOT_MODULE_CHASSIS); // 只有底盘属于核心。
    const RobotCanFilterDemand_t demands[] = {
        {ROBOT_MODULE_CHASSIS, PLATFORM_CAN_BUS_1, 4u, 0u}, // 第一条总线承载核心轮电机。
        {ROBOT_MODULE_ALARM, PLATFORM_CAN_BUS_2, 1u, 0u}, // 第二条总线只承载辅助设备。
    };
    TEST_CHECK(!RobotCanRuntimeInstall(NULL, demands, 2u)); // 空计划不能建立故障绑定。
    TEST_CHECK(!RobotCanRuntimeInstall(&plan, NULL, 1u)); // 需求数组缺失必须拒绝。
    RobotCanFilterDemand_t invalid = {ROBOT_MODULE_CHASSIS, PLATFORM_CAN_INVALID, 1u, 0u}; // 非法总线不能被忽略。
    TEST_CHECK(!RobotCanRuntimeInstall(&plan, &invalid, 1u)); // 失败后仍可安装有效配置。
    invalid = (RobotCanFilterDemand_t){ROBOT_MODULE_GIMBAL, PLATFORM_CAN_BUS_1, 1u, 0u}; // 未启用模块不能声明有效需求。
    TEST_CHECK(!RobotCanRuntimeInstall(&plan, &invalid, 1u)); // 不接受与车型计划不一致的需求。
    reject_observer_install = 1u; // 模拟平台拒绝观察者安装。
    TEST_CHECK(!RobotCanRuntimeInstall(&plan, demands, 2u) && installed_observer == NULL); // 启动者必须收到失败，不能假定保护已接入。
    reject_observer_install = 0u; // 后续验证成功安装。
    TEST_CHECK(RobotCanRuntimeInstall(&plan, demands, 2u)); // 安装真实车型依赖转换。
    TEST_CHECK(installed_observer != NULL); // 核心总线必须建立观察者。
    installed_observer(PLATFORM_CAN_BUS_2, installed_context); // 辅助总线故障不应全局失能。
    TEST_CHECK(RobotSafetyIsReady()); // 核心许可保持有效。
    installed_observer(PLATFORM_CAN_BUS_1, installed_context); // 核心总线故障立即通知。
    TEST_CHECK(!RobotSafetyIsReady()); // 无须等下一次状态更新即撤销许可。
    TEST_CHECK((RobotSafetyGetFaults() & ROBOT_FAULT_CAN_BUSOFF) != 0u); // 故障原因可查询。
    RobotSafetyUpdate(); // 控制器恢复不能自行清除故障。
    TEST_CHECK(!RobotSafetyIsReady()); // 在恢复流程接入前保持禁止。
    TEST_CHECK(!RobotCanRuntimeInstall(&plan, demands, 2u)); // 不允许运行中重置观察者。
    return 0;
}
