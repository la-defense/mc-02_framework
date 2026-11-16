/** @file robot_can_runtime.c
 * @brief 复用车型启动依赖，给核心 CAN 故障建立即时安全禁止。
 */
#include "robot_can_runtime.h"
#include "platform_can.h"
#include "robot_safety.h"
#include <stddef.h>

static uint8_t core_can_buses; // 每位对应一条已声明的核心逻辑总线。
static uint8_t observer_installed; // 不允许运行时覆盖已有故障绑定。

// 此回调在短中断保护内执行，安全管理只更新有界状态，不打印日志。
static void RobotCanRuntimeFault(PlatformCanBus_t bus, void *context)
{
    (void)context; // 车型依赖在启动时已复制到静态位图。
    if (bus < PLATFORM_CAN_BUS_1 || bus > PLATFORM_CAN_BUS_3) return; // 无效编号不能索引位图。
    const uint8_t bit = (uint8_t)(1u << ((unsigned)bus - (unsigned)PLATFORM_CAN_BUS_1)); // 转成该总线的固定位置。
    if (core_can_buses & bit) RobotFaultReport(ROBOT_FAULT_CAN_BUSOFF); // 核心依赖发生故障时立即撤销旧许可。
}

uint8_t RobotCanRuntimeInstall(const RobotModulePlan_t *plan,
                               const RobotCanFilterDemand_t *demands, uint8_t count)
{
    if (plan == NULL || count > 8u || (count != 0u && demands == NULL) || observer_installed) return 0u; // 启动声明必须完整且不能覆盖旧观察者。
    if (RobotModulePlanValidate(plan, NULL) != ROBOT_MODULE_OK) return 0u; // 无效依赖图不能用于判断核心。
    const uint32_t core = RobotModuleCoreClosure(plan); // 包含核心模块及其全部必需依赖。
    uint8_t buses = 0u; // 先在局部构造完整位图。
    for (uint8_t i = 0u; i < count; ++i)
    {
        const RobotCanFilterDemand_t demand = demands[i]; // 读取一个模块的实际总线需求。
        if ((unsigned)demand.module >= ROBOT_MODULE_COUNT || !(plan->enabled & ROBOT_MODULE_BIT(demand.module)) ||
            demand.bus < PLATFORM_CAN_BUS_1 || demand.bus > PLATFORM_CAN_BUS_3 || !PlatformCanBusIsBound(demand.bus)) return 0u; // 非法需求不能被静默忽略。
        if (core & ROBOT_MODULE_BIT(demand.module))
            buses |= (uint8_t)(1u << ((unsigned)demand.bus - (unsigned)PLATFORM_CAN_BUS_1)); // 核心模块或依赖使用的总线均受保护。
    }
    if (buses == 0u) return 1u; // 没有核心总线时无需登记全局禁止回调。
    core_can_buses = buses; // 安装前先准备观察者使用的只读位图。
    if (!PlatformCanInstallFaultObserver(RobotCanRuntimeFault, NULL))
    {
        core_can_buses = 0u; // 安装失败不留下看似生效的配置。
        return 0u; // 启动协调者必须按核心失败处理。
    }
    observer_installed = 1u; // 只在平台成功登记后标记已安装。
    return 1u;
}
