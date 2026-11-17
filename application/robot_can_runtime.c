/** @file robot_can_runtime.c
 * @brief 复用车型启动依赖，给核心 CAN 故障建立即时安全禁止。
 */
#include "robot_can_runtime.h"
#include "platform_can.h"
#include "robot_safety.h"
#include "platform_irq.h"
#include <stddef.h>

static uint8_t core_can_buses; // 每位对应一条已声明的核心逻辑总线。
static uint8_t observer_installed; // 不允许运行时覆盖已有故障绑定。
static uint8_t fault_can_buses; // 记录从安装以来发生故障的核心总线。
static uint32_t fault_generation; // 任一核心总线事件都会推进此代次。
static uint32_t bus_generation[3]; // 每条核心总线单独记录事件代次。

uint8_t RobotCanRuntimeGetFaultSnapshot(RobotCanFaultSnapshot_t *output)
{
    if (output == NULL) return 0u; // 没有输出存储时不执行读取。
    const PlatformIrqState_t irq = PlatformIrqSaveAndDisable(); // 一次复制总线集合和全部事件代次。
    if (!observer_installed)
    {
        PlatformIrqRestore(irq); // 尚未安装时恢复中断，不改变调用者的旧结果。
        return 0u;
    }
    RobotCanFaultSnapshot_t result = {0}; // 先构造完整的本地结果。
    result.core_bus_mask = core_can_buses; // 保留车型声明的核心总线。
    result.fault_bus_mask = fault_can_buses; // 报告安装后观察到的故障总线。
    result.fault_generation = fault_generation; // 任一核心总线的最新事件代次。
    for (unsigned i = 0u; i < 3u; ++i) result.bus_generation[i] = bus_generation[i]; // 各总线分别复制。
    PlatformIrqRestore(irq); // 恢复调用者的中断状态。
    *output = result; // 成功后才交付完整结果。
    return 1u;
}

uint8_t RobotCanRuntimeGetReadiness(RobotCanReadiness_t *output)
{
    if (output == NULL) return 0u; // 无输出空间时不读取硬件。
    RobotCanFaultSnapshot_t before = {0}; // 保存一轮查询开始前的总线和故障代次。
    if (!RobotCanRuntimeGetFaultSnapshot(&before)) return 0u; // 观察者未安装时不能声称总线受保护。
    uint8_t ready = 0u; // 尚未检查的总线都按不可用处理。
    for (unsigned index = 0u; index < 3u; ++index)
    {
        const uint8_t bit = (uint8_t)(1u << index); // 固定映射到第一至第三条逻辑总线。
        if ((before.core_bus_mask & bit) == 0u) continue; // 非核心总线不影响此结果。
        const PlatformCanBus_t bus = (PlatformCanBus_t)((unsigned)PLATFORM_CAN_BUS_1 + index); // 恢复板级逻辑编号。
        if (PlatformCanGetStatus(bus) == PLATFORM_CAN_STATUS_OK) ready |= bit; // 只有控制器状态明确正常才记录。
    }
    RobotCanFaultSnapshot_t after = {0}; // 再读取全部核心总线的故障事件。
    if (!RobotCanRuntimeGetFaultSnapshot(&after)) return 0u; // 无法确认时不交付部分结果。
    if (before.fault_generation != after.fault_generation || before.core_bus_mask != after.core_bus_mask)
        ready = 0u; // 查询期间发生短故障时撤销整轮观察。
    const RobotCanReadiness_t candidate = {after.core_bus_mask, ready, after.fault_generation}; // 保留调用者后续要核对的代次。
    *output = candidate; // 先完整检查再交付结果。
    return 1u;
}

// 此回调在短中断保护内执行，安全管理只更新有界状态，不打印日志。
static void RobotCanRuntimeFault(PlatformCanBus_t bus, void *context)
{
    (void)context; // 车型依赖在启动时已复制到静态位图。
    if (bus < PLATFORM_CAN_BUS_1 || bus > PLATFORM_CAN_BUS_3) return; // 无效编号不能索引位图。
    const uint8_t bit = (uint8_t)(1u << ((unsigned)bus - (unsigned)PLATFORM_CAN_BUS_1)); // 转成该总线的固定位置。
    if (core_can_buses & bit)
    {
        fault_can_buses |= bit; // 保留该核心总线曾发生故障的记录。
        ++fault_generation; // 即使故障位已置位，新的事件也不能丢失。
        ++bus_generation[(unsigned)bus - (unsigned)PLATFORM_CAN_BUS_1]; // 保留这条总线自己的事件代次。
        RobotFaultReport(ROBOT_FAULT_CAN_BUSOFF); // 核心依赖发生故障时立即撤销旧许可。
    }
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
