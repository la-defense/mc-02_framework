/** @file platform_can_start_stm32.c
 * @brief 将逻辑 CAN 总线的启动请求交给 H723 后端。
 */
#include "../can/platform_can.h"
#include "board_can_stm32.h"
#include <stddef.h>

_Static_assert(CAN_MX_REGISTER_CNT > 0 && CAN_MX_REGISTER_CNT <= UINT8_MAX,
               "CAN endpoint capacity must fit the platform interface");

static PlatformCanFaultObserver_t platform_fault_observer; // 保存整车安装的长期观察者。
static void *platform_fault_context; // 保存观察者上下文。

// 底层句柄在这里转换为板级逻辑编号，不向业务暴露 HAL 类型。
static void PlatformCanReportFault(FDCAN_HandleTypeDef *handle, void *context)
{
    (void)context; // 使用只在启动阶段安装的静态上下文。
    for (unsigned index = PLATFORM_CAN_BUS_1; index <= PLATFORM_CAN_BUS_3; ++index)
    {
        const PlatformCanBus_t bus = (PlatformCanBus_t)index; // 逐条核对板级绑定。
        if (BoardCanResolve(bus) == handle && platform_fault_observer != NULL)
        {
            platform_fault_observer(bus, platform_fault_context); // 交付实际故障总线。
            return; // 同一控制器只允许对应一个逻辑总线。
        }
    }
}

uint8_t PlatformCanInstallFaultObserver(PlatformCanFaultObserver_t observer, void *context)
{
    if (observer == NULL || platform_fault_observer != NULL) return 0u; // 无效或重复安装不能覆盖现有观察者。
    platform_fault_context = context; // 启动阶段先保存上下文。
    platform_fault_observer = observer; // 再准备可调用的上层回调。
    if (CANInstallFaultObserver(PlatformCanReportFault, NULL)) return 1u; // 底层成功登记后启用观察。
    platform_fault_observer = NULL; // 失败时撤销本层安装。
    platform_fault_context = NULL; // 避免保留未生效的上下文。
    return 0u;
}

uint8_t PlatformCanGetFilterCapacity(PlatformCanBus_t bus, PlatformCanFilterCapacity_t *capacity)
{
    FDCAN_HandleTypeDef *handle = BoardCanResolve(bus); // 查询板级逻辑总线绑定。
    if (handle == NULL || handle->Instance == NULL || capacity == NULL) return 0u; // 失败时保留旧输出。
    const uint32_t standard = handle->Init.StdFiltersNbr; // 控制器实际配置的标准槽数。
    const uint32_t extended = handle->Init.ExtFiltersNbr; // 控制器实际配置的扩展槽数。
    const PlatformCanFilterCapacity_t candidate = {
        .standard = (uint8_t)(standard < CAN_MX_REGISTER_CNT ? standard : CAN_MX_REGISTER_CNT),
        .extended = (uint8_t)(extended < CAN_MX_REGISTER_CNT ? extended : CAN_MX_REGISTER_CNT),
    }; // 单类容量不能超过全局登记数量。
    *capacity = candidate; // 所有检查通过后才交付结果。
    return 1u;
}

uint8_t PlatformCanEndpointCapacity(void)
{
    return (uint8_t)CAN_MX_REGISTER_CNT; // 与驱动使用同一登记上限。
}

uint8_t PlatformCanStartBus(PlatformCanBus_t bus)
{
    FDCAN_HandleTypeDef *handle = BoardCanResolve(bus); // 查找此逻辑总线绑定的控制器。
    if (handle == NULL) return 0u; // 没有绑定时不启动任何硬件。
    return CANStartBus(handle); // 使用底层按总线独立的启动与失败处理。
}

uint8_t PlatformCanBusIsQuarantined(PlatformCanBus_t bus)
{
    FDCAN_HandleTypeDef *handle = BoardCanResolve(bus); // 通过板级表解析逻辑总线。
    return handle == NULL ? 1u : CANBusIsQuarantined(handle); // 未绑定也不能放行依赖模块。
}
