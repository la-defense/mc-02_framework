/** @file chassis_supercap_legacy.c
 * @brief 保留旧超级电容的 MC-02 CAN 绑定，供现有底盘启动检查使用。
 */
#include "chassis_supercap_legacy.h"
#include "super_cap.h"
#include <stddef.h>

static SuperCapInstance *legacy_cap; // 仅保存本适配器创建的实例，避免撤销其他使用者。

uint8_t ChassisLegacySuperCapInit(void)
{
    SuperCap_Init_Config_s config = {
        .can_config = {
            .can_handle = &hcan2, // 原有第二条 CAN 总线。
            .tx_id = 0x302, // 原有发送标识，设备协议尚待确认。
            .rx_id = 0x301, // 原有接收标识，设备协议尚待确认。
        },
    };
    if (legacy_cap != NULL) return 0u; // 不覆盖此前已经登记的实例。
    legacy_cap = SuperCapInit(&config); // 保存本次成功申请的实例。
    return (uint8_t)(legacy_cap != NULL); // 未取得设备实例则拒绝底盘启动。
}

uint8_t ChassisLegacySuperCapRollbackStartup(void)
{
    if (legacy_cap == NULL) return 1u; // 未创建实例时没有资源需要撤销。
    const uint8_t closed = SuperCapRollbackStartup(legacy_cap); // 软件实例无论硬件结果均会释放。
    legacy_cap = NULL; // 丢弃可能已经失效的实例指针。
    return closed; // 硬件失败交由整车锁存故障。
}
