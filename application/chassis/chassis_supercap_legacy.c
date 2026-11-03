/** @file chassis_supercap_legacy.c
 * @brief 保留旧超级电容的 MC-02 CAN 绑定，供现有底盘启动检查使用。
 */
#include "chassis_supercap_legacy.h"
#include "super_cap.h"
#include <stddef.h>

uint8_t ChassisLegacySuperCapInit(void)
{
    SuperCap_Init_Config_s config = {
        .can_config = {
            .can_handle = &hcan2, // 原有第二条 CAN 总线。
            .tx_id = 0x302, // 原有发送标识，设备协议尚待确认。
            .rx_id = 0x301, // 原有接收标识，设备协议尚待确认。
        },
    };
    return (uint8_t)(SuperCapInit(&config) != NULL); // 未取得设备实例则拒绝底盘启动。
}
