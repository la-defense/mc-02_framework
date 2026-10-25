/**
 * @file bsp_flash_internal.h
 * @brief 只供参数维护实现使用的 Flash 授权生命周期接口。
 *
 * 普通模块只包含 bsp_flash.h，不能创建授权对象。授权须在参数层的安全回调
 * 成功后开启，并在同一次维护事务结束时关闭。
 */
#ifndef MC02_BSP_FLASH_INTERNAL_H
#define MC02_BSP_FLASH_INTERNAL_H

#include "bsp_flash.h"

struct BSPFlashAuthorization
{
    uint32_t generation;
    uint32_t started_at_ms;
    uint32_t seal;
};

/** 在安全准入前取时间戳，使底层期限不晚于 MC02-0036 外层期限。 */
uint32_t BSPFlashMaintenanceGetTick(void);

/** 参数层在调用任务级维护回调前检查当前是否处于普通任务。 */
uint8_t BSPFlashMaintenanceIsTaskContext(void);

/** 仅在 heater／TaskMonitor 准入成功后由 bsp_param 调用。 */
BSPFlashStatus_e BSPFlashAuthorizationOpen(BSPFlashAuthorization_t *authorization,
                                           uint32_t operation_started_at_ms);

/** 关闭授权；若总期限已到，返回 BSP_FLASH_TIMEOUT。 */
BSPFlashStatus_e BSPFlashAuthorizationClose(BSPFlashAuthorization_t *authorization);

#endif
