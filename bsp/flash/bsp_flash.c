/**
 * @file bsp_flash.c
 * @brief 检查 H723 链接分区并提供带维护授权的参数 Flash 操作。
 *
 * 参数区边界来自链接脚本；擦除和编程只接受参数维护层发出的短期授权。
 * 所有地址使用分区内字节偏移，硬件写入仅从 bsp_param 的普通任务路径进入。
 */
#include "bsp_flash.h"
#include "bsp_flash_internal.h"
#include "main.h"

#include <limits.h>
#include <string.h>

#define BSP_FLASH_BASE_ADDRESS 0x08000000u
#define BSP_FLASH_DEVICE_END 0x08100000u
#define BSP_FLASH_APPLICATION_END 0x080C0000u
#define BSP_FLASH_PARAMETER_A_START 0x080C0000u
#define BSP_FLASH_PARAMETER_B_START 0x080E0000u
#define BSP_FLASH_PARAMETER_END 0x08100000u
#define BSP_FLASH_PARAMETER_SECTOR_SIZE 0x20000u
#define BSP_FLASH_PROGRAM_GRANULARITY 32u
#define BSP_FLASH_CACHE_LINE_SIZE 32u
#define BSP_FLASH_AUTHORIZATION_SEAL 0x46373348u

extern uint8_t __mc02_flash_start__[];
extern uint8_t __mc02_flash_end__[];
extern uint8_t __mc02_param_a_start__[];
extern uint8_t __mc02_param_a_end__[];
extern uint8_t __mc02_param_b_start__[];
extern uint8_t __mc02_param_b_end__[];

static BSPFlashAuthorization_t *s_active_authorization;
static uint32_t s_authorization_generation;

static BSPFlashStatus_e BSPFlashBuildLayout(BSPFlashPartitionInfo_t layout[BSP_FLASH_PARTITION_COUNT])
{
    const uintptr_t app_start = (uintptr_t)__mc02_flash_start__;
    const uintptr_t app_end = (uintptr_t)__mc02_flash_end__;
    const uintptr_t param_a_start = (uintptr_t)__mc02_param_a_start__;
    const uintptr_t param_a_end = (uintptr_t)__mc02_param_a_end__;
    const uintptr_t param_b_start = (uintptr_t)__mc02_param_b_start__;
    const uintptr_t param_b_end = (uintptr_t)__mc02_param_b_end__;

    /* 拒绝与 H723VGTx 1MiB 布局不符的链接结果，防止参数擦写碰到程序区。 */
    if (app_start != BSP_FLASH_BASE_ADDRESS || app_end != BSP_FLASH_APPLICATION_END ||
        param_a_start != BSP_FLASH_PARAMETER_A_START ||
        param_a_end != BSP_FLASH_PARAMETER_B_START ||
        param_b_start != BSP_FLASH_PARAMETER_B_START ||
        param_b_end != BSP_FLASH_PARAMETER_END || param_b_end != BSP_FLASH_DEVICE_END)
        return BSP_FLASH_LAYOUT_INVALID;

    layout[BSP_FLASH_PARTITION_APPLICATION] = (BSPFlashPartitionInfo_t){
        .start_address = app_start,
        .size_bytes = (size_t)(app_end - app_start),
        .erase_granularity_bytes = BSP_FLASH_PARAMETER_SECTOR_SIZE,
        .program_granularity_bytes = BSP_FLASH_PROGRAM_GRANULARITY,
        .writable = 0u,
    };
    layout[BSP_FLASH_PARTITION_PARAMETER_A] = (BSPFlashPartitionInfo_t){
        .start_address = param_a_start,
        .size_bytes = (size_t)(param_a_end - param_a_start),
        .erase_granularity_bytes = BSP_FLASH_PARAMETER_SECTOR_SIZE,
        .program_granularity_bytes = BSP_FLASH_PROGRAM_GRANULARITY,
        .writable = 1u,
    };
    layout[BSP_FLASH_PARTITION_PARAMETER_B] = (BSPFlashPartitionInfo_t){
        .start_address = param_b_start,
        .size_bytes = (size_t)(param_b_end - param_b_start),
        .erase_granularity_bytes = BSP_FLASH_PARAMETER_SECTOR_SIZE,
        .program_granularity_bytes = BSP_FLASH_PROGRAM_GRANULARITY,
        .writable = 1u,
    };
    return BSP_FLASH_OK;
}

static BSPFlashStatus_e BSPFlashGetLayout(BSPFlashPartition_e partition,
                                           BSPFlashPartitionInfo_t *info)
{
    BSPFlashPartitionInfo_t layout[BSP_FLASH_PARTITION_COUNT];
    if (info == NULL)
        return BSP_FLASH_BAD_ARGUMENT;
    if ((unsigned)partition >= BSP_FLASH_PARTITION_COUNT)
        return BSP_FLASH_BAD_PARTITION;

    BSPFlashStatus_e status = BSPFlashBuildLayout(layout);
    if (status == BSP_FLASH_OK)
        *info = layout[partition];
    return status;
}

static BSPFlashStatus_e BSPFlashCheckRange(const BSPFlashPartitionInfo_t *info,
                                           size_t offset, size_t len)
{
    if (offset > info->size_bytes || len > info->size_bytes - offset)
        return BSP_FLASH_OUT_OF_RANGE;
    return BSP_FLASH_OK;
}

static uint8_t BSPFlashSourceOverlapsDevice(uintptr_t source_start, size_t len)
{
    if (len > (size_t)(UINTPTR_MAX - source_start))
        return 1u;
    const uintptr_t source_end = source_start + len;
    return (source_start < BSP_FLASH_DEVICE_END && source_end > BSP_FLASH_BASE_ADDRESS) ?
           1u : 0u;
}

static BSPFlashStatus_e BSPFlashMapHalStatus(HAL_StatusTypeDef status)
{
    if (status == HAL_BUSY)
        return BSP_FLASH_HAL_BUSY;
    if (status == HAL_TIMEOUT)
        return BSP_FLASH_HAL_TIMEOUT;
    return (status == HAL_OK) ? BSP_FLASH_OK : BSP_FLASH_HAL_ERROR;
}

static void BSPFlashInvalidateCache(uintptr_t address, size_t len)
{
    if (len == 0u)
        return;

    const uintptr_t aligned_start = address & ~(uintptr_t)(BSP_FLASH_CACHE_LINE_SIZE - 1u);
    const uintptr_t raw_end = address + len;
    const uintptr_t aligned_end = (raw_end + BSP_FLASH_CACHE_LINE_SIZE - 1u) &
                                  ~(uintptr_t)(BSP_FLASH_CACHE_LINE_SIZE - 1u);
    const size_t aligned_len = (size_t)(aligned_end - aligned_start);
    /* H723 分区边界均按缓存行对齐；无效化覆盖读写区间所在的完整缓存行。 */
    SCB_InvalidateDCache_by_Addr((uint32_t *)aligned_start, (int32_t)aligned_len);
}

static BSPFlashStatus_e BSPFlashCheckAuthorization(const BSPFlashAuthorization_t *authorization)
{
    if (__get_IPSR() != 0u)
        return BSP_FLASH_CONTEXT_ERROR;
    if (authorization == NULL || authorization != s_active_authorization ||
        authorization->seal != BSP_FLASH_AUTHORIZATION_SEAL ||
        authorization->generation != s_authorization_generation)
        return BSP_FLASH_UNAUTHORIZED;
    if ((uint32_t)(HAL_GetTick() - authorization->started_at_ms) >=
        BSP_FLASH_MAINTENANCE_TIMEOUT_MS)
        return BSP_FLASH_TIMEOUT;
    return BSP_FLASH_OK;
}

BSPFlashStatus_e BSPFlashGetPartitionInfo(BSPFlashPartition_e partition,
                                          BSPFlashPartitionInfo_t *info)
{
    if (info == NULL)
        return BSP_FLASH_BAD_ARGUMENT;
    BSPFlashPartitionInfo_t candidate;
    BSPFlashStatus_e status = BSPFlashGetLayout(partition, &candidate);
    if (status == BSP_FLASH_OK)
        *info = candidate;
    return status;
}

BSPFlashStatus_e BSPFlashRead(BSPFlashPartition_e partition, size_t offset,
                              void *data, size_t len)
{
    BSPFlashPartitionInfo_t info;
    if (len > 0u && data == NULL)
        return BSP_FLASH_BAD_ARGUMENT;

    BSPFlashStatus_e status = BSPFlashGetLayout(partition, &info);
    if (status != BSP_FLASH_OK)
        return status;
    status = BSPFlashCheckRange(&info, offset, len);
    if (status != BSP_FLASH_OK || len == 0u)
        return status;

    const uintptr_t address = info.start_address + offset;
    BSPFlashInvalidateCache(address, len);
    memcpy(data, (const void *)address, len);
    return BSP_FLASH_OK;
}

uint32_t BSPFlashMaintenanceGetTick(void)
{
    return HAL_GetTick();
}

uint8_t BSPFlashMaintenanceIsTaskContext(void)
{
    return (__get_IPSR() == 0u) ? 1u : 0u;
}

BSPFlashStatus_e BSPFlashAuthorizationOpen(BSPFlashAuthorization_t *authorization,
                                           uint32_t operation_started_at_ms)
{
    BSPFlashPartitionInfo_t layout[BSP_FLASH_PARTITION_COUNT];
    if (authorization == NULL)
        return BSP_FLASH_BAD_ARGUMENT;
    if (__get_IPSR() != 0u)
        return BSP_FLASH_CONTEXT_ERROR;
    if (s_active_authorization != NULL)
        return BSP_FLASH_UNAUTHORIZED;
    if (BSPFlashBuildLayout(layout) != BSP_FLASH_OK)
        return BSP_FLASH_LAYOUT_INVALID;
    if ((uint32_t)(HAL_GetTick() - operation_started_at_ms) >=
        BSP_FLASH_MAINTENANCE_TIMEOUT_MS)
        return BSP_FLASH_TIMEOUT;

    s_authorization_generation++;
    if (s_authorization_generation == 0u)
        s_authorization_generation++;
    authorization->generation = s_authorization_generation;
    authorization->started_at_ms = operation_started_at_ms;
    authorization->seal = BSP_FLASH_AUTHORIZATION_SEAL;
    s_active_authorization = authorization;
    return BSP_FLASH_OK;
}

BSPFlashStatus_e BSPFlashAuthorizationClose(BSPFlashAuthorization_t *authorization)
{
    BSPFlashStatus_e status = BSPFlashCheckAuthorization(authorization);
    if (authorization == NULL || authorization != s_active_authorization)
        return status;

    s_active_authorization = NULL;
    authorization->seal = 0u;
    authorization->generation = 0u;
    authorization->started_at_ms = 0u;
    return status;
}

BSPFlashStatus_e BSPFlashErase(BSPFlashPartition_e partition, size_t offset,
                               size_t len,
                               const BSPFlashAuthorization_t *authorization)
{
    BSPFlashPartitionInfo_t info;
    BSPFlashStatus_e status = BSPFlashGetLayout(partition, &info);
    if (status != BSP_FLASH_OK)
        return status;
    if (info.writable == 0u)
        return BSP_FLASH_READ_ONLY;
    if (len == 0u)
        return BSP_FLASH_BAD_ARGUMENT;
    status = BSPFlashCheckRange(&info, offset, len);
    if (status != BSP_FLASH_OK)
        return status;
    if ((offset % info.erase_granularity_bytes) != 0u ||
        (len % info.erase_granularity_bytes) != 0u)
        return BSP_FLASH_ALIGNMENT_ERROR;
    status = BSPFlashCheckAuthorization(authorization);
    if (status != BSP_FLASH_OK)
        return status;

    HAL_StatusTypeDef hal_status = HAL_FLASH_Unlock();
    if (hal_status != HAL_OK)
    {
        HAL_FLASH_Lock();
        return BSPFlashMapHalStatus(hal_status);
    }

    status = BSPFlashCheckAuthorization(authorization);
    if (status != BSP_FLASH_OK)
    {
        HAL_FLASH_Lock();
        return status;
    }

    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_ALL_ERRORS_BANK1);
    FLASH_EraseInitTypeDef erase = {0};
    erase.TypeErase = FLASH_TYPEERASE_SECTORS;
    erase.Banks = FLASH_BANK_1;
    erase.Sector = (partition == BSP_FLASH_PARTITION_PARAMETER_A) ?
                   FLASH_SECTOR_6 : FLASH_SECTOR_7;
    erase.NbSectors = (uint32_t)(len / info.erase_granularity_bytes);
    erase.VoltageRange = FLASH_VOLTAGE_RANGE_3;
    uint32_t sector_error = 0u;
    hal_status = HAL_FLASHEx_Erase(&erase, &sector_error);
    HAL_FLASH_Lock();
    if (hal_status == HAL_OK)
        BSPFlashInvalidateCache(info.start_address + offset, len);

    status = BSPFlashCheckAuthorization(authorization);
    if (status != BSP_FLASH_OK)
        return status;
    return BSPFlashMapHalStatus(hal_status);
}

BSPFlashStatus_e BSPFlashProgram(BSPFlashPartition_e partition, size_t offset,
                                 const void *data, size_t len,
                                 const BSPFlashAuthorization_t *authorization)
{
    BSPFlashPartitionInfo_t info;
    if (data == NULL || len == 0u)
        return BSP_FLASH_BAD_ARGUMENT;
    if (BSPFlashSourceOverlapsDevice((uintptr_t)data, len))
        return BSP_FLASH_BAD_ARGUMENT;

    BSPFlashStatus_e status = BSPFlashGetLayout(partition, &info);
    if (status != BSP_FLASH_OK)
        return status;
    if (info.writable == 0u)
        return BSP_FLASH_READ_ONLY;
    status = BSPFlashCheckRange(&info, offset, len);
    if (status != BSP_FLASH_OK)
        return status;
    if ((offset % info.program_granularity_bytes) != 0u ||
        (len % info.program_granularity_bytes) != 0u ||
        (((uintptr_t)data) % info.program_granularity_bytes) != 0u)
        return BSP_FLASH_ALIGNMENT_ERROR;
    status = BSPFlashCheckAuthorization(authorization);
    if (status != BSP_FLASH_OK)
        return status;

    HAL_StatusTypeDef hal_status = HAL_FLASH_Unlock();
    if (hal_status != HAL_OK)
    {
        HAL_FLASH_Lock();
        return BSPFlashMapHalStatus(hal_status);
    }

    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_ALL_ERRORS_BANK1);
    const uintptr_t destination = info.start_address + offset;
    for (size_t written = 0u; written < len; written += info.program_granularity_bytes)
    {
        status = BSPFlashCheckAuthorization(authorization);
        if (status != BSP_FLASH_OK)
            break;

        hal_status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_FLASHWORD,
                                       (uint32_t)(destination + written),
                                       (uint32_t)((uintptr_t)data + written));
        if (hal_status != HAL_OK)
        {
            status = BSPFlashMapHalStatus(hal_status);
            break;
        }
        BSPFlashInvalidateCache(destination + written, info.program_granularity_bytes);
    }
    HAL_FLASH_Lock();

    BSPFlashStatus_e final_status = BSPFlashCheckAuthorization(authorization);
    if (final_status != BSP_FLASH_OK)
        return final_status;
    return status;
}
