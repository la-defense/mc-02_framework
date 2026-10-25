/**
 * @file bsp_flash.h
 * @brief 提供 STM32H723 应用区与参数分区的只读／受保护访问。
 *
 * 应用区可读但不允许本接口擦写；参数 A/B 区按分区和字节偏移访问。
 * 参数保存由 bsp_param 统一执行，并在 MC02-0036 维护保护成功后取得内部授权。
 */
#ifndef MC02_BSP_FLASH_H
#define MC02_BSP_FLASH_H

#include <stddef.h>
#include <stdint.h>

#define BSP_FLASH_MAINTENANCE_TIMEOUT_MS 8000u

/** H723 固件使用的 Flash 分区。 */
typedef enum
{
    BSP_FLASH_PARTITION_APPLICATION = 0,
    BSP_FLASH_PARTITION_PARAMETER_A,
    BSP_FLASH_PARTITION_PARAMETER_B,
    BSP_FLASH_PARTITION_COUNT
} BSPFlashPartition_e;

/** 驱动操作状态；失败时查询输出保持原值。 */
typedef enum
{
    BSP_FLASH_OK = 0,
    BSP_FLASH_BAD_ARGUMENT,
    BSP_FLASH_BAD_PARTITION,
    BSP_FLASH_LAYOUT_INVALID,
    BSP_FLASH_OUT_OF_RANGE,
    BSP_FLASH_ALIGNMENT_ERROR,
    BSP_FLASH_READ_ONLY,
    BSP_FLASH_UNAUTHORIZED,
    BSP_FLASH_TIMEOUT,
    BSP_FLASH_HAL_ERROR,
    BSP_FLASH_HAL_BUSY,
    BSP_FLASH_HAL_TIMEOUT,
    BSP_FLASH_CONTEXT_ERROR
} BSPFlashStatus_e;

/** 分区布局的只读副本，地址与大小均为字节单位。 */
typedef struct
{
    uintptr_t start_address;
    size_t size_bytes;
    size_t erase_granularity_bytes;
    size_t program_granularity_bytes;
    uint8_t writable;
} BSPFlashPartitionInfo_t;

/* 仅供 bsp_param 使用的短期维护授权；具体定义放在内部头文件。 */
typedef struct BSPFlashAuthorization BSPFlashAuthorization_t;

/**
 * @brief 复制一个已由链接脚本定义并校验的分区布局。
 * @param[in] partition 分区枚举，允许 APPLICATION、PARAMETER_A、PARAMETER_B。
 * @param[out] info 接收布局；失败时保持原值。
 * @return BSP_FLASH_OK 或具体错误。
 */
BSPFlashStatus_e BSPFlashGetPartitionInfo(BSPFlashPartition_e partition,
                                         BSPFlashPartitionInfo_t *info);

/**
 * @brief 从已声明分区复制字节。
 * @param[in] partition 分区枚举。
 * @param[in] offset 分区内字节偏移；offset==分区长度时只允许 len==0。
 * @param[out] data 接收区，len>0 时须可写 len 字节。
 * @param[in] len 字节数；允许0，不访问 data。
 * @return BSP_FLASH_OK 或具体错误；参数／范围错误时不改 data。
 */
BSPFlashStatus_e BSPFlashRead(BSPFlashPartition_e partition, size_t offset,
                              void *data, size_t len);

/**
 * @brief 擦除参数分区中的完整扇区。
 * @param[in] partition 只允许 PARAMETER_A 或 PARAMETER_B。
 * @param[in] offset 分区内偏移，必须按擦除粒度对齐。
 * @param[in] len 字节数，必须是完整擦除粒度的整数倍。
 * @param[in] authorization 由参数层在维护准入成功后持有的短期授权。
 * @return BSP_FLASH_OK 或具体错误；超时／HAL错误时目标区可能已部分擦除。
 * @note 仅普通任务上下文调用；授权总期限从外层维护操作开始计算，最多8秒。
 */
BSPFlashStatus_e BSPFlashErase(BSPFlashPartition_e partition, size_t offset,
                               size_t len,
                               const BSPFlashAuthorization_t *authorization);

/**
 * @brief 按 Flash word 编程参数分区。
 * @param[in] partition 只允许 PARAMETER_A 或 PARAMETER_B。
 * @param[in] offset 分区内字节偏移，须满足32字节编程对齐。
 * @param[in] data RAM 源数据，须32字节对齐且至少可读 len 字节；不能来自或覆盖 H723 Flash。
 * @param[in] len 字节数，须为32的正整数倍且不能越过分区。
 * @param[in] authorization 由参数层在维护准入成功后持有的短期授权。
 * @return BSP_FLASH_OK 或具体错误；失败时已编程的前缀不会自动回滚。
 * @note 仅普通任务上下文调用；同一授权下的擦除和所有编程共用8秒总期限。
 */
BSPFlashStatus_e BSPFlashProgram(BSPFlashPartition_e partition, size_t offset,
                                 const void *data, size_t len,
                                 const BSPFlashAuthorization_t *authorization);

#endif
