/**
 * @file bsp_flash_invalid_layout_test.c
 * @brief 用错误的链接符号验证生产 Flash 驱动会拒绝不匹配的分区表。
 *
 * 测试只调用只读布局查询和范围入口；模拟 HAL 计数确认非法布局不会擦写。
 */
#include "bsp_flash.h"
#include "main.h"

#include <stdio.h>
#include <stdlib.h>

static unsigned int erase_calls;

HAL_StatusTypeDef HAL_FLASH_Unlock(void) { return HAL_OK; }
void HAL_FLASH_Lock(void) {}
HAL_StatusTypeDef HAL_FLASHEx_Erase(FLASH_EraseInitTypeDef *erase, uint32_t *error)
{
    (void)erase;
    (void)error;
    erase_calls++;
    return HAL_OK;
}
HAL_StatusTypeDef HAL_FLASH_Program(uint32_t type, uint32_t address, uint32_t data_address)
{
    (void)type;
    (void)address;
    (void)data_address;
    return HAL_OK;
}
void SCB_InvalidateDCache_by_Addr(uint32_t *address, int32_t size)
{
    (void)address;
    (void)size;
}
uint32_t HAL_GetTick(void) { return 0u; }
uint32_t __get_IPSR(void) { return 0u; }

int main(void)
{
    BSPFlashPartitionInfo_t info = {.start_address = 0x1234u, .size_bytes = 0x5678u};
    if (BSPFlashGetPartitionInfo(BSP_FLASH_PARTITION_PARAMETER_B, &info) !=
            BSP_FLASH_LAYOUT_INVALID ||
        info.start_address != 0x1234u || info.size_bytes != 0x5678u)
    {
        fputs("invalid linker layout was accepted or overwrote query output\n", stderr);
        return EXIT_FAILURE;
    }
    if (BSPFlashRead(BSP_FLASH_PARTITION_APPLICATION, 0u, NULL, 0u) !=
        BSP_FLASH_LAYOUT_INVALID)
    {
        fputs("invalid linker layout was accepted for read\n", stderr);
        return EXIT_FAILURE;
    }
    if (erase_calls != 0u)
    {
        fputs("invalid linker layout reached HAL erase\n", stderr);
        return EXIT_FAILURE;
    }
    puts("PASS: production Flash driver rejects invalid linker layout");
    return EXIT_SUCCESS;
}
