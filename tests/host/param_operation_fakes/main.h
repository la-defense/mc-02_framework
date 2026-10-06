#ifndef MC02_PARAM_OPERATION_FAKE_MAIN_H
#define MC02_PARAM_OPERATION_FAKE_MAIN_H

#include <stdint.h>

typedef enum
{
    HAL_OK = 0,
    HAL_ERROR = 1,
    HAL_BUSY = 2,
    HAL_TIMEOUT = 3
} HAL_StatusTypeDef;

typedef struct
{
    uint32_t TypeErase;
    uint32_t Banks;
    uint32_t Sector;
    uint32_t NbSectors;
    uint32_t VoltageRange;
} FLASH_EraseInitTypeDef;

#define FLASH_FLAG_ALL_ERRORS_BANK1 0xFFFFFFFFu
#define FLASH_TYPEERASE_SECTORS 0u
#define FLASH_BANK_1 1u
#define FLASH_SECTOR_6 6u
#define FLASH_SECTOR_7 7u
#define FLASH_VOLTAGE_RANGE_3 3u
#define FLASH_TYPEPROGRAM_FLASHWORD 1u

HAL_StatusTypeDef HAL_FLASH_Unlock(void);
void HAL_FLASH_Lock(void);
HAL_StatusTypeDef HAL_FLASHEx_Erase(FLASH_EraseInitTypeDef *erase, uint32_t *sector_error);
HAL_StatusTypeDef HAL_FLASH_Program(uint32_t type, uint32_t address, uint32_t data_address);
void SCB_InvalidateDCache_by_Addr(uint32_t *address, int32_t size);

#define __HAL_FLASH_CLEAR_FLAG(flags) ((void)(flags))

#endif
