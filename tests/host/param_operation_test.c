#define _GNU_SOURCE
#include "bsp_param.h"
#include "main.h"
#include "bsp_watchdog.h"
#include "task_monitor.h"
#include "imu_heater.h"

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>

#define FLASH_BASE 0x080c0000u
#define FLASH_BYTES 0x40000u

static char events[128];
static uint32_t event_count;
static uint8_t heater_inhibited;
static uint8_t monitor_allow = 1u;
static uint8_t monitor_end_success = 1u;
static HAL_StatusTypeDef erase_status = HAL_OK;
static HAL_StatusTypeDef program_status = HAL_OK;
static uint32_t watchdog_timeout_ms = 200u;
static uint32_t watchdog_feed_count;

#define CHECK(condition)                                                         \
    do                                                                           \
    {                                                                            \
        if (!(condition))                                                        \
        {                                                                        \
            fprintf(stderr, "FAIL:%d: %s\n", __LINE__, #condition);             \
            return 1;                                                            \
        }                                                                        \
    } while (0)

static void event(char value)
{
    if (event_count < sizeof(events))
        events[event_count++] = value;
}

static void reset_events(void)
{
    memset(events, 0, sizeof(events));
    event_count = 0u;
    heater_inhibited = 0u;
    watchdog_feed_count = 0u;
}

uint8_t IMUHeaterBeginLongOperation(IMUHeater_LongOperation_e operation)
{
    if (operation != IMU_HEATER_LONG_OPERATION_FLASH)
        return 0u;
    event('H');
    heater_inhibited = 1u;
    return 1u;
}

void IMUHeaterEndLongOperation(IMUHeater_LongOperation_e operation, uint8_t success)
{
    if (operation != IMU_HEATER_LONG_OPERATION_FLASH)
        return;
    event(success ? 'h' : 'x');
    if (success)
        heater_inhibited = 0u;
}

static uint8_t ParamOperationGuardBegin(void)
{
    if (!IMUHeaterBeginLongOperation(IMU_HEATER_LONG_OPERATION_FLASH))
        return 0u;
    if (!TaskMonitorBeginLongOperation(TASK_MONITOR_INS, TASK_MONITOR_LONG_OPERATION_FLASH))
    {
        IMUHeaterEndLongOperation(IMU_HEATER_LONG_OPERATION_FLASH, 0u);
        return 0u;
    }
    return 1u;
}

static uint8_t ParamOperationGuardEnd(uint8_t success)
{
    uint8_t within_window = TaskMonitorEndLongOperation(TASK_MONITOR_INS,
                                                        TASK_MONITOR_LONG_OPERATION_FLASH);
    uint8_t operation_success = (uint8_t)(success && within_window);
    IMUHeaterEndLongOperation(IMU_HEATER_LONG_OPERATION_FLASH, operation_success);
    return operation_success;
}

uint8_t TaskMonitorBeginLongOperation(TaskMonitor_Id_e id, TaskMonitor_LongOperation_e operation)
{
    if (id != TASK_MONITOR_INS || operation != TASK_MONITOR_LONG_OPERATION_FLASH)
        return 0u;
    event('M');
    return monitor_allow;
}

uint8_t TaskMonitorEndLongOperation(TaskMonitor_Id_e id, TaskMonitor_LongOperation_e operation)
{
    if (id != TASK_MONITOR_INS || operation != TASK_MONITOR_LONG_OPERATION_FLASH)
        return 0u;
    event('m');
    return monitor_end_success;
}

uint32_t BSP_WatchdogGetTimeout(void)
{
    event('g');
    return watchdog_timeout_ms;
}

void BSP_WatchdogSetTimeout(uint32_t timeout_ms)
{
    event('s');
    watchdog_timeout_ms = timeout_ms;
}

void BSP_WatchdogFeed(void)
{
    event('f');
    watchdog_feed_count++;
}

void MC02ParamOperationLog(void) {}

HAL_StatusTypeDef HAL_FLASH_Unlock(void) { return HAL_OK; }
void HAL_FLASH_Lock(void) { event('L'); }

HAL_StatusTypeDef HAL_FLASHEx_Erase(FLASH_EraseInitTypeDef *erase, uint32_t *sector_error)
{
    event('E');
    if (!heater_inhibited || event_count == 0u || events[0] != 'H' ||
        event_count < 2u || events[1] != 'M')
        return HAL_ERROR;
    *sector_error = 0u;
    if (erase_status != HAL_OK)
        return erase_status;
    uint32_t address = FLASH_BASE + (erase->Sector - FLASH_SECTOR_6) * 0x20000u;
    uint32_t bytes = erase->NbSectors * 0x20000u;
    memset((void *)(uintptr_t)address, 0xff, bytes);
    return HAL_OK;
}

HAL_StatusTypeDef HAL_FLASH_Program(uint32_t type, uint32_t address, uint32_t data_address)
{
    (void)type;
    event('P');
    if (!heater_inhibited || event_count < 2u || events[0] != 'H' || events[1] != 'M')
        return HAL_ERROR;
    if (program_status != HAL_OK)
        return program_status;
    memcpy((void *)(uintptr_t)address, (const void *)(uintptr_t)data_address, 32u);
    return HAL_OK;
}

void SCB_InvalidateDCache_by_Addr(uint32_t *address, int32_t size)
{
    (void)address;
    (void)size;
}

int main(void)
{
    void *flash = mmap((void *)(uintptr_t)FLASH_BASE, FLASH_BYTES,
                       PROT_READ | PROT_WRITE, MAP_PRIVATE | MAP_ANONYMOUS | MAP_FIXED_NOREPLACE,
                       -1, 0);
    if (flash == MAP_FAILED)
    {
        perror("mmap flash window");
        return EXIT_FAILURE;
    }
    memset(flash, 0xff, FLASH_BYTES);

    ParamSetLongOperationGuard(NULL);
    reset_events();
    CHECK(ParamCommit() == 0u);
    CHECK(ParamReset() == 0u);
    CHECK(event_count == 0u);

    const ParamLongOperationGuard_t guard = {
        .begin = ParamOperationGuardBegin,
        .end = ParamOperationGuardEnd,
    };
    ParamSetLongOperationGuard(&guard);

    reset_events();
    monitor_allow = 0u;
    CHECK(ParamCommit() == 0u);
    CHECK(event_count == 3u && events[0] == 'H' && events[1] == 'M' && events[2] == 'x');
    CHECK(watchdog_feed_count == 0u);

    reset_events();
    monitor_allow = 1u;
    monitor_end_success = 1u;
    erase_status = HAL_OK;
    program_status = HAL_OK;
    CHECK(ParamCommit() == 1u);
    CHECK(events[0] == 'H' && events[1] == 'M');
    CHECK(heater_inhibited == 0u);
    CHECK(watchdog_timeout_ms == 200u);
    CHECK(watchdog_feed_count == 1u);
    CHECK(param_commit_count == 1u);

    reset_events();
    monitor_end_success = 0u;
    erase_status = HAL_OK;
    CHECK(ParamCommit() == 0u);
    CHECK(heater_inhibited == 1u);
    CHECK(watchdog_timeout_ms == 200u);

    reset_events();
    monitor_allow = 0u;
    param_loaded = 1u;
    param_seq = 7u;
    CHECK(ParamReset() == 0u);
    CHECK(event_count == 3u && events[0] == 'H' && events[1] == 'M' && events[2] == 'x');
    CHECK(param_loaded == 1u && param_seq == 7u);

    reset_events();
    monitor_allow = 1u;
    monitor_end_success = 1u;
    erase_status = HAL_OK;
    CHECK(ParamReset() == 1u);
    CHECK(heater_inhibited == 0u);
    CHECK(param_loaded == 0u && param_seq == 0u);
    for (uint32_t i = 0u; i < FLASH_BYTES; ++i)
        CHECK(((const uint8_t *)flash)[i] == 0xffu);

    /* If erase succeeds but the safety window cannot close cleanly, report
       failure while retaining the live RAM cache for the caller. */
    const uint32_t cached_parameter = 0x12345678u;
    uint32_t cached_readback = 0u;
    CHECK(ParamSetU32(PARAM_KEY_IMU_G_NORM, cached_parameter) == 1u);
    param_loaded = 1u;
    param_seq = 23u;
    param_active_region = 1u;
    monitor_end_success = 0u;
    erase_status = HAL_OK;
    reset_events();
    CHECK(ParamReset() == 0u);
    CHECK(heater_inhibited == 1u);
    CHECK(param_loaded == 1u && param_seq == 23u && param_active_region == 1u);
    CHECK(ParamGetU32(PARAM_KEY_IMU_G_NORM, &cached_readback) == 1u);
    CHECK(cached_readback == cached_parameter);

    reset_events();
    erase_status = HAL_ERROR;
    monitor_end_success = 1u;
    CHECK(ParamCommit() == 0u);
    CHECK(heater_inhibited == 1u);
    CHECK(watchdog_timeout_ms == 200u);
    CHECK(watchdog_feed_count == 1u);

    munmap(flash, FLASH_BYTES);
    puts("PASS: production ParamCommit begins with heater lock and bounded INS monitoring");
    return EXIT_SUCCESS;
}
