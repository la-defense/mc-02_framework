/** @file param_operation_test.c
 * @brief 用模拟 Flash 验证生产参数维护流程、旧记录读取及 CRC 失败不擦写。
 */
#define _GNU_SOURCE
#include "bsp_param.h"
#include "bsp_flash.h"
#include "bsp_flash_internal.h"
#include "crc_injection.h"
#include "main.h"
#include "bsp_watchdog.h"
#include "task_monitor.h"
#include "imu_heater.h"

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>

#define FLASH_BASE 0x08000000u
#define FLASH_BYTES 0x100000u
#define PARAM_FLASH_BASE 0x080c0000u
#define PARAM_FLASH_BYTES 0x40000u

static char events[128];
static uint32_t event_count;
static uint8_t heater_inhibited;
static uint8_t monitor_allow = 1u;
static uint8_t monitor_end_success = 1u;
static HAL_StatusTypeDef erase_status = HAL_OK;
static HAL_StatusTypeDef program_status = HAL_OK;
static uint32_t watchdog_timeout_ms = 200u;
static uint32_t watchdog_feed_count;
static uint32_t tick_ms;
static uint32_t erase_tick_advance;
static uint32_t program_tick_advance;
static uint32_t interrupt_context;
static uint32_t erase_fail_sector = UINT32_MAX;
static HAL_StatusTypeDef unlock_status = HAL_OK;
static uint8_t program_word[32] __attribute__((aligned(32)));

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
    tick_ms = 0u;
    erase_tick_advance = 0u;
    program_tick_advance = 0u;
    interrupt_context = 0u;
    erase_fail_sector = UINT32_MAX;
    unlock_status = HAL_OK;
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

HAL_StatusTypeDef HAL_FLASH_Unlock(void) { event('U'); return unlock_status; }
void HAL_FLASH_Lock(void) { event('L'); }

HAL_StatusTypeDef HAL_FLASHEx_Erase(FLASH_EraseInitTypeDef *erase, uint32_t *sector_error)
{
    event('E');
    if (!heater_inhibited || event_count == 0u || events[0] != 'H' ||
        event_count < 2u || events[1] != 'M')
        return HAL_ERROR;
    if (erase->Sector == erase_fail_sector)
    {
        *sector_error = erase->Sector;
        return HAL_ERROR;
    }
    *sector_error = 0u;
    if (erase_status != HAL_OK)
        return erase_status;
    uint32_t address = PARAM_FLASH_BASE + (erase->Sector - FLASH_SECTOR_6) * 0x20000u;
    uint32_t bytes = erase->NbSectors * 0x20000u;
    memset((void *)(uintptr_t)address, 0xff, bytes);
    tick_ms += erase_tick_advance;
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
    const uint8_t *source = (const uint8_t *)(uintptr_t)data_address;
    uint8_t *destination = (uint8_t *)(uintptr_t)address;
    for (size_t i = 0u; i < 32u; ++i)
    {
        if ((destination[i] & source[i]) != source[i])
            return HAL_ERROR;
    }
    memcpy(destination, source, 32u);
    tick_ms += program_tick_advance;
    return HAL_OK;
}

void SCB_InvalidateDCache_by_Addr(uint32_t *address, int32_t size)
{
    (void)address;
    (void)size;
}

uint32_t HAL_GetTick(void) { return tick_ms; }
uint32_t __get_IPSR(void) { return interrupt_context; }

int main(int argc, char **argv)
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

    BSPFlashPartitionInfo_t info = {.start_address = 1u, .size_bytes = 2u};
    CHECK(BSPFlashGetPartitionInfo(BSP_FLASH_PARTITION_APPLICATION, &info) == BSP_FLASH_OK);
    CHECK(info.start_address == FLASH_BASE && info.size_bytes == 0xC0000u && info.writable == 0u);
    CHECK(BSPFlashGetPartitionInfo(BSP_FLASH_PARTITION_PARAMETER_A, &info) == BSP_FLASH_OK);
    CHECK(info.start_address == PARAM_FLASH_BASE && info.size_bytes == 0x20000u && info.writable == 1u);
    CHECK(info.erase_granularity_bytes == 0x20000u && info.program_granularity_bytes == 32u);
    CHECK(BSPFlashGetPartitionInfo(BSP_FLASH_PARTITION_PARAMETER_B, &info) == BSP_FLASH_OK);
    CHECK(info.start_address == PARAM_FLASH_BASE + 0x20000u && info.size_bytes == 0x20000u);
    CHECK(BSPFlashGetPartitionInfo(BSP_FLASH_PARTITION_COUNT, &info) == BSP_FLASH_BAD_PARTITION);
    CHECK(info.start_address == PARAM_FLASH_BASE + 0x20000u);
    CHECK(BSPFlashGetPartitionInfo(BSP_FLASH_PARTITION_APPLICATION, NULL) == BSP_FLASH_BAD_ARGUMENT);

    ((uint8_t *)flash)[0] = 0x5au;
    uint8_t read_byte = 0u;
    CHECK(BSPFlashRead(BSP_FLASH_PARTITION_APPLICATION, 0u, &read_byte, 1u) == BSP_FLASH_OK);
    CHECK(read_byte == 0x5au);
    read_byte = 0xA5u;
    CHECK(BSPFlashRead(BSP_FLASH_PARTITION_APPLICATION, 0xBFFFFu, &read_byte, 2u) == BSP_FLASH_OUT_OF_RANGE);
    CHECK(read_byte == 0xA5u);
    CHECK(BSPFlashRead(BSP_FLASH_PARTITION_PARAMETER_A, 0x20000u, NULL, 0u) == BSP_FLASH_OK);
    CHECK(BSPFlashRead(BSP_FLASH_PARTITION_PARAMETER_A, 0u, NULL, 1u) == BSP_FLASH_BAD_ARGUMENT);
    CHECK(BSPFlashRead(BSP_FLASH_PARTITION_PARAMETER_A, SIZE_MAX, &read_byte, 1u) == BSP_FLASH_OUT_OF_RANGE);
    CHECK(BSPFlashErase(BSP_FLASH_PARTITION_APPLICATION, 0u, 0x20000u, NULL) == BSP_FLASH_READ_ONLY);
    CHECK(BSPFlashProgram(BSP_FLASH_PARTITION_APPLICATION, 0u, program_word, sizeof(program_word), NULL) == BSP_FLASH_READ_ONLY);
    CHECK(BSPFlashErase(BSP_FLASH_PARTITION_PARAMETER_B, 0x1f000u, 0x2000u, NULL) == BSP_FLASH_OUT_OF_RANGE);
    CHECK(BSPFlashProgram(BSP_FLASH_PARTITION_PARAMETER_A, 0x1ffe0u, program_word,
                          sizeof(program_word) * 2u, NULL) == BSP_FLASH_OUT_OF_RANGE);
    CHECK(BSPFlashErase(BSP_FLASH_PARTITION_PARAMETER_A, 0u, 0x20000u, NULL) == BSP_FLASH_UNAUTHORIZED);
    CHECK(BSPFlashProgram(BSP_FLASH_PARTITION_PARAMETER_A, 0u, program_word, sizeof(program_word), NULL) == BSP_FLASH_UNAUTHORIZED);
    CHECK(BSPFlashProgram(BSP_FLASH_PARTITION_PARAMETER_A, 0u,
                          (const void *)(uintptr_t)PARAM_FLASH_BASE,
                          sizeof(program_word), NULL) == BSP_FLASH_BAD_ARGUMENT);
    CHECK(BSPFlashErase(BSP_FLASH_PARTITION_PARAMETER_A, 0u, 1u, NULL) == BSP_FLASH_ALIGNMENT_ERROR);
    CHECK(BSPFlashProgram(BSP_FLASH_PARTITION_PARAMETER_A, 1u, program_word, sizeof(program_word), NULL) == BSP_FLASH_ALIGNMENT_ERROR);
    CHECK(BSPFlashProgram(BSP_FLASH_PARTITION_PARAMETER_A, 0u, &program_word[1], 31u, NULL) == BSP_FLASH_ALIGNMENT_ERROR);
    CHECK(event_count == 0u);

    // 固定旧格式记录：key=0x0101，4字节值0x12345678，CRC由独立 zlib 核对。
    if (argc > 1)
    {
        const uint8_t legacy[] = {0x4d, 0x30, 0x43, 0x32, 0x01, 0x00, 0x08, 0x00, 0x07, 0x00, 0x00, 0x00, 0x9b, 0xce, 0x47, 0x1a, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x01, 0x04, 0x00, 0x78, 0x56, 0x34, 0x12};
        memcpy((void *)(uintptr_t)PARAM_FLASH_BASE, legacy, sizeof(legacy));
        uint8_t fail_read = strcmp(argv[1], "read-failure") == 0;
        if (fail_read) crc_fail_after = 0;
        CHECK(ParamInit() == (fail_read ? 0u : 1u));
        uint32_t value = 0xabcdefu;
        CHECK(ParamGetU32(PARAM_KEY_IMU_G_NORM, &value) == (fail_read ? 0u : 1u));
        CHECK(value == (fail_read ? 0xabcdefu : 0x12345678u));
        CHECK(event_count == 0u);
        munmap(flash, FLASH_BYTES);
        return EXIT_SUCCESS;
    }
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

    // 单独检查底层把HAL忙碌和超时映射成不同结果；此令牌只在测试进程中直接建立。
    BSPFlashAuthorization_t direct_authorization = {0};
    CHECK(BSPFlashAuthorizationOpen(&direct_authorization, HAL_GetTick()) == BSP_FLASH_OK);
    heater_inhibited = 1u;
    events[0] = 'H';
    events[1] = 'M';
    event_count = 2u;
    unlock_status = HAL_BUSY;
    CHECK(BSPFlashErase(BSP_FLASH_PARTITION_PARAMETER_A, 0u, 0x20000u,
                        &direct_authorization) == BSP_FLASH_HAL_BUSY);
    unlock_status = HAL_TIMEOUT;
    CHECK(BSPFlashErase(BSP_FLASH_PARTITION_PARAMETER_A, 0u, 0x20000u,
                        &direct_authorization) == BSP_FLASH_HAL_TIMEOUT);
    unlock_status = HAL_OK;
    erase_status = HAL_BUSY;
    CHECK(BSPFlashErase(BSP_FLASH_PARTITION_PARAMETER_A, 0u, 0x20000u,
                        &direct_authorization) == BSP_FLASH_HAL_BUSY);
    erase_status = HAL_TIMEOUT;
    CHECK(BSPFlashErase(BSP_FLASH_PARTITION_PARAMETER_A, 0u, 0x20000u,
                        &direct_authorization) == BSP_FLASH_HAL_TIMEOUT);
    unlock_status = HAL_BUSY;
    CHECK(BSPFlashProgram(BSP_FLASH_PARTITION_PARAMETER_A, 0u, program_word,
                          sizeof(program_word), &direct_authorization) == BSP_FLASH_HAL_BUSY);
    unlock_status = HAL_TIMEOUT;
    CHECK(BSPFlashProgram(BSP_FLASH_PARTITION_PARAMETER_A, 0u, program_word,
                          sizeof(program_word), &direct_authorization) == BSP_FLASH_HAL_TIMEOUT);
    unlock_status = HAL_OK;
    program_status = HAL_BUSY;
    CHECK(BSPFlashProgram(BSP_FLASH_PARTITION_PARAMETER_A, 0u, program_word,
                          sizeof(program_word), &direct_authorization) == BSP_FLASH_HAL_BUSY);
    program_status = HAL_TIMEOUT;
    CHECK(BSPFlashProgram(BSP_FLASH_PARTITION_PARAMETER_A, 0u, program_word,
                          sizeof(program_word), &direct_authorization) == BSP_FLASH_HAL_TIMEOUT);
    CHECK(BSPFlashAuthorizationClose(&direct_authorization) == BSP_FLASH_OK);

    reset_events();
    monitor_allow = 0u;
    CHECK(ParamCommit() == 0u);
    CHECK(event_count == 3u && events[0] == 'H' && events[1] == 'M' && events[2] == 'x');
    CHECK(watchdog_feed_count == 0u);

    // ISR不能进入会调用任务监控与加热器接口的参数维护流程。
    reset_events();
    monitor_allow = 1u;
    interrupt_context = 1u;
    CHECK(ParamCommit() == 0u);
    CHECK(event_count == 0u);
    interrupt_context = 0u;

    reset_events();
    monitor_allow = 1u;
    monitor_end_success = 1u;
    erase_status = HAL_OK;
    program_status = HAL_OK;
    // 第一次 CRC 失败时，必须在任何擦除或编程事件前退出。
    crc_fail_after = 0;
    CHECK(ParamCommit() == 0u);
    CHECK(memchr(events, 'E', event_count) == NULL && memchr(events, 'P', event_count) == NULL);
    reset_events();
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
    for (uint32_t i = 0u; i < PARAM_FLASH_BYTES; ++i)
        CHECK(((const uint8_t *)(uintptr_t)PARAM_FLASH_BASE)[i] == 0xffu);

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

    // HAL 解锁失败须立即锁回，不进入擦除或编程。
    reset_events();
    erase_status = HAL_OK;
    unlock_status = HAL_ERROR;
    CHECK(ParamCommit() == 0u);
    CHECK(memchr(events, 'E', event_count) == NULL && memchr(events, 'P', event_count) == NULL);
    CHECK(memchr(events, 'L', event_count) != NULL);
    CHECK(watchdog_timeout_ms == 200u);

    // 任一Flash word写入失败都必须锁回并保留原活动区域。
    reset_events();
    unlock_status = HAL_OK;
    program_status = HAL_ERROR;
    const uint32_t active_region_before_program_failure = param_active_region;
    const uint32_t sequence_before_program_failure = param_seq;
    CHECK(ParamCommit() == 0u);
    CHECK(memchr(events, 'E', event_count) != NULL && memchr(events, 'P', event_count) != NULL);
    CHECK(memchr(events, 'L', event_count) != NULL);
    CHECK(param_active_region == active_region_before_program_failure);
    CHECK(param_seq == sequence_before_program_failure);

    // 维护授权沿用外层开始时刻；达到8秒后不得继续写入。
    reset_events();
    unlock_status = HAL_OK;
    erase_tick_advance = BSP_FLASH_MAINTENANCE_TIMEOUT_MS;
    CHECK(ParamCommit() == 0u);
    CHECK(memchr(events, 'E', event_count) != NULL && memchr(events, 'P', event_count) == NULL);
    CHECK(heater_inhibited == 1u && watchdog_timeout_ms == 200u);

    // 若 Reset 的第二个区域失败，第一个区域可能已擦除，但 RAM 参数不变。
    reset_events();
    erase_tick_advance = 0u;
    erase_fail_sector = FLASH_SECTOR_7;
    const uint32_t retained_value = 0x89ABCDEFu;
    uint32_t retained_readback = 0u;
    CHECK(ParamSetU32(PARAM_KEY_IMU_G_NORM, retained_value) == 1u);
    param_loaded = 1u;
    param_seq = 31u;
    param_active_region = 0u;
    CHECK(ParamReset() == 0u);
    CHECK(param_loaded == 1u && param_seq == 31u && param_active_region == 0u);
    CHECK(ParamGetU32(PARAM_KEY_IMU_G_NORM, &retained_readback) == 1u);
    CHECK(retained_readback == retained_value);

    // 毫秒计数回绕时仍按无符号差值计算8秒维护期限。
    reset_events();
    erase_fail_sector = UINT32_MAX;
    program_status = HAL_OK;
    tick_ms = UINT32_MAX - 3u;
    CHECK(ParamCommit() == 1u);

    munmap(flash, FLASH_BYTES);
    puts("PASS: production ParamCommit begins with heater lock and bounded INS monitoring");
    return EXIT_SUCCESS;
}
