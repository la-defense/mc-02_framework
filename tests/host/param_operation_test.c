/** @file param_operation_test.c
 * @brief 用模拟 Flash 验证生产参数组事务、v1兼容、A/B回退和维护失败处理。
 */
#define _GNU_SOURCE
#include "bsp_param.h"
#include "bsp_flash.h"
#include "bsp_flash_internal.h"
#include "crc_injection.h"
#include "crc.h"
#include "main.h"
#include "bsp_watchdog.h"
#include "task_monitor.h"
#include "imu_heater.h"

#include <stdint.h>
#include <math.h>
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

#define PARAM_TEST_GENERIC_KEY 0x7f00u

#define CHECK(condition)                                                         \
    do                                                                           \
    {                                                                            \
        if (!(condition))                                                        \
        {                                                                        \
            fprintf(stderr, "FAIL:%d: %s\n", __LINE__, #condition);             \
            return 1;                                                            \
        }                                                                        \
    } while (0)

static uint8_t ParamOperationGuardBegin(void);
static uint8_t ParamOperationGuardEnd(uint8_t success);

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

static void store_le16(uint8_t *target, uint16_t value)
{
    target[0] = (uint8_t)value;
    target[1] = (uint8_t)(value >> 8);
}

static void store_le32(uint8_t *target, uint32_t value)
{
    target[0] = (uint8_t)value;
    target[1] = (uint8_t)(value >> 8);
    target[2] = (uint8_t)(value >> 16);
    target[3] = (uint8_t)(value >> 24);
}

static uint16_t append_parameter(uint8_t *payload, uint16_t used, uint16_t key,
                                 const void *value, uint16_t size)
{
    store_le16(&payload[used], key);
    store_le16(&payload[used + 2u], size);
    memcpy(&payload[used + 4u], value, size);
    const uint16_t padded = (uint16_t)((size + 3u) & (uint16_t)~3u);
    memset(&payload[used + 4u + size], 0, padded - size);
    return (uint16_t)(used + 4u + padded);
}

static uint16_t make_calibration_payload(uint8_t *payload, float g_norm,
                                         uint8_t include_schema, uint32_t schema_version)
{
    const float offset[3] = {0.01f, -0.02f, 0.0f};
    uint16_t used = append_parameter(payload, 0u, PARAM_KEY_IMU_GYRO_OFFSET,
                                     offset, sizeof(offset));
    used = append_parameter(payload, used, PARAM_KEY_IMU_G_NORM, &g_norm,
                            sizeof(g_norm));
    if (include_schema)
        used = append_parameter(payload, used, PARAM_KEY_IMU_CALIB_SCHEMA_VERSION,
                                &schema_version, sizeof(schema_version));
    return used;
}

static int write_test_record(uint8_t region, uint32_t sequence,
                             const uint8_t *payload, uint16_t payload_size)
{
    uint8_t *sector = (uint8_t *)(uintptr_t)(PARAM_FLASH_BASE + (uint32_t)region * 0x20000u);
    uint8_t header[32] = {0};
    uint32_t crc = 0u;
    if (CRCCompute(CRC32_IEEE, payload, payload_size, &crc) != CRC_OK)
        return 0;
    store_le32(&header[0], PARAM_MAGIC);
    store_le16(&header[4], PARAM_VERSION);
    store_le16(&header[6], payload_size);
    store_le32(&header[8], sequence);
    store_le32(&header[12], crc);
    memset(sector, 0xff, 0x20000u);
    memcpy(sector, header, sizeof(header));
    memcpy(sector + sizeof(header), payload, payload_size);
    return 1;
}

static int test_parameter_group_api(void)
{
    CHECK(ParamInit() == 0u);
    CHECK(ParamGetDescriptorCount() == 4u);

    ParamDescriptor_t descriptor;
    memset(&descriptor, 0, sizeof(descriptor));
    CHECK(ParamGetDescriptor(0u, &descriptor) == PARAM_RESULT_OK);
    CHECK(descriptor.key == PARAM_KEY_IMU_GYRO_OFFSET);
    CHECK(descriptor.value_type == PARAM_VALUE_FLOAT32_ARRAY);
    CHECK(descriptor.element_count == 3u && descriptor.byte_length == 12u);
    CHECK(strcmp(descriptor.unit, "rad/s") == 0);
    CHECK(descriptor.minimum_float == -0.1f && descriptor.maximum_float == 0.1f);
    CHECK(descriptor.minimum_float_inclusive == 1u && descriptor.maximum_float_inclusive == 1u);
    CHECK(descriptor.default_float[0] == 0.0f && descriptor.logical_version == 2u);
    CHECK(ParamGetDescriptor(2u, &descriptor) == PARAM_RESULT_OK);
    CHECK(descriptor.value_type == PARAM_VALUE_IMU_CALIB_META && descriptor.optional == 1u);
    CHECK(descriptor.minimum_float == PARAM_IMU_CALIB_META_TEMP_MIN_C &&
          descriptor.maximum_float == PARAM_IMU_CALIB_META_TEMP_MAX_C);
    CHECK(descriptor.minimum_float_inclusive == 0u && descriptor.maximum_float_inclusive == 0u);
    CHECK(descriptor.minimum_u32 == PARAM_IMU_CALIB_OK &&
          descriptor.maximum_u32 == PARAM_IMU_CALIB_OK);

    ParamDescriptor_t unchanged;
    memset(&unchanged, 0x5a, sizeof(unchanged));
    ParamDescriptor_t before = unchanged;
    CHECK(ParamGetDescriptor(4u, &unchanged) == PARAM_RESULT_INDEX_OUT_OF_RANGE);
    CHECK(memcmp(&unchanged, &before, sizeof(unchanged)) == 0);

    ParamImuCalibrationGroup_t group = {
        .gyro_offset_rad_s = {0.02f, -0.03f, 0.0f},
        .g_norm_m_s2 = 9.805f,
        .meta = {.result = PARAM_IMU_CALIB_OK, .temperature = 24.0f, .time_s = 77u},
        .has_meta = 1u,
    };
    ParamImuCalibrationTransaction_t transaction;
    memset(&transaction, 0xa5, sizeof(transaction));
    ParamImuCalibrationTransaction_t transaction_before = transaction;
    ParamImuCalibrationGroup_t invalid = group;
    invalid.gyro_offset_rad_s[1] = NAN;
    CHECK(ParamImuCalibrationStage(&invalid, &transaction) == PARAM_RESULT_INVALID_VALUE);
    CHECK(memcmp(&transaction, &transaction_before, sizeof(transaction)) == 0);

    ParamImuCalibrationGroup_t readback;
    uint32_t source_version = 99u;
    memset(&readback, 0x3c, sizeof(readback));
    CHECK(ParamImuCalibrationRead(&readback, &source_version) == PARAM_RESULT_NOT_FOUND);
    CHECK(source_version == 99u);

    CHECK(ParamImuCalibrationStage(&group, &transaction) == PARAM_RESULT_OK);
    CHECK(ParamImuCalibrationAccept(&transaction) == PARAM_RESULT_OK);
    CHECK(ParamSetFloat(PARAM_KEY_IMU_G_NORM, 10.0f) == 0u);
    CHECK(ParamImuCalibrationRead(&readback, &source_version) == PARAM_RESULT_OK);
    CHECK(source_version == 2u && readback.has_meta == 1u);
    CHECK(readback.gyro_offset_rad_s[0] == group.gyro_offset_rad_s[0]);
    CHECK(readback.g_norm_m_s2 == group.g_norm_m_s2);
    CHECK(readback.meta.time_s == group.meta.time_s);

    group.g_norm_m_s2 = 10.5001f;
    CHECK(ParamImuCalibrationStage(&group, &transaction) == PARAM_RESULT_INVALID_VALUE);
    CHECK(ParamImuCalibrationRead(&readback, &source_version) == PARAM_RESULT_OK);
    CHECK(readback.g_norm_m_s2 == 9.805f);

    /* 新版本标记随下一次提交写入，接受参数组本身不触碰 Flash。 */
    CHECK(event_count == 0u);
    ParamLongOperationGuard_t guard = {
        .begin = ParamOperationGuardBegin,
        .end = ParamOperationGuardEnd,
    };
    ParamSetLongOperationGuard(&guard);
    CHECK(ParamCommit() == 1u);
    CHECK(param_loaded == 1u && param_seq == 1u);
    const uint8_t *saved = (const uint8_t *)(uintptr_t)(PARAM_FLASH_BASE + 0x20000u + 32u);
    const uint8_t *saved_header = (const uint8_t *)(uintptr_t)(PARAM_FLASH_BASE + 0x20000u);
    const uint16_t saved_payload_size = (uint16_t)(saved_header[6] |
                                                   ((uint16_t)saved_header[7] << 8));
    uint32_t saved_schema = 0u;
    uint8_t saw_schema = 0u;
    for (uint16_t off = 0u; off + 4u <= saved_payload_size; )
    {
        uint16_t key = (uint16_t)(saved[off] | ((uint16_t)saved[off + 1u] << 8));
        uint16_t size = (uint16_t)(saved[off + 2u] | ((uint16_t)saved[off + 3u] << 8));
        if (key == PARAM_KEY_IMU_CALIB_SCHEMA_VERSION && size == sizeof(saved_schema))
        {
            memcpy(&saved_schema, &saved[off + 4u], sizeof(saved_schema));
            saw_schema = 1u;
            break;
        }
        off = (uint16_t)(off + 4u + ((size + 3u) & (uint16_t)~3u));
    }
    CHECK(saw_schema == 1u && saved_schema == 2u);
    return 0;
}

static int test_parameter_group_requires_initialization(void)
{
    ParamImuCalibrationGroup_t output;
    memset(&output, 0x5au, sizeof(output));
    const ParamImuCalibrationGroup_t before = output;
    uint32_t source_version = 0x12345678u;
    CHECK(ParamImuCalibrationRead(&output, &source_version) == PARAM_RESULT_NOT_INITIALIZED);
    CHECK(memcmp(&output, &before, sizeof(output)) == 0);
    CHECK(source_version == 0x12345678u);
    CHECK(ParamImuCalibrationRead(NULL, &source_version) == PARAM_RESULT_INVALID_ARGUMENT);

    ParamImuCalibrationGroup_t valid = {
        .gyro_offset_rad_s = {0.0f, 0.01f, -0.01f},
        .g_norm_m_s2 = 9.805f,
    };
    ParamImuCalibrationTransaction_t transaction;
    CHECK(ParamImuCalibrationStage(&valid, &transaction) == PARAM_RESULT_OK);
    const ParamImuCalibrationTransaction_t transaction_before = transaction;
    CHECK(ParamImuCalibrationAccept(&transaction) == PARAM_RESULT_NOT_INITIALIZED);
    CHECK(memcmp(&transaction, &transaction_before, sizeof(transaction)) == 0);
    return 0;
}

static int test_parameter_group_legacy_load(void)
{
    uint8_t payload[PARAM_MAX_PAYLOAD] = {0};
    uint16_t payload_size = make_calibration_payload(payload, 9.7f, 0u, 0u);
    const ParamImuCalibMeta_t old_meta = {
        .result = PARAM_IMU_CALIB_FAILED,
        .temperature = NAN,
        .time_s = 12u,
    };
    payload_size = append_parameter(payload, payload_size, PARAM_KEY_IMU_CALIB_META,
                                    &old_meta, sizeof(old_meta));
    CHECK(write_test_record(0u, 11u, payload, payload_size));
    CHECK(ParamInit() == 1u);
    ParamImuCalibrationGroup_t readback;
    uint32_t source_version = 0u;
    CHECK(ParamImuCalibrationRead(&readback, &source_version) == PARAM_RESULT_OK);
    CHECK(source_version == 1u && readback.g_norm_m_s2 == 9.7f);
    CHECK(readback.has_meta == 0u);
    CHECK(param_active_region == 0u && param_seq == 11u);
    CHECK(event_count == 0u);
    return 0;
}

static int test_parameter_group_capacity_failure(void)
{
    CHECK(ParamInit() == 0u);
    ParamImuCalibrationGroup_t original = {
        .gyro_offset_rad_s = {0.01f, 0.02f, 0.0f},
        .g_norm_m_s2 = 9.8f,
        .has_meta = 0u,
    };
    ParamImuCalibrationTransaction_t transaction;
    CHECK(ParamImuCalibrationStage(&original, &transaction) == PARAM_RESULT_OK);
    CHECK(ParamImuCalibrationAccept(&transaction) == PARAM_RESULT_OK);

    uint8_t filler[476];
    memset(filler, 0x3c, sizeof(filler));
    CHECK(ParamSet(0x7f01u, filler, sizeof(filler)) == 1u);

    ParamImuCalibrationGroup_t with_meta = original;
    with_meta.meta.result = PARAM_IMU_CALIB_OK;
    with_meta.meta.temperature = 21.5f;
    with_meta.meta.time_s = 44u;
    with_meta.has_meta = 1u;
    CHECK(ParamImuCalibrationStage(&with_meta, &transaction) == PARAM_RESULT_OK);
    const ParamImuCalibrationTransaction_t transaction_before = transaction;
    CHECK(ParamImuCalibrationAccept(&transaction) == PARAM_RESULT_NO_SPACE);
    CHECK(memcmp(&transaction, &transaction_before, sizeof(transaction)) == 0);

    ParamImuCalibrationGroup_t readback;
    uint32_t source_version = 0u;
    CHECK(ParamImuCalibrationRead(&readback, &source_version) == PARAM_RESULT_OK);
    CHECK(source_version == 2u && readback.has_meta == 0u);
    CHECK(readback.g_norm_m_s2 == original.g_norm_m_s2);
    uint8_t filler_copy[sizeof(filler)];
    CHECK(ParamGet(0x7f01u, filler_copy, sizeof(filler_copy)) == 1u);
    CHECK(memcmp(filler_copy, filler, sizeof(filler)) == 0);

    /* 原始参数扩容失败时同样保留旧条目，而非先删除再报错。 */
    uint8_t larger_filler[480];
    memset(larger_filler, 0xa7, sizeof(larger_filler));
    CHECK(ParamSet(0x7f01u, larger_filler, sizeof(larger_filler)) == 0u);
    CHECK(ParamGet(0x7f01u, filler_copy, sizeof(filler_copy)) == 1u);
    CHECK(memcmp(filler_copy, filler, sizeof(filler)) == 0);
    return 0;
}

static int test_parameter_group_flash_failure_keeps_old_record(void)
{
    uint8_t old_payload[PARAM_MAX_PAYLOAD] = {0};
    const uint16_t old_size = make_calibration_payload(old_payload, 9.8f, 0u, 0u);
    CHECK(write_test_record(0u, 8u, old_payload, old_size));
    uint8_t old_record[64];
    memcpy(old_record, (const void *)(uintptr_t)PARAM_FLASH_BASE, sizeof(old_record));
    CHECK(ParamInit() == 1u);

    ParamImuCalibrationGroup_t replacement = {
        .gyro_offset_rad_s = {-0.01f, 0.03f, 0.0f},
        .g_norm_m_s2 = 10.0f,
        .meta = {.result = PARAM_IMU_CALIB_OK, .temperature = 23.0f, .time_s = 90u},
        .has_meta = 1u,
    };
    ParamImuCalibrationTransaction_t transaction;
    CHECK(ParamImuCalibrationStage(&replacement, &transaction) == PARAM_RESULT_OK);
    CHECK(ParamImuCalibrationAccept(&transaction) == PARAM_RESULT_OK);
    const ParamLongOperationGuard_t guard = {
        .begin = ParamOperationGuardBegin,
        .end = ParamOperationGuardEnd,
    };
    ParamSetLongOperationGuard(&guard);
    program_status = HAL_ERROR;
    CHECK(ParamCommit() == 0u);
    CHECK(param_active_region == 0u && param_seq == 8u);
    CHECK(memcmp(old_record, (const void *)(uintptr_t)PARAM_FLASH_BASE,
                 sizeof(old_record)) == 0);

    ParamImuCalibrationGroup_t readback;
    uint32_t source_version = 0u;
    CHECK(ParamImuCalibrationRead(&readback, &source_version) == PARAM_RESULT_OK);
    CHECK(source_version == 2u && readback.g_norm_m_s2 == replacement.g_norm_m_s2);
    return 0;
}

static int test_parameter_group_semantic_fallback(void)
{
    uint8_t older[PARAM_MAX_PAYLOAD] = {0};
    uint8_t newer[PARAM_MAX_PAYLOAD] = {0};
    const uint16_t older_size = make_calibration_payload(older, 9.8f, 0u, 0u);
    const uint16_t newer_size = make_calibration_payload(newer, 11.0f, 1u, 2u);
    CHECK(write_test_record(0u, 20u, older, older_size));
    CHECK(write_test_record(1u, 21u, newer, newer_size));
    CHECK(ParamInit() == 1u);
    ParamImuCalibrationGroup_t readback;
    uint32_t source_version = 0u;
    CHECK(ParamImuCalibrationRead(&readback, &source_version) == PARAM_RESULT_OK);
    CHECK(param_active_region == 0u && param_seq == 20u);
    CHECK(source_version == 1u && readback.g_norm_m_s2 == 9.8f);
    return 0;
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

    if (argc > 1 && strcmp(argv[1], "group") == 0)
    {
        int result = test_parameter_group_api();
        munmap(flash, FLASH_BYTES);
        return result == 0 ? EXIT_SUCCESS : EXIT_FAILURE;
    }
    if (argc > 1 && strcmp(argv[1], "group-uninitialized") == 0)
    {
        int result = test_parameter_group_requires_initialization();
        munmap(flash, FLASH_BYTES);
        return result == 0 ? EXIT_SUCCESS : EXIT_FAILURE;
    }
    if (argc > 1 && strcmp(argv[1], "legacy-bmi") == 0)
    {
        int result = test_parameter_group_legacy_load();
        munmap(flash, FLASH_BYTES);
        return result == 0 ? EXIT_SUCCESS : EXIT_FAILURE;
    }
    if (argc > 1 && strcmp(argv[1], "semantic-fallback") == 0)
    {
        int result = test_parameter_group_semantic_fallback();
        munmap(flash, FLASH_BYTES);
        return result == 0 ? EXIT_SUCCESS : EXIT_FAILURE;
    }
    if (argc > 1 && strcmp(argv[1], "group-capacity") == 0)
    {
        int result = test_parameter_group_capacity_failure();
        munmap(flash, FLASH_BYTES);
        return result == 0 ? EXIT_SUCCESS : EXIT_FAILURE;
    }
    if (argc > 1 && strcmp(argv[1], "group-commit-failure") == 0)
    {
        int result = test_parameter_group_flash_failure_keeps_old_record();
        munmap(flash, FLASH_BYTES);
        return result == 0 ? EXIT_SUCCESS : EXIT_FAILURE;
    }

    // 固定旧格式记录：未知测试键0x7f00，验证保留原始参数读取兼容。
    if (argc > 1)
    {
        const uint8_t legacy[] = {0x4d, 0x30, 0x43, 0x32, 0x01, 0x00, 0x08, 0x00, 0x07, 0x00, 0x00, 0x00, 0x73, 0x65, 0xba, 0x6a, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x7f, 0x04, 0x00, 0x78, 0x56, 0x34, 0x12};
        memcpy((void *)(uintptr_t)PARAM_FLASH_BASE, legacy, sizeof(legacy));
        uint8_t fail_read = strcmp(argv[1], "read-failure") == 0;
        if (fail_read) crc_fail_after = 0;
        CHECK(ParamInit() == (fail_read ? 0u : 1u));
        uint32_t value = 0xabcdefu;
        CHECK(ParamGetU32(PARAM_TEST_GENERIC_KEY, &value) == (fail_read ? 0u : 1u));
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
    CHECK(ParamSetU32(PARAM_TEST_GENERIC_KEY, cached_parameter) == 1u);
    param_loaded = 1u;
    param_seq = 23u;
    param_active_region = 1u;
    monitor_end_success = 0u;
    erase_status = HAL_OK;
    reset_events();
    CHECK(ParamReset() == 0u);
    CHECK(heater_inhibited == 1u);
    CHECK(param_loaded == 1u && param_seq == 23u && param_active_region == 1u);
    CHECK(ParamGetU32(PARAM_TEST_GENERIC_KEY, &cached_readback) == 1u);
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
    CHECK(ParamSetU32(PARAM_TEST_GENERIC_KEY, retained_value) == 1u);
    param_loaded = 1u;
    param_seq = 31u;
    param_active_region = 0u;
    CHECK(ParamReset() == 0u);
    CHECK(param_loaded == 1u && param_seq == 31u && param_active_region == 0u);
    CHECK(ParamGetU32(PARAM_TEST_GENERIC_KEY, &retained_readback) == 1u);
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
