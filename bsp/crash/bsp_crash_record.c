/**
 * @file bsp_crash_record.c
 * @brief 在固定容量内检查崩溃记录头部、版本、长度和CRC。
 * 该模块不读取硬件、不分配内存；输入可来自主机文件或异常保留区的局部副本。
 * @author MC-02 contributors
 * @license BSD-3-Clause; see LICENSE.
 */
#include "bsp_crash_record.h"

#include <stddef.h>

#include "bsp_crash_crc.h"

#define CRASH_V3_MAGIC_BUSY 0x59535542u
#define CRASH_V3_MAGIC_OK 0x4B4F5243u

_Static_assert(sizeof(CrashLog_t) <= CRASH_RECORD_CAPACITY,
               "v4崩溃记录必须装入链接保留区");
_Static_assert(sizeof(CrashLog_t) == 336u,
               "v4记录长度变化时须更新离线布局和固定测试向量");
_Static_assert((sizeof(CrashLog_t) & 3u) == 0u,
               "崩溃记录长度须为4的倍数");
_Static_assert(offsetof(CrashLog_t, crc32) == sizeof(CrashLog_t) - sizeof(uint32_t),
               "CRC必须位于记录末尾");
_Static_assert((offsetof(CrashLog_t, crc32) - sizeof(uint32_t)) % 4u == 0u,
               "CRC覆盖范围须按32位字对齐");

static uint32_t CrashLogReadLE32(const uint8_t *bytes)
{
    return (uint32_t)bytes[0] | ((uint32_t)bytes[1] << 8u) |
           ((uint32_t)bytes[2] << 16u) | ((uint32_t)bytes[3] << 24u);
}

static uint32_t CrashLogCRC32Bytes(const uint8_t *bytes, size_t length)
{
    uint32_t crc = 0xFFFFFFFFu;
    for (size_t index = 0u; index < length; ++index)
    {
        crc ^= bytes[index];
        for (uint32_t bit = 0u; bit < 8u; ++bit)
            crc = (crc >> 1u) ^ ((crc & 1u) ? 0xEDB88320u : 0u);
    }
    return ~crc;
}

CrashRecordStatus_t CrashLogRecordInspect(const uint8_t *bytes, size_t capacity,
                                          CrashRecordInfo_t *info)
{
    CrashRecordInfo_t result = {0};
    uint32_t magic;

    result.status = CRASH_RECORD_BAD_ARGUMENT;
    result.expected_length = (uint32_t)sizeof(CrashLog_t);
    if (info == NULL)
        return result.status;

    if (capacity > CRASH_RECORD_CAPACITY || (bytes == NULL && capacity != 0u))
    {
        *info = result;
        return result.status;
    }
    if (capacity == 0u)
    {
        result.status = CRASH_RECORD_EMPTY;
        *info = result;
        return result.status;
    }
    if (capacity < sizeof(uint32_t))
    {
        result.status = CRASH_RECORD_BAD_LENGTH;
        result.actual_length = (uint32_t)capacity;
        *info = result;
        return result.status;
    }

    magic = CrashLogReadLE32(bytes);
    if (magic == 0u)
    {
        result.status = CRASH_RECORD_EMPTY;
        *info = result;
        return result.status;
    }
    if (capacity < 3u * sizeof(uint32_t))
    {
        result.status = CRASH_RECORD_BAD_LENGTH;
        result.actual_length = (uint32_t)capacity;
        *info = result;
        return result.status;
    }

    result.actual_length = CrashLogReadLE32(bytes + sizeof(uint32_t));
    result.version = CrashLogReadLE32(bytes + 2u * sizeof(uint32_t));
    if ((magic == CRASH_V3_MAGIC_OK || magic == CRASH_V3_MAGIC_BUSY) &&
        result.version == CRASH_LOG_V3_VERSION)
    {
        result.status = CRASH_RECORD_LEGACY_V3;
        *info = result;
        return result.status;
    }
    if (result.version != CRASH_LOG_VERSION)
    {
        result.status = CRASH_RECORD_BAD_VERSION;
        *info = result;
        return result.status;
    }
    if (result.actual_length != sizeof(CrashLog_t) || result.actual_length > capacity ||
        result.actual_length > CRASH_RECORD_CAPACITY)
    {
        result.status = CRASH_RECORD_BAD_LENGTH;
        *info = result;
        return result.status;
    }
    if (magic != CRASH_LOG_MAGIC_OK && magic != CRASH_LOG_MAGIC_BUSY)
    {
        result.status = CRASH_RECORD_BAD_MAGIC;
        *info = result;
        return result.status;
    }

    result.progress = CrashLogReadLE32(bytes + offsetof(CrashLog_t, progress));
    if (magic == CRASH_LOG_MAGIC_BUSY && result.progress != 4u)
    {
        result.status = CRASH_RECORD_WRITING;
        *info = result;
        return result.status;
    }

    // CRC从len字段开始，排除变化中的magic和末尾CRC字本身。
    const size_t crc_data_length = offsetof(CrashLog_t, crc32) - sizeof(uint32_t);
    const uint32_t expected_crc = CrashLogCRC32Bytes(bytes + sizeof(uint32_t), crc_data_length);
    const uint32_t stored_crc = CrashLogReadLE32(bytes + offsetof(CrashLog_t, crc32));
    if (expected_crc != stored_crc)
    {
        result.status = CRASH_RECORD_BAD_CRC;
        *info = result;
        return result.status;
    }

    result.status = (magic == CRASH_LOG_MAGIC_BUSY) ?
                        CRASH_RECORD_VALID_BUSY_FINAL : CRASH_RECORD_VALID;
    *info = result;
    return result.status;
}

uint32_t CrashTestPermitMagicNormalize(uint32_t stored_magic)
{
    if (stored_magic == CRASH_TEST_PERMIT_MAGIC ||
        stored_magic == CRASH_TEST_PERMIT_CONSUMED)
        return stored_magic;

    return 0u;
}
