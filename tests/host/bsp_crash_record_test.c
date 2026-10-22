/*
 * @file bsp_crash_record_test.c
 * @brief 用固定字节验证崩溃记录版本、长度和CRC检查。
 * 该测试链接生产检查函数，不访问目标硬件；TEST_CHECK在Release下仍执行。
 * @author MC-02 contributors
 * @license BSD-3-Clause; see LICENSE.
 */
#include "bsp_crash_record.h"

#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "test_check.h"

_Static_assert(sizeof(CrashLog_t) == 336u, "v4记录固定为336字节");
_Static_assert(offsetof(CrashLog_t, crc32) == 332u, "CRC固定在记录末尾");

static void WriteLE32(uint8_t *bytes, uint32_t value)
{
    bytes[0] = (uint8_t)value;
    bytes[1] = (uint8_t)(value >> 8u);
    bytes[2] = (uint8_t)(value >> 16u);
    bytes[3] = (uint8_t)(value >> 24u);
}

static void MakeValidV4(uint8_t bytes[sizeof(CrashLog_t)])
{
    (void)memset(bytes, 0, sizeof(CrashLog_t));
    WriteLE32(bytes, CRASH_LOG_MAGIC_OK);
    WriteLE32(bytes + 4u, (uint32_t)sizeof(CrashLog_t));
    WriteLE32(bytes + 8u, CRASH_LOG_VERSION);
    WriteLE32(bytes + 12u, 4u);
    WriteLE32(bytes + 16u, 1u);
    WriteLE32(bytes + 20u, CRASH_TYPE_HARDFAULT);
    /* 该答案由独立的zlib CRC32-IEEE计算，范围为偏移4到331。 */
    WriteLE32(bytes + offsetof(CrashLog_t, crc32), 0x635E3450u);
}

static void TestFixedV4VectorAndBusyFinal(void)
{
    uint8_t bytes[sizeof(CrashLog_t)];
    CrashRecordInfo_t info = {0};

    MakeValidV4(bytes);
    TEST_CHECK(CrashLogRecordInspect(bytes, sizeof(bytes), &info) == CRASH_RECORD_VALID);
    TEST_CHECK(info.actual_length == sizeof(bytes));
    TEST_CHECK(info.expected_length == sizeof(bytes));
    TEST_CHECK(info.version == CRASH_LOG_VERSION);
    TEST_CHECK(info.progress == 4u);

    WriteLE32(bytes, CRASH_LOG_MAGIC_BUSY);
    TEST_CHECK(CrashLogRecordInspect(bytes, sizeof(bytes), &info) ==
               CRASH_RECORD_VALID_BUSY_FINAL);
    WriteLE32(bytes + 12u, 3u);
    TEST_CHECK(CrashLogRecordInspect(bytes, sizeof(bytes), &info) == CRASH_RECORD_WRITING);
    TEST_CHECK(info.progress == 3u);
}

static void TestV3AndMalformedRecords(void)
{
    uint8_t legacy[252] = {0u};
    uint8_t bytes[sizeof(CrashLog_t)];
    CrashRecordInfo_t info = {0};

    WriteLE32(legacy, 0x4B4F5243u);
    WriteLE32(legacy + 4u, (uint32_t)sizeof(legacy));
    WriteLE32(legacy + 8u, CRASH_LOG_V3_VERSION);
    TEST_CHECK(CrashLogRecordInspect(legacy, sizeof(legacy), &info) == CRASH_RECORD_LEGACY_V3);
    TEST_CHECK(info.version == CRASH_LOG_V3_VERSION);
    TEST_CHECK(info.actual_length == sizeof(legacy));

    MakeValidV4(bytes);
    TEST_CHECK(CrashLogRecordInspect(bytes, 0u, &info) == CRASH_RECORD_EMPTY);
    TEST_CHECK(CrashLogRecordInspect(NULL, 1u, &info) == CRASH_RECORD_BAD_ARGUMENT);
    TEST_CHECK(CrashLogRecordInspect(bytes, 3u, &info) == CRASH_RECORD_BAD_LENGTH);
    TEST_CHECK(CrashLogRecordInspect(bytes, sizeof(bytes) - 1u, &info) ==
               CRASH_RECORD_BAD_LENGTH);
    TEST_CHECK(CrashLogRecordInspect(bytes, CRASH_RECORD_CAPACITY + 1u, &info) ==
               CRASH_RECORD_BAD_ARGUMENT);
    TEST_CHECK(CrashLogRecordInspect(bytes, sizeof(bytes), NULL) == CRASH_RECORD_BAD_ARGUMENT);

    WriteLE32(bytes, 0x12345678u);
    TEST_CHECK(CrashLogRecordInspect(bytes, sizeof(bytes), &info) == CRASH_RECORD_BAD_MAGIC);
    MakeValidV4(bytes);
    WriteLE32(bytes + 8u, 99u);
    TEST_CHECK(CrashLogRecordInspect(bytes, sizeof(bytes), &info) == CRASH_RECORD_BAD_VERSION);
    MakeValidV4(bytes);
    WriteLE32(bytes + 4u, (uint32_t)sizeof(bytes) - 4u);
    TEST_CHECK(CrashLogRecordInspect(bytes, sizeof(bytes), &info) == CRASH_RECORD_BAD_LENGTH);
    MakeValidV4(bytes);
    bytes[offsetof(CrashLog_t, type)] ^= 1u;
    TEST_CHECK(CrashLogRecordInspect(bytes, sizeof(bytes), &info) == CRASH_RECORD_BAD_CRC);
    MakeValidV4(bytes);
    bytes[offsetof(CrashLog_t, crc32)] ^= 1u;
    TEST_CHECK(CrashLogRecordInspect(bytes, sizeof(bytes), &info) == CRASH_RECORD_BAD_CRC);
}

static void TestCrashPermitMagicNormalization(void)
{
    TEST_CHECK(CrashTestPermitMagicNormalize(0u) == 0u);
    TEST_CHECK(CrashTestPermitMagicNormalize(CRASH_TEST_PERMIT_MAGIC) ==
               CRASH_TEST_PERMIT_MAGIC);
    TEST_CHECK(CrashTestPermitMagicNormalize(CRASH_TEST_PERMIT_CONSUMED) ==
               CRASH_TEST_PERMIT_CONSUMED);
    TEST_CHECK(CrashTestPermitMagicNormalize(0x0802ADE0u) == 0u);
    TEST_CHECK(CrashTestPermitMagicNormalize(UINT32_MAX) == 0u);
}

int main(void)
{
    TestFixedV4VectorAndBusyFinal();
    TestV3AndMalformedRecords();
    TestCrashPermitMagicNormalization();
    return 0;
}
