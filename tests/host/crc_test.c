/** @file crc_test.c
 * @brief 统一 CRC 的主机行为测试；固定向量和独立逐位算法检查生产只读表。
 * @note 使用 TEST_CHECK，Release 下也执行检查；Linux 额外运行真实线程。
 */
#include "crc.h"
#include "bsp_crash_crc.h"
#include "bsp_crash.h"
#include "test_check.h"
#include <string.h>
#ifdef CRC_TEST_THREADS
#include <pthread.h>
#endif

// 参考算法始终从最高位处理，与生产实现的低位反射查表分别表达。
static uint32_t reverse_bits(uint32_t value, unsigned width)
{
    uint32_t result = 0u;
    for (unsigned bit = 0u; bit < width; ++bit)
    { result = (result << 1u) | (value & 1u); value >>= 1u; }
    return result;
}

static uint32_t reference(CRCAlgorithm_t algorithm, const uint8_t *data, size_t length)
{
    const CRCConfiguration_t *config = &CRCConfigurations[algorithm];
    uint32_t value = config->initial;
    uint32_t mask = 0xffffffffu >> (32u - config->width);
    // 每字节先按输入方向转换，再逐位展开正常表示的多项式。
    for (size_t index = 0u; index < length; ++index)
    {
        uint32_t byte = config->reflect_input ? reverse_bits(data[index], 8u) : data[index];
        value ^= byte << (config->width - 8u);
        for (unsigned bit = 0u; bit < 8u; ++bit)
            value = ((value << 1u) ^ ((value & (1u << (config->width - 1u))) ? config->polynomial : 0u)) & mask;
    }
    if (config->reflect_output) value = reverse_bits(value, config->width);
    return value ^ config->xor_output;
}

static void vectors(void)
{
    const uint8_t check[] = "123456789";
    const uint32_t expected[] = {0xa2u, 0x0bu, 0x4b37u, 0x6f91u, 0xcbf43926u};
    uint8_t bytes[513];
    for (size_t index = 0u; index < sizeof(bytes); ++index) bytes[index] = (uint8_t)(index * 73u + 19u);
    for (unsigned algorithm = 0u; algorithm < CRC_ALGORITHM_COUNT; ++algorithm)
    {
        uint32_t result;
        TEST_CHECK(CRCCompute((CRCAlgorithm_t)algorithm, check, 9u, &result) == CRC_OK);
        TEST_CHECK(result == expected[algorithm]);
        TEST_CHECK(CRCCompute((CRCAlgorithm_t)algorithm, NULL, 0u, &result) == CRC_OK);
        TEST_CHECK(result == reference((CRCAlgorithm_t)algorithm, NULL, 0u));
        // 全部单字节以及跨表边界长度，不只比较同一组组包／解析函数。
        for (unsigned byte = 0u; byte < 256u; ++byte)
        {
            uint8_t input = (uint8_t)byte;
            TEST_CHECK(CRCCompute((CRCAlgorithm_t)algorithm, &input, 1u, &result) == CRC_OK);
            TEST_CHECK(result == reference((CRCAlgorithm_t)algorithm, &input, 1u));
        }
        for (size_t length = 0u; length <= sizeof(bytes); ++length)
        {
            TEST_CHECK(CRCCompute((CRCAlgorithm_t)algorithm, bytes, length, &result) == CRC_OK);
            TEST_CHECK(result == reference((CRCAlgorithm_t)algorithm, bytes, length));
            CRCContext_t context = {0};
            TEST_CHECK(CRCInit(&context, (CRCAlgorithm_t)algorithm) == CRC_OK);
            size_t split = length / 3u;
            TEST_CHECK(CRCUpdate(&context, bytes, split) == CRC_OK);
            TEST_CHECK(CRCUpdate(&context, NULL, 0u) == CRC_OK);
            TEST_CHECK(CRCUpdate(&context, bytes + split, length - split) == CRC_OK);
            uint32_t chunked;
            TEST_CHECK(CRCFinal(&context, &chunked) == CRC_OK && chunked == result);
            TEST_CHECK(CRCFinal(&context, &chunked) == CRC_OK && chunked == result);
        }
    }
    const uint32_t words[] = {0x04030201u, 0x08070605u};
    uint32_t crash, ordinary;
    const uint8_t little_endian[] = {1,2,3,4,5,6,7,8};
    TEST_CHECK(CrashCRC32Words(words, 2u, &crash));
    TEST_CHECK(CRCCompute(CRC32_IEEE, little_endian, sizeof(little_endian), &ordinary) == CRC_OK);
    TEST_CHECK(crash == ordinary && crash == 0x3fca88c5u);
    // 使用历史记录的完整布局；头部40字节不参与 CRC，余下212字节逐字处理。
    TEST_CHECK(sizeof(CrashLog_t) == 252u && offsetof(CrashLog_t, task_name) == 40u);
    uint32_t record_words[53];
    for (unsigned index = 0u; index < 53u; ++index) record_words[index] = 0x01020304u * index;
    TEST_CHECK(CrashCRC32Words(record_words, 53u, &crash));
    TEST_CHECK(crash == reference(CRC32_IEEE, (const uint8_t *)record_words, sizeof(record_words)));
}

static void failures(void)
{
    CRCContext_t context = {0};
    uint32_t output = 0x12345678u;
    TEST_CHECK(CRCFinal(&context, &output) == CRC_ERROR_STATE && output == 0x12345678u);
    TEST_CHECK(CRCUpdate(&context, NULL, 0u) == CRC_ERROR_STATE);
    TEST_CHECK(CRCInit(NULL, CRC32_IEEE) == CRC_ERROR_ARGUMENT);
    TEST_CHECK(CRCInit(&context, CRC32_IEEE) == CRC_OK);
    const CRCContext_t saved = context;
    TEST_CHECK(CRCInit(&context, (CRCAlgorithm_t)-1) == CRC_ERROR_ALGORITHM);
    TEST_CHECK(memcmp(&context, &saved, sizeof(saved)) == 0);
    TEST_CHECK(CRCUpdate(&context, NULL, 1u) == CRC_ERROR_ARGUMENT);
    TEST_CHECK(memcmp(&context, &saved, sizeof(saved)) == 0);
    TEST_CHECK(CRCFinal(&context, NULL) == CRC_ERROR_ARGUMENT);
    TEST_CHECK(CRCFinal(NULL, &output) == CRC_ERROR_ARGUMENT);
    TEST_CHECK(CRCUpdate(NULL, NULL, 0u) == CRC_ERROR_ARGUMENT);
    TEST_CHECK(CRCCompute(CRC32_IEEE, NULL, 1u, &output) == CRC_ERROR_ARGUMENT);
    TEST_CHECK(CRCCompute(CRC_ALGORITHM_COUNT, NULL, 0u, &output) == CRC_ERROR_ALGORITHM);
    TEST_CHECK(CRCCompute(CRC32_IEEE, NULL, 0u, NULL) == CRC_ERROR_ARGUMENT);
    TEST_CHECK(output == 0x12345678u);
    context.algorithm = CRC_ALGORITHM_COUNT;
    TEST_CHECK(CRCFinal(&context, &output) == CRC_ERROR_ALGORITHM);
    context = saved;
    context.algorithm = CRC8_CAN_A5;
    TEST_CHECK(CRCFinal(&context, &output) == CRC_ERROR_STATE);
    TEST_CHECK(!CrashCRC32Words(NULL, 1u, &output) && output == 0x12345678u);
    TEST_CHECK(!CrashCRC32Words(NULL, 0u, NULL));
    TEST_CHECK(CrashCRC32Words(NULL, 0u, &output) && output == 0u);
}

static void *instance_run(void *argument)
{
    CRCAlgorithm_t algorithm = *(const CRCAlgorithm_t *)argument;
    for (unsigned pass = 0u; pass < 100u; ++pass)
    {
        CRCContext_t first = {0}, second = {0};
        uint8_t bytes[97];
        memset(bytes, (int)pass, sizeof(bytes));
        TEST_CHECK(CRCInit(&first, algorithm) == CRC_OK);
        TEST_CHECK(CRCInit(&second, CRC8_CAN_A5) == CRC_OK);
        for (size_t index = 0u; index < sizeof(bytes); ++index)
        {
            TEST_CHECK(CRCUpdate(&first, bytes + index, 1u) == CRC_OK);
            TEST_CHECK(CRCUpdate(&second, bytes + index, 1u) == CRC_OK);
            uint32_t partial;
            TEST_CHECK(CRCFinal(&first, &partial) == CRC_OK);
            TEST_CHECK(partial == reference(algorithm, bytes, index + 1u));
        }
        uint32_t result;
        TEST_CHECK(CRCFinal(&second, &result) == CRC_OK);
        TEST_CHECK(result == reference(CRC8_CAN_A5, bytes, sizeof(bytes)));
    }
    return NULL;
}

int main(int argc, char **argv)
{
    TEST_CHECK(argc == 2);
    if (strcmp(argv[1], "vectors") == 0) vectors();
    else if (strcmp(argv[1], "failures") == 0) failures();
    else
    {
        CRCAlgorithm_t algorithms[CRC_ALGORITHM_COUNT];
#ifdef CRC_TEST_THREADS
        pthread_t threads[CRC_ALGORITHM_COUNT];
#endif
        for (unsigned index = 0u; index < CRC_ALGORITHM_COUNT; ++index)
        {
            algorithms[index] = (CRCAlgorithm_t)index;
#ifdef CRC_TEST_THREADS
            TEST_CHECK(pthread_create(&threads[index], NULL, instance_run, &algorithms[index]) == 0);
#else
            instance_run(&algorithms[index]);
#endif
        }
#ifdef CRC_TEST_THREADS
        for (unsigned index = 0u; index < CRC_ALGORITHM_COUNT; ++index)
            TEST_CHECK(pthread_join(threads[index], NULL) == 0);
#endif
    }
    return 0;
}
