/**
 * @file float_to_string_test.c
 * @brief 通过生产 Float2Str 检查三位小数、失败保留、写入范围及不同数组独立使用。
 * @note CTest按场景启动；reference模式接收float位形，供独立Python十进制参考核对。
 */
#include "float_to_string.h"
#include "test_check.h"
#include <float.h>
#include <inttypes.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#ifdef FLOAT2STR_TEST_THREADS
#include <pthread.h>
#endif

// 给数组两侧和未使用尾部放入哨兵，确认成功只写完整文字及结尾0。
static void expect_value(float value, const char *expected)
{
    unsigned char guarded[48];
    const size_t offset = 8u;
    size_t written = strlen(expected) + 1u;
    TEST_CHECK(written <= FLOAT2STR_BUFFER_SIZE);
    memset(guarded, 0xA5, sizeof(guarded));
    TEST_CHECK(Float2Str((char *)guarded + offset, written, value) == FLOAT2STR_OK);
    TEST_CHECK(strcmp((char *)guarded + offset, expected) == 0);
    for (size_t i = 0u; i < sizeof(guarded); ++i)
        if (i < offset || i >= offset + written)
            TEST_CHECK(guarded[i] == 0xA5u);
}

static void values(void)
{
    const struct { float input; const char *expected; } cases[] = {
        {0.0f, "0.000"}, {-0.0f, "0.000"}, {1.0f, "1.000"}, {-1.0f, "-1.000"},
        {1.2f, "1.200"}, {-1.2f, "-1.200"}, {0.0004f, "0.000"}, {-0.0004f, "0.000"},
        {0.0005f, "0.001"}, {-0.0005f, "-0.001"}, {0.0625f, "0.063"}, {-0.0625f, "-0.063"},
        {1.0625f, "1.063"}, {-1.0625f, "-1.063"}, {1.0005f, "1.000"},
        {9.9996f, "10.000"}, {-9.9996f, "-10.000"}, {999.9996f, "1000.000"},
        {FLT_TRUE_MIN, "0.000"}, {-FLT_TRUE_MIN, "0.000"}, {FLT_MIN, "0.000"},
        {2147483520.0f, "2147483520.000"}, {-2147483648.0f, "-2147483648.000"}
    };
    for (size_t i = 0u; i < sizeof(cases) / sizeof(cases[0]); ++i)
        expect_value(cases[i].input, cases[i].expected);

    // 0.0625能被float精确保存，向两侧走一份可表示值，检查半数两侧的选择。
    expect_value(nextafterf(0.0625f, 0.0f), "0.062");
    expect_value(nextafterf(0.0625f, INFINITY), "0.063");
    expect_value(nextafterf(-0.0625f, 0.0f), "-0.062");
    expect_value(nextafterf(-0.0625f, -INFINITY), "-0.063");

    char reused[FLOAT2STR_BUFFER_SIZE];
    TEST_CHECK(Float2Str(reused, sizeof(reused), -2147483648.0f) == FLOAT2STR_OK);
    TEST_CHECK(Float2Str(reused, sizeof(reused), 0.0f) == FLOAT2STR_OK);
    TEST_CHECK(strcmp(reused, "0.000") == 0);
}

static void expect_failure(float value, size_t capacity, Float2StrStatus_t expected)
{
    unsigned char guarded[48], before[48];
    memset(guarded, 0xC7, sizeof(guarded));
    memcpy(before, guarded, sizeof(before));
    TEST_CHECK(Float2Str((char *)guarded + 8u, capacity, value) == expected);
    TEST_CHECK(memcmp(before, guarded, sizeof(guarded)) == 0);
}

static void failures(void)
{
    TEST_CHECK(Float2Str(NULL, FLOAT2STR_BUFFER_SIZE, 1.0f) == FLOAT2STR_BAD_ARGUMENT);
    TEST_CHECK(Float2Str(NULL, 0u, NAN) == FLOAT2STR_BAD_ARGUMENT);
    const float nonfinite[] = {NAN, INFINITY, -INFINITY};
    for (size_t i = 0u; i < sizeof(nonfinite) / sizeof(nonfinite[0]); ++i)
        expect_failure(nonfinite[i], FLOAT2STR_BUFFER_SIZE, FLOAT2STR_NONFINITE);
    const float out_of_range[] = {FLT_MAX, -FLT_MAX, 2147483648.0f,
        nextafterf(-2147483648.0f, -INFINITY)};
    for (size_t i = 0u; i < sizeof(out_of_range) / sizeof(out_of_range[0]); ++i)
        expect_failure(out_of_range[i], FLOAT2STR_BUFFER_SIZE, FLOAT2STR_OUT_OF_RANGE);

    // 少一个字节也不能接受，且数组中原来未结束的文字不会被部分改写。
    for (size_t capacity = 0u; capacity < sizeof("1.000"); ++capacity)
        expect_failure(1.0f, capacity, FLOAT2STR_BUFFER_TOO_SMALL);
    for (size_t capacity = 0u; capacity < FLOAT2STR_BUFFER_SIZE; ++capacity)
        expect_failure(-2147483648.0f, capacity, FLOAT2STR_BUFFER_TOO_SMALL);
}

typedef struct
{
    float value;
    const char *expected;
    char buffer[FLOAT2STR_BUFFER_SIZE];
} Worker;

static void step(Worker *worker)
{
    TEST_CHECK(Float2Str(worker->buffer, sizeof(worker->buffer), worker->value) == FLOAT2STR_OK);
    TEST_CHECK(strcmp(worker->buffer, worker->expected) == 0);
}

#ifdef FLOAT2STR_TEST_THREADS
static void *run_worker(void *argument)
{
    Worker *worker = argument;
    for (unsigned i = 0u; i < 10000u; ++i)
        step(worker);
    return NULL;
}
#endif

static void instances(void)
{
    Worker a = {.value = -1.0625f, .expected = "-1.063"};
    Worker b = {.value = 999.9996f, .expected = "1000.000"};
    for (unsigned i = 0u; i < 10000u; ++i)
    {
        step(&a);
        step(&b);
    }
#ifdef FLOAT2STR_TEST_THREADS
    pthread_t ta, tb;
    TEST_CHECK(pthread_create(&ta, NULL, run_worker, &a) == 0);
    TEST_CHECK(pthread_create(&tb, NULL, run_worker, &b) == 0);
    TEST_CHECK(pthread_join(ta, NULL) == 0);
    TEST_CHECK(pthread_join(tb, NULL) == 0);
#endif
}

// 只把给定32位位形交给生产函数；参考答案由Python的Decimal独立生成。
static void reference_stream(void)
{
    TEST_CHECK(sizeof(float) == sizeof(uint32_t) && FLT_RADIX == 2 && FLT_MANT_DIG == 24);
    char line[32];
    while (fgets(line, sizeof(line), stdin) != NULL)
    {
        uint32_t bits;
        char extra;
        TEST_CHECK(sscanf(line, "%" SCNx32 " %c", &bits, &extra) == 1);
        float value;
        memcpy(&value, &bits, sizeof(value));
        char output[FLOAT2STR_BUFFER_SIZE];
        Float2StrStatus_t status = Float2Str(output, sizeof(output), value);
        if (status == FLOAT2STR_OK)
            TEST_CHECK(printf("OK %s\n", output) > 0);
        else if (status == FLOAT2STR_NONFINITE)
            TEST_CHECK(puts("NONFINITE") >= 0);
        else
        {
            TEST_CHECK(status == FLOAT2STR_OUT_OF_RANGE);
            TEST_CHECK(puts("OUT_OF_RANGE") >= 0);
        }
    }
    TEST_CHECK(!ferror(stdin));
}

/** @brief 执行指定场景；名称错误时明确失败，所有检查在Release仍执行。 */
int main(int argc, char **argv)
{
    TEST_CHECK(argc == 2);
    if (strcmp(argv[1], "values") == 0) values();
    else if (strcmp(argv[1], "failures") == 0) failures();
    else if (strcmp(argv[1], "instances") == 0) instances();
    else
    {
        TEST_CHECK(strcmp(argv[1], "reference") == 0);
        reference_stream();
    }
    return EXIT_SUCCESS;
}
