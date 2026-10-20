/** @file log_test.c
 * @brief 执行生产日志的混排、容量、失败和自测路径；只替代RTT、时钟及一次格式化失败。
 * @note 超长文字来自进程参数，编译时已知的超长常量由独立编译负例验证。
 */
#include "bsp_log.h"
#include "test_check.h"
#include <stdarg.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

static char sent[4096];
static size_t sent_size;
static unsigned writes, initialized;
static int fail_next;

void SEGGER_RTT_Init(void) { ++initialized; }
uint32_t HAL_GetTick(void) { return 0u; }

unsigned SEGGER_RTT_WriteString(unsigned index, const char *text)
{
    TEST_CHECK(initialized == 1u);
    TEST_CHECK(index == BUFFER_INDEX);
    size_t size = strlen(text);
    TEST_CHECK(sent_size + size < sizeof(sent));
    memcpy(sent + sent_size, text, size + 1u);
    sent_size += size;
    ++writes;
    return (unsigned)size;
}

#ifdef _WIN32
#define REAL_VSNPRINTF __real___mingw_vsnprintf
#define WRAP_VSNPRINTF __wrap___mingw_vsnprintf
#define WRAP_SNPRINTF __wrap___mingw_snprintf
#else
#define REAL_VSNPRINTF __real_vsnprintf
#define WRAP_VSNPRINTF __wrap_vsnprintf
#define WRAP_SNPRINTF __wrap_snprintf
#endif
int REAL_VSNPRINTF(char *out, size_t capacity, const char *format, va_list args);

// 正常路径仍使用标准库；仅在选定一次调用返回负值，避免执行不合法的可变参数。
int WRAP_VSNPRINTF(char *out, size_t capacity, const char *format, va_list args)
{
    if (fail_next)
    {
        fail_next = 0;
        return -1;
    }
    return REAL_VSNPRINTF(out, capacity, format, args);
}

int WRAP_SNPRINTF(char *out, size_t capacity, const char *format, ...)
{
    va_list args;
    va_start(args, format);
    int result = WRAP_VSNPRINTF(out, capacity, format, args);
    va_end(args);
    return result;
}

#if defined(__linux__)
// Ubuntu的受检标准库入口也走同一次失败注入，正常计算仍交给真实vsnprintf。
int __wrap___vsnprintf_chk(char *out, size_t capacity, int flag, size_t object_size,
                          const char *format, va_list args)
{
    (void)flag;
    (void)object_size;
    return WRAP_VSNPRINTF(out, capacity, format, args);
}
int __wrap___snprintf_chk(char *out, size_t capacity, int flag, size_t object_size,
                         const char *format, ...)
{
    (void)flag;
    (void)object_size;
    va_list args;
    va_start(args, format);
    int result = WRAP_VSNPRINTF(out, capacity, format, args);
    va_end(args);
    return result;
}
#endif

static void clear_output(void)
{
    memset(sent, 0xA5, sizeof(sent));
    sent[0] = '\0';
    sent_size = 0u;
    writes = 0u;
}

static void values(void)
{
    TEST_CHECK(initialized == 1u);
    clear_output();
    TEST_CHECK(PrintLog("%s %d %.3f", "OK", -7, 1.25) == (int)strlen("OK -7 1.250"));
    TEST_CHECK(strcmp(sent, "OK -7 1.250") == 0 && writes == 1u);
    clear_output();
    LOGINFO("n=%u", 3u);
    TEST_CHECK(strcmp(sent, "  " RTT_CTRL_TEXT_BRIGHT_GREEN "I:n=3\r\n" RTT_CTRL_RESET) == 0);
}

static void boundaries(const char *input)
{
    TEST_CHECK(strlen(input) == 300u);
    const size_t lengths[] = {0u, 236u, 237u, 245u, 246u, 247u, 255u, 256u, 300u};
    // 同一份运行时文字从不同位置开始读取，分别走完整输出和少一字节的边界。
    for (size_t i = 0u; i < sizeof(lengths) / sizeof(lengths[0]); ++i)
    {
        size_t length = lengths[i];
        const char *text = input + 300u - length;
        clear_output();
        TEST_CHECK(PrintLog("%s", text) == (int)length);
        size_t actual = length < 255u ? length : 255u;
        TEST_CHECK(sent_size == actual && memcmp(sent, text, actual) == 0);
        TEST_CHECK(sent[actual] == '\0' && (unsigned char)sent[actual + 1u] == 0xA5u);

        clear_output();
        LOG("%s", text);
        actual = length < 245u ? length : 245u;
        TEST_CHECK(sent_size == 2u + actual + strlen("\r\n" RTT_CTRL_RESET));
        TEST_CHECK(memcmp(sent, "  ", 2u) == 0 && memcmp(sent + 2u, text, actual) == 0);
        TEST_CHECK(strcmp(sent + 2u + actual, "\r\n" RTT_CTRL_RESET) == 0);

        clear_output();
        LOGINFO("%s", text);
        const char *prefix = "  " RTT_CTRL_TEXT_BRIGHT_GREEN "I:";
        size_t prefix_size = strlen(prefix);
        size_t maximum = LOG_BUF_SIZE - LOG_BODY_TAIL - 1u - prefix_size;
        actual = length < maximum ? length : maximum;
        TEST_CHECK(sent_size == prefix_size + actual + strlen("\r\n" RTT_CTRL_RESET));
        TEST_CHECK(memcmp(sent, prefix, prefix_size) == 0 && memcmp(sent + prefix_size, text, actual) == 0);
        TEST_CHECK(strcmp(sent + prefix_size + actual, "\r\n" RTT_CTRL_RESET) == 0);
    }
}

static void failures(const char *text)
{
    clear_output();
    fail_next = 1;
    TEST_CHECK(PrintLog("%s", text) == -1);
    TEST_CHECK(writes == 0u && fail_next == 0);
    fail_next = 1;
    // 输入来自进程参数，编译器不能把已知合法常量的返回长度提前算成常数。
    LOGWARNING("%s", text);
    TEST_CHECK(writes == 1u && fail_next == 0);
    TEST_CHECK(strcmp(sent, "\r\n" RTT_CTRL_RESET) == 0);
}

/** @brief 执行指定生产路径；错误场景名和参数缺失均返回失败。 */
int main(int argc, char **argv)
{
    TEST_CHECK(argc >= 2);
    BSPLogInit();
    if (strcmp(argv[1], "values") == 0) values();
    else if (strcmp(argv[1], "boundaries") == 0)
    {
        TEST_CHECK(argc == 3);
        boundaries(argv[2]);
    }
    else if (strcmp(argv[1], "failures") == 0)
    {
        TEST_CHECK(argc == 3);
        failures(argv[2]);
    }
    else
    {
        TEST_CHECK(strcmp(argv[1], "selftest") == 0);
        clear_output();
        LogSelfTest();
        TEST_CHECK(writes == 6u && strstr(sent, "tail_s=OK") != NULL);
        TEST_CHECK(strstr(sent, "100% value=2") != NULL);
    }
    return EXIT_SUCCESS;
}
