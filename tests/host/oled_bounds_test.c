/** @file oled_bounds_test.c
 * @brief 通过生产刷新入口读取像素，验证OLED绘图、字符及格式化边界。
 * HAL只记录发送字节；链接器包装vsnprintf仅用于格式化失败注入。
 */
#include "oled.h"
#include "main.h"
#include "test_check.h"
#include <stdio.h>
#include <stdarg.h>
#include <string.h>

I2C_HandleTypeDef hi2c2;
static uint8_t screen[128][8];
static unsigned writes;
static int capture;
static int fail_format;
static size_t format_capacity;

int __real_vsnprintf(char *buffer, size_t size, const char *format, va_list arguments);
int __wrap_vsnprintf(char *buffer, size_t size, const char *format, va_list arguments)
{
    format_capacity = size;
    if (fail_format) return -1;
    return __real_vsnprintf(buffer, size, format, arguments);
}
int __real___vsnprintf_chk(char *buffer, size_t size, int flag, size_t object_size,
                           const char *format, va_list arguments);
int __wrap___vsnprintf_chk(char *buffer, size_t size, int flag, size_t object_size,
                           const char *format, va_list arguments)
{
    format_capacity = size;
    if (fail_format) return -1;
    return __real___vsnprintf_chk(buffer, size, flag, object_size, format, arguments);
}

HAL_StatusTypeDef HAL_I2C_Master_Transmit(I2C_HandleTypeDef *handle, uint16_t address,
                                        uint8_t *data, uint16_t length, uint32_t timeout)
{
    TEST_CHECK(handle == &hi2c2 && address == OLED_I2C_ADDRESS);
    TEST_CHECK(length == 2u && timeout == 10u);
    if (capture)
    {
        unsigned page = writes / 131u;
        unsigned offset = writes % 131u;
        TEST_CHECK(page < 8u);
        // 每页先发三条定位命令，再发128个像素字节；同时检查线上顺序。
        if (offset < 3u)
        {
            const uint8_t expected[] = {(uint8_t)(0xb0u + page), 0x10u, 0x00u};
            TEST_CHECK(data[0] == 0u && data[1] == expected[offset]);
        }
        else
        {
            TEST_CHECK(data[0] == 0x40u);
            screen[offset - 3u][page] = data[1];
        }
    }
    ++writes;
    return HAL_OK;
}

static void read_screen(void)
{
    writes = 0u;
    capture = 1;
    OLED_refresh_gram();
    capture = 0;
    TEST_CHECK(writes == 8u * 131u);
}

static void clear_screen(void)
{
    OLED_operate_gram(PEN_CLEAR);
}

static void expect_pixels(const uint8_t (*points)[2], size_t count)
{
    uint8_t expected[128][8] = {{0}};
    for (size_t i = 0; i < count; ++i)
        expected[points[i][0]][points[i][1] / 8u] |= (uint8_t)(1u << (points[i][1] % 8u));
    read_screen();
    TEST_CHECK(memcmp(expected, screen, sizeof(screen)) == 0);
}

static void lines(void)
{
    // 固定答案包含陡坡及负斜率；反向时必须得到同一组像素。
    static const uint8_t shallow[][2] = {{2,2},{3,2},{4,3},{5,3},{6,4},{7,4}};
    static const uint8_t steep[][2] = {{2,2},{2,3},{3,4},{3,5},{4,6},{4,7}};
    for (unsigned variant = 0; variant < 8u; ++variant)
    {
        uint8_t points[6][2];
        const uint8_t (*base)[2] = (variant & 1u) ? steep : shallow;
        for (size_t i = 0; i < 6u; ++i)
        {
            points[i][0] = (variant & 2u) ? (uint8_t)(20u - base[i][0]) : base[i][0];
            points[i][1] = (variant & 4u) ? (uint8_t)(20u - base[i][1]) : base[i][1];
        }
        for (unsigned reverse = 0; reverse < 2u; ++reverse)
        {
            clear_screen();
            unsigned first = reverse ? 5u : 0u, last = reverse ? 0u : 5u;
            OLED_draw_line(points[first][0], points[first][1], points[last][0], points[last][1], PEN_WRITE);
            expect_pixels(points, 6u);
        }
    }
    clear_screen();
    OLED_draw_line(127, 63, 127, 63, PEN_WRITE);
    const uint8_t point[][2] = {{127,63}};
    expect_pixels(point, 1u);
    clear_screen();
    OLED_draw_line(127, 0, 0, 0, PEN_WRITE);
    OLED_draw_line(0, 63, 0, 0, PEN_WRITE);
    read_screen();
    for (unsigned x = 0; x < 128u; ++x)
        for (unsigned page = 0; page < 8u; ++page)
            TEST_CHECK(screen[x][page] == (x == 0u ? 0xffu : (page == 0u ? 1u : 0u)));
    // 反转同一条线两次还原；清除只影响该线。
    OLED_draw_line(0, 0, 127, 0, PEN_INVERSION);
    OLED_draw_line(127, 0, 0, 0, PEN_INVERSION);
    OLED_draw_line(0, 0, 0, 63, PEN_CLEAR);
    read_screen();
    TEST_CHECK(screen[0][0] == 0u && screen[127][0] == 1u);
}

static void invalid(void)
{
    OLED_operate_gram(PEN_WRITE);
    OLED_draw_point(-1, 0, PEN_CLEAR);
    OLED_draw_point(0, 64, PEN_CLEAR);
    OLED_draw_point(0, 0, (pen_typedef)99);
    read_screen(); TEST_CHECK(screen[0][0] == 0xffu);
    OLED_draw_line(0, 0, 128, 1, PEN_CLEAR);
    OLED_draw_line(0, 255, 0, 0, PEN_CLEAR);
    OLED_draw_line(0, 0, 255, 0, PEN_CLEAR);
    read_screen(); TEST_CHECK(screen[0][0] == 0xffu);
    OLED_draw_line(0, 0, 1, 1, (pen_typedef)99);
    OLED_operate_gram((pen_typedef)99);
    read_screen();
    TEST_CHECK(screen[0][0] == 0xffu);
    OLED_show_char(5, 0, 'A');
    OLED_show_char(0, 21, 'A');
    OLED_show_char(255, 255, 'A');
    OLED_show_char(0, 0, 0x1f);
    OLED_show_char(0, 0, 0x7f);
    OLED_show_char(0, 0, 0xff);
    read_screen(); TEST_CHECK(screen[0][0] == 0xffu);
    OLED_show_string(0, 0, NULL);
    OLED_show_string(5, 0, (uint8_t *)"A");
    OLED_show_string(0, 21, (uint8_t *)"A");
    OLED_show_string(0, 0, (uint8_t *)"ABC\001");
    read_screen(); TEST_CHECK(screen[0][0] == 0xffu);
    OLED_printf(5, 0, "%s", "A");
    OLED_printf(0, 21, "%s", "A");
    OLED_printf(0, 0, NULL);
    OLED_printf(0, 0, "%s", "ABC\177");
    fail_format = 1;
    OLED_printf(0, 0, "%s", "A");
    fail_format = 0;
    read_screen();
    for (unsigned x = 0; x < 128u; ++x)
        for (unsigned p = 0; p < 8u; ++p)
            if (screen[x][p] != 0xffu)
            {
                (void)fprintf(stderr, "unexpected pixel byte x=%u page=%u value=%u\n", x, p, screen[x][p]);
                TEST_CHECK(0);
            }
    writes = 0u;
    OLED_set_pos(128, 0);
    OLED_set_pos(0, 8);
    OLED_set_pos(255, 255);
    TEST_CHECK(writes == 0u);
    OLED_set_pos(127, 7);
    TEST_CHECK(writes == 3u);
}

static void characters(void)
{
    // 与生产字库分别编译的已知A字形，检查列内像素顺序。
    static const char *glyph[] = {"......", "......", "..#...", "..#...", "..##..", ".#.#..",
                                 ".#.#..", ".####.", ".#..#.", "##..##", "......", "......"};
    clear_screen();
    OLED_show_char(0, 0, 'A');
    read_screen();
    for (unsigned y = 0; y < 12u; ++y)
        for (unsigned x = 0; x < 6u; ++x)
            TEST_CHECK(((screen[x][y / 8u] >> (y % 8u)) & 1u) == (glyph[y][x] == '#'));
    // 全部95个合法字符在最后一格绘制；与左上角字形平移结果逐像素对照。
    for (unsigned ch = 0x20u; ch <= 0x7eu; ++ch)
    {
        uint8_t reference[6][12];
        clear_screen();
        OLED_show_char(0, 0, (uint8_t)ch);
        read_screen();
        for (unsigned x = 0; x < 6u; ++x)
            for (unsigned y = 0; y < 12u; ++y)
                reference[x][y] = (screen[x][y / 8u] >> (y % 8u)) & 1u;
        clear_screen();
        OLED_show_char(4, 20, (uint8_t)ch);
        read_screen();
        for (unsigned x = 0; x < 128u; ++x)
            for (unsigned y = 0; y < 64u; ++y)
            {
                unsigned expected = (x >= 120u && x < 126u && y >= 48u && y < 60u)
                    ? reference[x - 120u][y - 48u] : 0u;
                TEST_CHECK(((screen[x][y / 8u] >> (y % 8u)) & 1u) == expected);
            }
    }
}

static void strings(void)
{
    uint8_t expected[128][8];
    clear_screen();
    OLED_show_char(3, 20, 'A'); OLED_show_char(4, 0, 'B');
    read_screen(); memcpy(expected, screen, sizeof(expected));
    clear_screen(); OLED_show_string(3, 20, (uint8_t *)"AB"); read_screen();
    TEST_CHECK(memcmp(expected, screen, sizeof(expected)) == 0);
    // 恰好占满最后一格的非零结尾数组：到屏幕末端就停止，不再读下一字节。
    uint8_t last[] = {'A'};
    clear_screen(); OLED_show_char(4, 20, 'A'); read_screen(); memcpy(expected, screen, sizeof(expected));
    clear_screen(); OLED_show_string(4, 20, last); read_screen();
    TEST_CHECK(memcmp(expected, screen, sizeof(expected)) == 0);
    uint8_t long_text[600]; memset(long_text, 'A', sizeof(long_text)); long_text[599] = 0;
    clear_screen(); OLED_show_string(0, 0, long_text); read_screen();
    memcpy(expected, screen, sizeof(expected));
    clear_screen();
    for (unsigned row = 0; row < 5u; ++row)
        for (unsigned col = 0; col < 21u; ++col) OLED_show_char((uint8_t)row, (uint8_t)col, 'A');
    read_screen(); TEST_CHECK(memcmp(expected, screen, sizeof(expected)) == 0);
    OLED_show_string(0, 0, (uint8_t *)""); read_screen();
    TEST_CHECK(memcmp(expected, screen, sizeof(expected)) == 0);
}

static void formatting(void)
{
    uint8_t expected[128][8];
    const char *text[] = {"", "A", "12345678901234567890", "123456789012345678901", "1234567890123456789012"};
    for (unsigned col = 0; col <= 20u; ++col)
        for (size_t i = 0; i < sizeof(text) / sizeof(text[0]); ++i)
        {
            char prefix[22];
            size_t length = strlen(text[i]), available = 21u - col;
            if (length > available) length = available;
            memcpy(prefix, text[i], length); prefix[length] = 0;
            clear_screen(); OLED_show_string(0, (uint8_t)col, (uint8_t *)prefix);
            read_screen(); memcpy(expected, screen, sizeof(expected));
            clear_screen(); OLED_printf(0, (uint8_t)col, "%s", text[i]);
            if (format_capacity != available + 1u)
                (void)fprintf(stderr, "format size col=%u text=%zu expected=%zu actual=%zu\n",
                              col, i, available + 1u, format_capacity);
            TEST_CHECK(format_capacity == available + 1u);
            read_screen(); TEST_CHECK(memcmp(expected, screen, sizeof(expected)) == 0);
        }
    clear_screen(); OLED_show_string(2, 1, (uint8_t *)"A -3 1.25");
    read_screen(); memcpy(expected, screen, sizeof(expected));
    clear_screen(); OLED_printf(2, 1, "%s %d %.2f", "A", -3, 1.25);
    read_screen(); TEST_CHECK(memcmp(expected, screen, sizeof(expected)) == 0);
    // 动态长输入验证运行时容量；先清屏，避免把未覆盖旧格误判为格式化残留。
    char long_text[2048]; memset(long_text, 'A', sizeof(long_text)); long_text[2047] = 0;
    clear_screen(); OLED_printf(0, 20, "%s", long_text);
    clear_screen(); OLED_printf(0, 0, "%s", "B"); read_screen();
    memcpy(expected, screen, sizeof(expected));
    clear_screen(); OLED_show_char(0, 0, 'B'); read_screen();
    TEST_CHECK(memcmp(expected, screen, sizeof(expected)) == 0);
}

int main(int argc, char **argv)
{
    TEST_CHECK(argc == 2);
    if (strcmp(argv[1], "lines") == 0) lines();
    else if (strcmp(argv[1], "invalid") == 0) invalid();
    else if (strcmp(argv[1], "characters") == 0) characters();
    else if (strcmp(argv[1], "strings") == 0) strings();
    else if (strcmp(argv[1], "formatting") == 0) formatting();
    else TEST_CHECK(0);
    return 0;
}
