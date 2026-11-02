/**
 * @file lcd_buf.c
 * @brief 用固定单行 RGB565 缓冲准备绘图数据，并检查尺寸后交给 LCD SPI 实现发送。
 * @note 绘图入口由单显示任务顺序调用；失败可能已经写出前面的完整行。
 */
#include "lcd_buf.h"

#include "lcd_internal.h"
#include "lcdfont.h"
#include "../../bsp/dwt/platform_probe.h"

#include <limits.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

#define LCD_BUF_MAX_W 280u
#define LCD_DRAW_BUDGET_MS 250u
#define LCD_TEXT_BUDGET_MS 20u

static uint16_t lcd_line_buf[LCD_BUF_MAX_W];

_Static_assert(LCD_PANEL_W <= LCD_BUF_MAX_W, "行缓冲必须能装下整屏一行像素");

volatile uint32_t lcd_spi_bytes = 0u;
volatile uint32_t lcd_prof_row_us = 0u;
volatile uint16_t lcd_prof_row_bytes = 0u;
volatile uint32_t lcd_prof_spi_us = 0u;
volatile uint32_t lcd_prof_rows = 0u;
volatile uint32_t lcd_prof_clear_us = 0u;

static uint32_t lcd_sat_add(uint32_t value, uint32_t amount)
{
    return (UINT32_MAX - value < amount) ? UINT32_MAX : value + amount;
}

static void lcd_profile_row(uint32_t started, uint16_t bytes, LCDStatus_t status)
{
    uint32_t elapsed = PlatformProbeElapsedUs(started);
    lcd_prof_row_us = elapsed;
    lcd_prof_row_bytes = bytes;
    lcd_prof_spi_us = lcd_sat_add(lcd_prof_spi_us, elapsed);
    lcd_prof_rows = lcd_sat_add(lcd_prof_rows, 1u);
    if (status == LCD_OK)
        lcd_spi_bytes = lcd_sat_add(lcd_spi_bytes, bytes);
}

static LCDStatus_t lcd_set_panel_address(void)
{
    LCDStatus_t status = LCD_InternalWriteCommand(0x2Au);
    if (status == LCD_OK) status = LCD_InternalWriteData8((uint8_t)(LCD_PANEL_X0 >> 8));
    if (status == LCD_OK) status = LCD_InternalWriteData8((uint8_t)LCD_PANEL_X0);
    if (status == LCD_OK) status = LCD_InternalWriteData8((uint8_t)((LCD_PANEL_X0 + LCD_PANEL_W - 1) >> 8));
    if (status == LCD_OK) status = LCD_InternalWriteData8((uint8_t)(LCD_PANEL_X0 + LCD_PANEL_W - 1));
    if (status == LCD_OK) status = LCD_InternalWriteCommand(0x2Bu);
    if (status == LCD_OK) status = LCD_InternalWriteData8((uint8_t)(LCD_PANEL_Y0 >> 8));
    if (status == LCD_OK) status = LCD_InternalWriteData8((uint8_t)LCD_PANEL_Y0);
    if (status == LCD_OK) status = LCD_InternalWriteData8((uint8_t)((LCD_PANEL_Y0 + LCD_PANEL_H - 1) >> 8));
    if (status == LCD_OK) status = LCD_InternalWriteData8((uint8_t)(LCD_PANEL_Y0 + LCD_PANEL_H - 1));
    if (status == LCD_OK) status = LCD_InternalWriteCommand(0x2Cu);
    return status;
}

LCDStatus_t LCD_BufClearPanel(void)
{
    uint32_t started = PlatformProbeStart();
    LCDStatus_t status = LCD_InternalBegin(LCD_DRAW_BUDGET_MS, 0u);
    if (status != LCD_OK)
        return status;

    LCDState_t panel_state;
    status = LCDGetState(&panel_state);
    if (status == LCD_OK && !panel_state.panel_initialized)
        status = LCD_NOT_INITIALIZED;
    if (status != LCD_OK)
        return LCD_InternalEnd(status);

    // 同一条黑色行缓冲重复发送，避免为整屏清理分配帧缓冲。
    for (uint16_t i = 0u; i < LCD_PANEL_W; ++i)
        lcd_line_buf[i] = BLACK;

    status = lcd_set_panel_address();
    if (status == LCD_OK)
        status = LCD_InternalBeginPixelData();

    uint16_t row_bytes = (uint16_t)(LCD_PANEL_W * sizeof(lcd_line_buf[0]));
    // 任一行发送失败就停止，后面的像素不能在错误状态下继续写。
    for (uint16_t row = 0u; status == LCD_OK && row < LCD_PANEL_H; ++row)
    {
        uint32_t row_started = PlatformProbeStart();
        status = LCD_InternalWritePixelData((const uint8_t *)lcd_line_buf, row_bytes);
        lcd_profile_row(row_started, row_bytes, status);
    }
    LCD_InternalEndPixelData();
    lcd_prof_clear_us = PlatformProbeElapsedUs(started);
    return LCD_InternalEnd(status);
}

static LCDStatus_t lcd_fill(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color)
{
    uint16_t width = (uint16_t)(x2 - x1 + 1u);
    uint16_t row_bytes = (uint16_t)(width * sizeof(lcd_line_buf[0]));
    // 先把矩形的一行填好，再循环发送到包含 y1 和 y2 的所有行。
    for (uint16_t col = 0u; col < width; ++col)
        lcd_line_buf[col] = color;

    LCDStatus_t status = LCD_InternalSetAddress(x1, y1, x2, y2);
    if (status == LCD_OK)
        status = LCD_InternalBeginPixelData();
    for (uint16_t row = y1; status == LCD_OK && row <= y2; ++row)
    {
        uint32_t row_started = PlatformProbeStart();
        status = LCD_InternalWritePixelData((const uint8_t *)lcd_line_buf, row_bytes);
        lcd_profile_row(row_started, row_bytes, status);
    }
    LCD_InternalEndPixelData();
    return status;
}

LCDStatus_t LCD_BufFill(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color)
{
    LCDStatus_t status = LCD_InternalBegin(LCD_DRAW_BUDGET_MS, 0u);
    if (status != LCD_OK)
        return status;
    if (x1 >= LCD_W || x2 >= LCD_W || y1 >= LCD_H || y2 >= LCD_H || x1 > x2 || y1 > y2)
        status = LCD_OUT_OF_RANGE;
    else
        status = lcd_fill(x1, y1, x2, y2, color);
    return LCD_InternalEnd(status);
}

static uint8_t lcd_ascii_font(uint8_t sizey, const uint8_t **font, uint8_t *width)
{
    if (sizey == 16u)
    {
        *font = (const uint8_t *)ascii_1608;
        *width = 8u;
        return 1u;
    }
    if (sizey == 12u)
    {
        *font = (const uint8_t *)ascii_1206;
        *width = 6u;
        return 1u;
    }
    return 0u;
}

LCDStatus_t LCD_BufShowAscii(uint16_t x, uint16_t y, const char *str, size_t length,
                             uint16_t fc, uint16_t bc, uint8_t sizey)
{
    LCDStatus_t status = LCD_InternalBegin(LCD_TEXT_BUDGET_MS, 0u);
    if (status != LCD_OK)
        return status;

    const uint8_t *font = NULL;
    uint8_t char_width = 0u;
    if (!lcd_ascii_font(sizey, &font, &char_width))
        status = LCD_OUT_OF_RANGE;
    else if (length != 0u && str == NULL)
        status = LCD_BAD_ARGUMENT;
    else if (length > LCD_BUF_MAX_W / char_width)
        status = LCD_OUT_OF_RANGE;
    else if (length != 0u && (length * char_width > (size_t)LCD_W || (size_t)x >= (size_t)LCD_W ||
                              (size_t)y >= (size_t)LCD_H ||
                              (size_t)x > (size_t)LCD_W - length * char_width ||
                              (size_t)sizey > (size_t)LCD_H - y))
        status = LCD_OUT_OF_RANGE;

    // 先检查全部字节，非法字符不会留下已经画出的前缀。
    for (size_t i = 0u; status == LCD_OK && i < length; ++i)
    {
        uint8_t ch = (uint8_t)str[i];
        if (ch < 0x20u || ch > 0x7Eu)
            status = LCD_BAD_TEXT;
    }

    if (status != LCD_OK || length == 0u)
        return LCD_InternalEnd(status);

    uint16_t width = (uint16_t)(length * char_width);
    uint8_t bytes_per_row = (uint8_t)((char_width + 7u) / 8u);
    uint8_t bytes_per_char = (uint8_t)(bytes_per_row * sizey);
    status = LCD_InternalSetAddress(x, y, (uint16_t)(x + width - 1u),
                                    (uint16_t)(y + sizey - 1u));
    if (status == LCD_OK)
        status = LCD_InternalBeginPixelData();

    // 每次先拼完整一行所有字符的像素，发送成功后再处理下一行。
    for (uint8_t row = 0u; status == LCD_OK && row < sizey; ++row)
    {
        for (size_t ci = 0u; ci < length; ++ci)
        {
            uint8_t ch = (uint8_t)str[ci];
            const uint8_t *glyph = font + (size_t)(ch - 0x20u) * bytes_per_char;
            for (uint8_t col = 0u; col < char_width; ++col)
            {
                uint8_t byte_index = (uint8_t)(row * bytes_per_row + col / 8u);
                uint8_t mask = (uint8_t)(1u << (col % 8u));
                lcd_line_buf[ci * char_width + col] = (glyph[byte_index] & mask) ? fc : bc;
            }
        }
        uint32_t row_started = PlatformProbeStart();
        uint16_t row_bytes = (uint16_t)(width * sizeof(lcd_line_buf[0]));
        status = LCD_InternalWritePixelData((const uint8_t *)lcd_line_buf, row_bytes);
        lcd_profile_row(row_started, row_bytes, status);
    }
    LCD_InternalEndPixelData();
    return LCD_InternalEnd(status);
}

LCDStatus_t LCD_BufShowCnString(uint16_t x, uint16_t y, const Lcd_CnChar_e *glyphs, size_t count,
                                uint16_t fc, uint16_t bc)
{
    LCDStatus_t status = LCD_InternalBegin(LCD_TEXT_BUDGET_MS, 0u);
    if (status != LCD_OK)
        return status;

    if (count != 0u && glyphs == NULL)
        status = LCD_BAD_ARGUMENT;
    else if (count > LCD_BUF_MAX_W / 16u)
        status = LCD_OUT_OF_RANGE;
    else if (count != 0u && (count * 16u > (size_t)LCD_W || (size_t)x >= (size_t)LCD_W ||
                             (size_t)y >= (size_t)LCD_H ||
                             (size_t)x > (size_t)LCD_W - count * 16u ||
                             16u > (size_t)LCD_H - y))
        status = LCD_OUT_OF_RANGE;

    for (size_t i = 0u; status == LCD_OK && i < count; ++i)
    {
        if ((unsigned)glyphs[i] >= (unsigned)LCD_CN_COUNT)
            status = LCD_BAD_GLYPH;
    }

    if (status != LCD_OK || count == 0u)
        return LCD_InternalEnd(status);

    uint16_t width = (uint16_t)(count * 16u);
    status = LCD_InternalSetAddress(x, y, (uint16_t)(x + width - 1u), (uint16_t)(y + 15u));
    if (status == LCD_OK)
        status = LCD_InternalBeginPixelData();
    // 中文字模按列存储，逐个像素组合出16行后写入同一块窗口。
    for (uint8_t row = 0u; status == LCD_OK && row < 16u; ++row)
    {
        for (size_t gi = 0u; gi < count; ++gi)
        {
            const uint8_t *glyph = lcd_cn_font16[glyphs[gi]];
            for (uint8_t col = 0u; col < 16u; ++col)
            {
                uint8_t byte_index = (uint8_t)(col * 2u + row / 8u);
                uint8_t mask = (uint8_t)(1u << (row % 8u));
                lcd_line_buf[gi * 16u + col] = (glyph[byte_index] & mask) ? fc : bc;
            }
        }
        uint32_t row_started = PlatformProbeStart();
        uint16_t row_bytes = (uint16_t)(width * sizeof(lcd_line_buf[0]));
        status = LCD_InternalWritePixelData((const uint8_t *)lcd_line_buf, row_bytes);
        lcd_profile_row(row_started, row_bytes, status);
    }
    LCD_InternalEndPixelData();
    return LCD_InternalEnd(status);
}

LCDStatus_t LCD_BufShowCnChar(uint16_t x, uint16_t y, Lcd_CnChar_e ch, uint16_t fc, uint16_t bc)
{
    return LCD_BufShowCnString(x, y, &ch, 1u, fc, bc);
}

LCDStatus_t LCD_BufShowInt(uint16_t x, uint16_t y, int32_t value,
                           uint16_t fc, uint16_t bc, uint8_t sizey)
{
    char text[16];
    int written = snprintf(text, sizeof(text), "%ld", (long)value);
    if (written < 0 || (size_t)written >= sizeof(text))
        return LCD_BAD_TEXT;
    return LCD_BufShowAscii(x, y, text, (size_t)written, fc, bc, sizey);
}

LCDStatus_t LCD_BufFormatFloat(float value, uint8_t decimals, char *output,
                               size_t capacity, size_t *length)
{
    if (output == NULL || length == NULL)
        return LCD_BAD_ARGUMENT;
    if (!isfinite(value))
        return LCD_NONFINITE;
    if (decimals > 3u)
        return LCD_OUT_OF_RANGE;

    uint32_t scale = 1u;
    // 小数位最多三位，逐次扩大十进制比例。
    for (uint8_t i = 0u; i < decimals; ++i)
        scale *= 10u;

    double rounded = round((double)value * (double)scale);
    if (!isfinite(rounded))
        return LCD_OUT_OF_RANGE;
    if (rounded < (double)INT32_MIN || rounded > (double)INT32_MAX)
        return LCD_OUT_OF_RANGE;

    int32_t scaled = (int32_t)rounded;
    uint8_t negative = (uint8_t)(scaled < 0);
    uint32_t magnitude = (scaled < 0) ? (uint32_t)(-(int64_t)scaled) : (uint32_t)scaled;
    uint32_t integer = magnitude / scale;
    uint32_t fraction = magnitude % scale;
    char candidate[16];
    size_t used = 0u;

    if (negative)
        candidate[used++] = '-';
    int count = snprintf(candidate + used, sizeof(candidate) - used, "%lu", (unsigned long)integer);
    if (count < 0 || (size_t)count >= sizeof(candidate) - used)
        return LCD_BAD_TEXT;
    used += (size_t)count;
    if (decimals != 0u)
    {
        candidate[used++] = '.';
        uint32_t divisor = scale / 10u;
        for (uint8_t i = 0u; i < decimals; ++i)
        {
            candidate[used++] = (char)('0' + (fraction / divisor) % 10u);
            divisor /= 10u;
        }
    }
    candidate[used] = '\0';
    if (capacity <= used)
        return LCD_OUT_OF_RANGE;

    memcpy(output, candidate, used + 1u);
    *length = used;
    return LCD_OK;
}

LCDStatus_t LCD_BufShowFloat(uint16_t x, uint16_t y, float value, uint8_t decimals,
                             uint16_t fc, uint16_t bc, uint8_t sizey)
{
    char text[16];
    size_t length = 0u;
    LCDStatus_t status = LCD_BufFormatFloat(value, decimals, text, sizeof(text), &length);
    if (status != LCD_OK)
        return status;
    return LCD_BufShowAscii(x, y, text, length, fc, bc, sizey);
}
