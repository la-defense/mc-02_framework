#include "lcd_buf.h"
#include "lcd.h"
#include "lcdfont.h"
#include "spi.h"
#include "main.h"
#include "string.h"
#include "stdio.h"

#define LCD_BUF_MAX_W 280u

static uint16_t lcd_line_buf[LCD_BUF_MAX_W];

static void LCD_BufBegin(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2)
{
    LCD_Address_Set(x1, y1, x2, y2);
    LCD_DC_Set();
    LCD_CS_Clr();
}

static void LCD_BufWriteRow(uint16_t pixels)
{
    HAL_SPI_Transmit(&hspi1, (uint8_t *)lcd_line_buf, (uint16_t)(pixels * 2u), 20);
}

static void LCD_BufEnd(void)
{
    LCD_CS_Set();
}

void LCD_BufFill(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color)
{
    if (x1 >= LCD_W || y1 >= LCD_H || x2 >= LCD_W || y2 >= LCD_H || x1 > x2 || y1 > y2)
        return;

    uint16_t width = (uint16_t)(x2 - x1 + 1u);
    for (uint16_t i = 0; i < width; ++i)
        lcd_line_buf[i] = color;

    LCD_BufBegin(x1, y1, x2, y2);
    for (uint16_t row = y1; row <= y2; ++row)
        LCD_BufWriteRow(width);
    LCD_BufEnd();
}

void LCD_BufShowAscii(uint16_t x, uint16_t y, const char *str, uint16_t fc, uint16_t bc, uint8_t sizey)
{
    if (str == NULL)
        return;

    const uint8_t *font;
    uint8_t sizex;
    if (sizey == 16)
    {
        font = (const uint8_t *)ascii_1608;
        sizex = 8;
    }
    else if (sizey == 12)
    {
        font = (const uint8_t *)ascii_1206;
        sizex = 6;
    }
    else
    {
        return;
    }

    uint16_t len = (uint16_t)strlen(str);
    uint16_t width = (uint16_t)(len * sizex);
    if (width == 0 || width > LCD_BUF_MAX_W || x + width > LCD_W || y + sizey > LCD_H)
        return;

    const uint8_t bytes_per_row = (uint8_t)((sizex + 7u) / 8u);
    const uint8_t bytes_per_char = (uint8_t)(bytes_per_row * sizey);
    LCD_BufBegin(x, y, (uint16_t)(x + width - 1u), (uint16_t)(y + sizey - 1u));

    for (uint8_t row = 0; row < sizey; ++row)
    {
        for (uint16_t ci = 0; ci < len; ++ci)
        {
            uint8_t ch = (uint8_t)str[ci];
            if (ch < 32u || ch > 126u)
                ch = (uint8_t)' ';
            const uint8_t *glyph = font + (uint32_t)(ch - 32u) * bytes_per_char;

            for (uint8_t col = 0; col < sizex; ++col)
            {
                /* The example ASCII fonts are row-major, LSB = leftmost pixel. */
                uint8_t byte_index = (uint8_t)(row * bytes_per_row + (col / 8u));
                uint8_t mask = (uint8_t)(1u << (col % 8u));
                lcd_line_buf[ci * sizex + col] = (glyph[byte_index] & mask) ? fc : bc;
            }
        }
        LCD_BufWriteRow(width);
    }
    LCD_BufEnd();
}

void LCD_BufShowCnString(uint16_t x, uint16_t y, const Lcd_CnChar_e *str, uint16_t fc, uint16_t bc)
{
    if (str == NULL)
        return;

    uint16_t len = 0;
    while (str[len] != LCD_CN_COUNT && len < 32u)
        len++;
    if (len == 0)
        return;

    uint16_t width = (uint16_t)(len * 16u);
    if (x + width > LCD_W || y + 16u > LCD_H)
        return;

    LCD_BufBegin(x, y, (uint16_t)(x + width - 1u), (uint16_t)(y + 15u));
    for (uint8_t row = 0; row < 16u; ++row)
    {
        for (uint16_t gi = 0; gi < len; ++gi)
        {
            const uint8_t *glyph = lcd_cn_font16[str[gi]];
            for (uint8_t col = 0; col < 16u; ++col)
            {
                uint8_t byte_index = (uint8_t)(col * 2u + (row / 8u));
                uint8_t mask = (uint8_t)(1u << (row % 8u));
                lcd_line_buf[gi * 16u + col] = (glyph[byte_index] & mask) ? fc : bc;
            }
        }
        LCD_BufWriteRow(width);
    }
    LCD_BufEnd();
}

void LCD_BufShowCnChar(uint16_t x, uint16_t y, Lcd_CnChar_e ch, uint16_t fc, uint16_t bc)
{
    Lcd_CnChar_e str[2] = {ch, LCD_CN_COUNT};
    LCD_BufShowCnString(x, y, str, fc, bc);
}

void LCD_BufShowInt(uint16_t x, uint16_t y, int32_t value, uint16_t fc, uint16_t bc, uint8_t sizey)
{
    char buf[16];
    snprintf(buf, sizeof(buf), "%ld", (long)value);
    LCD_BufShowAscii(x, y, buf, fc, bc, sizey);
}

void LCD_BufShowFloat(uint16_t x, uint16_t y, float value, uint8_t decimals, uint16_t fc, uint16_t bc, uint8_t sizey)
{
    if (decimals > 3u)
        decimals = 3u;

    uint32_t scale = 1u;
    for (uint8_t i = 0; i < decimals; ++i)
        scale *= 10u;

    int32_t scaled = (int32_t)(value * (float)scale + (value >= 0.0f ? 0.5f : -0.5f));
    uint8_t negative = (scaled < 0) ? 1u : 0u;
    if (negative)
        scaled = -scaled;

    uint32_t integer = (uint32_t)(scaled / (int32_t)scale);
    uint32_t fraction = (uint32_t)(scaled % (int32_t)scale);

    char buf[24];
    char *p = buf;
    if (negative)
        *p++ = '-';

    char intbuf[12];
    snprintf(intbuf, sizeof(intbuf), "%lu", (unsigned long)integer);
    for (char *q = intbuf; *q; ++q)
        *p++ = *q;

    if (decimals > 0u)
    {
        *p++ = '.';
        for (uint8_t i = 0; i < decimals; ++i)
        {
            uint32_t div = 1u;
            for (uint8_t j = 0; j < (uint8_t)(decimals - 1u - i); ++j)
                div *= 10u;
            *p++ = (char)('0' + (fraction / div) % 10u);
        }
    }
    *p = '\0';

    LCD_BufShowAscii(x, y, buf, fc, bc, sizey);
}
