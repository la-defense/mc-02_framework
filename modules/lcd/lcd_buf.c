#include "lcd_buf.h"
#include "lcd.h"
#include "lcdfont.h"
#include "spi.h"
#include "main.h"
#include "string.h"
#include "stdio.h"

#define LCD_BUF_MAX_W 280u

static uint16_t lcd_line_buf[LCD_BUF_MAX_W];

_Static_assert(LCD_PANEL_W <= LCD_BUF_MAX_W, "行缓冲必须能装下整屏一行像素");

/* 调试用: 统计 SPI 发送失败的次数(热插拔/总线异常时可以看出来) */
volatile uint32_t lcd_spi_errors = 0;

static void LCD_BufBegin(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2)
{
    LCD_Address_Set(x1, y1, x2, y2);
    LCD_DC_Set();
    LCD_CS_Clr();
}

static void LCD_BufWriteRow(uint16_t pixels)
{
    if (HAL_SPI_Transmit(&hspi1, (uint8_t *)lcd_line_buf, (uint16_t)(pixels * 2u), 20) != HAL_OK)
    {
        /* 失败后 HAL 句柄可能停在 BUSY, 中止掉, 免得后面所有发送都失败 */
        lcd_spi_errors++;
        HAL_SPI_Abort(&hspi1);
    }
}

static void LCD_BufEnd(void)
{
    LCD_CS_Set();
}

/* 直接按 ST7789 物理地址设置窗口: 参数就是 0x2a/0x2b 要写入的值, 不再做任何叠加。
   LCD_PANEL_X0/Y0/W/H 已经包含竖屏行轴(横屏列轴)的 20 偏移, 所以这里不能再加 20
   (之前多加了 20, 导致左边缘一条 20 像素宽的区域永远清不到)。 */
static void LCD_BufBeginPanel(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2)
{
    LCD_WR_REG(0x2a);
    LCD_WR_DATA(x1);
    LCD_WR_DATA(x2);
    LCD_WR_REG(0x2b);
    LCD_WR_DATA(y1);
    LCD_WR_DATA(y2);
    LCD_WR_REG(0x2c);
    LCD_DC_Set();
    LCD_CS_Clr();
}

/* 整屏(整块玻璃)刷黑: 擦掉上一版固件或更早画面留在内缩区之外的残留像素。
   内缩(LCD_MARGIN)使正常绘制区变小，如果不做这一步，旧像素会一直留在边框附近。 */
void LCD_BufClearPanel(void)
{
    for (uint16_t i = 0; i < (uint16_t)LCD_PANEL_W; ++i)
        lcd_line_buf[i] = BLACK;

    LCD_BufBeginPanel(LCD_PANEL_X0, LCD_PANEL_Y0,
                      (uint16_t)(LCD_PANEL_X0 + LCD_PANEL_W - 1u),
                      (uint16_t)(LCD_PANEL_Y0 + LCD_PANEL_H - 1u));
    for (uint16_t row = 0; row < (uint16_t)LCD_PANEL_H; ++row)
        LCD_BufWriteRow((uint16_t)LCD_PANEL_W);
    LCD_BufEnd();
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
