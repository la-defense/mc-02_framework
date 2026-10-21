/**
* @file oled.c
* @brief 维护 OLED 显存缓冲区，并通过 I2C 向面板发送指令和像素数据。
* 绘图函数只改缓冲区；OLED_refresh_gram 按8页、每页128列发送整屏。
* 所有入口须由同一调用者串行使用；当前没有接入业务，不保证面板通信成功。
 */
#include "oled.h"
#include "oledfont.h"
#include "main.h"
#include <stdio.h>
#include <stdarg.h>

extern I2C_HandleTypeDef hi2c2;
static uint8_t OLED_GRAM[128][8];

// 先检查画笔，避免非法枚举被当作清除或反转操作。
static int valid_pen(pen_typedef pen)
{
    // 先转成无符号整数，调用方传入枚举外数值时也能明确拒绝。
    unsigned int value = (unsigned int)pen;
    return value <= (unsigned int)PEN_INVERSION;
}

// 字符网格为5行、每行21格，确认后再计算字模起点。
static int valid_character_position(uint8_t row, uint8_t col)
{
    return row < 5u && col < 21u;
}

// 只检查本次能够显示的部分，屏幕末端之后不再读取字符串。
static int visible_text_length(const uint8_t *text, size_t capacity, size_t *length)
{
    if (text == NULL) return 0;
    size_t count = 0u;
    while (count < capacity && text[count] != '\0')
    {
        if (text[count] < 0x20u || text[count] > 0x7eu) return 0;
        ++count;
    }
    *length = count;
    return 1;
}

static void oled_write_byte(uint8_t dat, uint8_t cmd)
{
    static uint8_t cmd_data[2];
    // 控制字节区分指令与显存数据，第二字节才是实际发送内容。
    if(cmd == OLED_CMD)
    {
        cmd_data[0] = 0x00;
    }
    else
    {
        cmd_data[0] = 0x40;
    }
    cmd_data[1] = dat;
    HAL_I2C_Master_Transmit(&hi2c2, OLED_I2C_ADDRESS, cmd_data, 2, 10);
}



void OLED_init(void)
{
    oled_write_byte(0xAE, OLED_CMD);    //display off
    oled_write_byte(0x20, OLED_CMD);    //Set Memory Addressing Mode	
    oled_write_byte(0x10, OLED_CMD);    //00,Horizontal Addressing Mode;01,Vertical Addressing Mode;10,Page Addressing Mode (RESET);11,Invalid
    oled_write_byte(0xb0, OLED_CMD);    //Set Page Start Address for Page Addressing Mode,0-7
    oled_write_byte(0xc8, OLED_CMD);    //Set COM Output Scan Direction
    oled_write_byte(0x00, OLED_CMD);    //---set low column address
    oled_write_byte(0x10, OLED_CMD);    //---set high column address
    oled_write_byte(0x40, OLED_CMD);    //--set start line address
    oled_write_byte(0x81, OLED_CMD);    //--set contrast control register
    oled_write_byte(0xff, OLED_CMD);    //brightness 0x00~0xff
    oled_write_byte(0xa1, OLED_CMD);    //--set segment re-map 0 to 127
    oled_write_byte(0xa6, OLED_CMD);    //--set normal display
    oled_write_byte(0xa8, OLED_CMD);    //--set multiplex ratio(1 to 64)
    oled_write_byte(0x3F, OLED_CMD);    //
    oled_write_byte(0xa4, OLED_CMD);    //0xa4,Output follows RAM content;0xa5,Output ignores RAM content
    oled_write_byte(0xd3, OLED_CMD);    //-set display offset
    oled_write_byte(0x00, OLED_CMD);    //-not offset
    oled_write_byte(0xd5, OLED_CMD);    //--set display clock divide ratio/oscillator frequency
    oled_write_byte(0xf0, OLED_CMD);    //--set divide ratio
    oled_write_byte(0xd9, OLED_CMD);    //--set pre-charge period
    oled_write_byte(0x22, OLED_CMD);    //
    oled_write_byte(0xda, OLED_CMD);    //--set com pins hardware configuration
    oled_write_byte(0x12, OLED_CMD);
    oled_write_byte(0xdb, OLED_CMD);    //--set vcomh
    oled_write_byte(0x20, OLED_CMD);    //0x20,0.77xVcc
    oled_write_byte(0x8d, OLED_CMD);    //--set DC-DC enable
    oled_write_byte(0x14, OLED_CMD);    //
    oled_write_byte(0xaf, OLED_CMD);    //--turn on oled panel
}



void OLED_display_on(void)
{
    oled_write_byte(0x8d, OLED_CMD);
    oled_write_byte(0x14, OLED_CMD);
    oled_write_byte(0xaf, OLED_CMD);
}



void OLED_display_off(void)
{
    oled_write_byte(0x8d, OLED_CMD);
    oled_write_byte(0x10, OLED_CMD);
    oled_write_byte(0xae, OLED_CMD);
}



void OLED_operate_gram(pen_typedef pen)
{
    uint8_t i, n;

    if (!valid_pen(pen)) return;

    // 外层遍历8页，内层遍历每页128列。
    for (i = 0; i < 8; i++)
    {
        for (n = 0; n < 128; n++)
        {
            if (pen == PEN_WRITE)
            {
                OLED_GRAM[n][i] = 0xff;
            }
            else if (pen == PEN_CLEAR)
            {
                OLED_GRAM[n][i] = 0x00;
            }
            else
            {
                // 已检查画笔，剩下的操作为逐位反转。
                OLED_GRAM[n][i] = 0xff - OLED_GRAM[n][i];
            }
        }
    }
}


void OLED_set_pos(uint8_t x, uint8_t y)
{
    // 第二个参数是页号，不是像素行；错误地址不向面板提交。
    if (x >= X_WIDTH || y >= Y_WIDTH / 8u) return;
    // 先设页，再把列地址拆成高4位和低4位发送。
    oled_write_byte((0xb0 + y), OLED_CMD);              //set page address y
    oled_write_byte(((x&0xf0)>>4)|0x10, OLED_CMD);      //set column high address
    oled_write_byte((x&0x0f), OLED_CMD);                //set column low address
}


void OLED_draw_point(int8_t x, int8_t y, pen_typedef pen)
{
    uint8_t page = 0, row = 0;

    // 越界坐标不改写缓冲区。
    // int8_t的正上限就是127，不需要再次比较x是否大于127。
    if (x < 0 || y < 0 || y >= Y_WIDTH || !valid_pen(pen))
    {
        return;
    }
    // 每个字节竖着保存8个像素，page 选字节，row 选字节中的那一位。
    page = y / 8;
    row = y % 8;

    if (pen == PEN_WRITE)
    {
        OLED_GRAM[x][page] |= 1 << row;
    }
    else if (pen == PEN_INVERSION)
    {
        OLED_GRAM[x][page] ^= 1 << row;
    }
    else
    {
        OLED_GRAM[x][page] &= ~(1 << row);
    }
}



  
void OLED_draw_line(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, pen_typedef pen)
{
    // 整条线先检查，避免画了一部分才发现端点越界。
    if (x1 >= X_WIDTH || x2 >= X_WIDTH || y1 >= Y_WIDTH || y2 >= Y_WIDTH || !valid_pen(pen)) return;
    // 按列、再按行排序，使交换端点后的半像素选择也完全相同。
    if (x1 > x2 || (x1 == x2 && y1 > y2))
    {
        uint8_t temporary = x1; x1 = x2; x2 = temporary;
        temporary = y1; y1 = y2; y2 = temporary;
    }
    int x = x1, y = y1;
    int dx = (int)x2 - x1;
    int dy = y2 >= y1 ? (int)y1 - y2 : (int)y2 - y1;
    int step_y = y1 < y2 ? 1 : -1;
    int error = dx + dy;
    // 每步至少推进一个坐标，最多写128个点；含起点与终点，无浮点除法。
    for (;;)
    {
        OLED_draw_point((int8_t)x, (int8_t)y, pen);
        if (x == x2 && y == y2) break;
        int twice_error = 2 * error;
        if (twice_error >= dy) { error += dy; ++x; }
        if (twice_error <= dx) { error += dx; y += step_y; }
    }
}


void OLED_show_char(uint8_t row, uint8_t col, uint8_t chr)
{
    // 必须在相乘、减去空格之前检查，防止坐标回绕或索引越过字库。
    if (!valid_character_position(row, col) || chr < 0x20u || chr > 0x7eu) return;
    uint8_t x = col * 6;
    uint8_t y = row * 12;
    uint8_t temp, t, t1;
    uint8_t y0 = y;
    chr = chr - ' ';

    // 每个字形占6列、12行；两个字节依次提供一列的12个像素。
    for (t = 0; t < 12; t++)
    {
        temp = asc2_1206[chr][t];

        for (t1 = 0; t1 < 8; t1++)
        {
            if (temp&0x80)
                OLED_draw_point(x, y, PEN_WRITE);
            else
                OLED_draw_point(x, y, PEN_CLEAR);

            temp <<= 1;
            y++;
            if ((y - y0) == 12)
            {
                // 这一列绘制满12个像素后回到起始行，再画下一列。
                y = y0;
                x++;
                break;
            }
        }
    }
}


void OLED_show_string(uint8_t row, uint8_t col, uint8_t *chr)
{
    if (!valid_character_position(row, col)) return;
    size_t capacity = (5u - row) * 21u - col;
    size_t length;
    // 先检查所有待显示字符，失败时不留下已经画出的半段文字。
    if (!visible_text_length(chr, capacity, &length)) return;
    for (size_t n = 0u; n < length; ++n)
    {
        OLED_show_char(row, col, chr[n]);
        col++;

        if (col > 20)
        {
            col = 0;
            row += 1;
        }
    }
}



void OLED_printf(uint8_t row, uint8_t col, const char *fmt,...)
{
    if (!valid_character_position(row, col) || fmt == NULL) return;
    char buffer[22];
    va_list ap;
    size_t available = 21u - col;
    va_start(ap, fmt);
    // 多留一个字节保存结束标记；格式化超长时只保留本行放得下的前缀。
    int result = vsnprintf(buffer, available + 1u, fmt, ap);
    va_end(ap);
    if (result < 0) return;
    OLED_show_string(row, col, (uint8_t *)buffer);
}


void OLED_refresh_gram(void)
{
    uint8_t i, n;

    // 逐页设置起点，再连续发送该页128列的字节。
    for (i = 0; i < 8; i++)
    {
        OLED_set_pos(0, i);
        for (n = 0; n < 128; n++)
        {
            oled_write_byte(OLED_GRAM[n][i], OLED_DATA);
        }
    }
}




void OLED_LOGO(void)
{
    uint8_t temp_char = 0;
    uint8_t x = 0, y = 0;
    uint8_t i = 0;
    OLED_operate_gram(PEN_CLEAR);


    // 字库按页保存；逐页、逐列读取，再按位画出8个竖直像素。
    for(; y < 64; y += 8)
    {
        for(x = 0; x < 128; x++)
        {
            temp_char = LOGO_BMP[x][y/8];
            for(i = 0; i < 8; i++)
            {
                if(temp_char & 0x80)
                {
                    OLED_draw_point(x, y + i,PEN_WRITE);
                }
                else
                {
                    OLED_draw_point(x,y + i,PEN_CLEAR);
                }
                temp_char <<= 1;
            }
        }
    }
    OLED_refresh_gram();
}
