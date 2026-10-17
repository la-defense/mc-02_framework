/**
* @file crc16.c
* @brief 用查表方式计算 CRC-16；首次调用时生成查找表。
 */
#include "crc16.h"

static uint8_t crc_tab16_init = 0;
static uint16_t crc_tab16[256];

/**
* @brief 计算一段数据的 CRC-16。空指针时保留起始值。
 */
uint16_t crc_16(const uint8_t *input_str, uint16_t num_bytes)
{
    uint16_t crc;
    const uint8_t *ptr;
    uint16_t a;
    // 第一次计算时先生成查找表，后续调用直接复用。
    if (!crc_tab16_init)
        init_crc16_tab();
    crc = CRC_START_16;
    ptr = input_str;
    if (ptr != NULL)
        for (a = 0; a < num_bytes; a++)
        {
            crc = (crc >> 8) ^ crc_tab16[(crc ^ (uint16_t)*ptr++) & 0x00FF];
        }
    return crc;
}

/**
* @brief 计算一段数据的 Modbus CRC。算法使用本文件定义的 CRC-16 查找表。
 */

uint16_t crc_modbus(const uint8_t *input_str, uint16_t num_bytes)
{
    uint16_t crc;
    const uint8_t *ptr;
    uint16_t a;

    // 第一次计算时先生成查找表，后续调用直接复用。
    if (!crc_tab16_init)
        init_crc16_tab();

    crc = CRC_START_MODBUS;
    ptr = input_str;
    if (ptr != NULL)
        for (a = 0; a < num_bytes; a++)
        {

            crc = (crc >> 8) ^ crc_tab16[(crc ^ (uint16_t)*ptr++) & 0x00FF];
        }
    return crc;
}

/** @brief 用一个字节继续更新传入的 CRC 值。 */
uint16_t update_crc_16(uint16_t crc, uint8_t c)
{
    // 第一次计算时先生成查找表，后续调用直接复用。
    if (!crc_tab16_init)
        init_crc16_tab();
    return (crc >> 8) ^ crc_tab16[(crc ^ (uint16_t)c) & 0x00FF];
}

/**
* @brief 按多项式逐项生成256个表值，供后续字节计算查表。
 */
void init_crc16_tab(void)
{
    uint16_t i;
    uint16_t j;
    uint16_t crc;
    uint16_t c;
    // 每个可能的字节值各生成一个表项。
    for (i = 0; i < 256; i++)
    {
        crc = 0;
        c = i;
        // 按位处理一个字节，逐次移位并按多项式修正 CRC。
        for (j = 0; j < 8; j++)
        {
            if ((crc ^ c) & 0x0001)
                crc = (crc >> 1) ^ CRC_POLY_16;
            else
                crc = crc >> 1;
            c = c >> 1;
        }
        crc_tab16[i] = crc;
    }
    crc_tab16_init = 1;
}
