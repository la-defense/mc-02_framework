/**
* @file crc16.h
* @brief 声明 CRC-16 和 Modbus CRC 计算接口。
*/
#ifndef __CRC16_H
#define __CRC16_H
#include "main.h"

/** @brief 两种 CRC 计算使用的起始值和查表多项式。 */
#define CRC_START_16 0xFFFF
#define CRC_START_MODBUS 0xFFFF
#define CRC_POLY_16 0xA001

/**
* @brief 计算一段数据的 CRC-16。
* @param input_str 数据地址；可以为 NULL，此时返回起始值。
* @param num_bytes 要处理的字节数。
* @return 计算得到的16位 CRC；长度为0时返回起始值。
*/
uint16_t crc_16(const uint8_t *input_str, uint16_t num_bytes);
/**
* @brief 计算一段数据的 Modbus CRC。
* @param input_str 数据地址；可以为 NULL，此时返回起始值。
* @param num_bytes 要处理的字节数。
* @return 计算得到的16位 CRC；长度为0时返回起始值。
*/
uint16_t crc_modbus(const uint8_t *input_str, uint16_t num_bytes);
/**
* @brief 把一个新字节并入已有 CRC 结果。
* @param crc 前面数据算出的 CRC。
* @param c 要加入的字节。
* @return 加入该字节后的 CRC。
*/
uint16_t update_crc_16(uint16_t crc, uint8_t c);
/** @brief 生成256项 CRC 查表；通常由计算接口首次调用时自动执行。 */
void init_crc16_tab(void);

#endif
