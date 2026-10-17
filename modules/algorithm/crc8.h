/**
* @file crc8.h
* @brief 声明 CRC-8 查表计算接口。
*/
#ifndef __CRC8_H
#define __CRC8_H

#include "main.h"

/** @brief CRC-8 每段新数据开始计算时使用的初始值。 */
#define CRC_START_8 0x00

/**
* @brief 计算一段数据的 CRC-8。
* @param input_str 数据地址；可以为 NULL，此时返回初始值。
* @param num_bytes 要处理的字节数。
* @return 计算得到的8位 CRC；长度为0时返回初始值。
*/
uint8_t crc_8(const uint8_t *input_str, uint16_t num_bytes);
/**
* @brief 把一个新字节并入已有 CRC 结果。
* @param crc 前面数据算出的 CRC。
* @param val 要加入的字节。
* @return 加入该字节后的 CRC。
*/
uint8_t update_crc_8(uint8_t crc, uint8_t val);

#endif
