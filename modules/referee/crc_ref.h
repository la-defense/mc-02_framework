/**
* @file crc_ref.h
* @brief 声明裁判系统帧使用的 CRC-8 和 CRC-16 校验接口。
* Append 接口把校验字节写入缓冲区；Verify 接口只检查，不修改数据。
*/
#ifndef __CRC_H_
#define __CRC_H_

/* 这组查表算法与 modules/algorithm 中的 CRC 接口分开使用。 */

#include <stdint.h>

#define TRUE 1
#define FALSE 0
/* CRC-8 */
/**
* @brief 计算并写入 CRC-8。
* @param pchMessage 可写帧缓冲区；校验值写入最后1个字节。
* @param dwLength 缓冲区总字节数，至少为3，且包含校验字节。
* @note 空指针或长度不大于2时直接返回。
*/
void Append_CRC8_Check_Sum(uint8_t *pchMessage, uint16_t dwLength);
/**
* @brief 检查缓冲区末尾的 CRC-8。
* @param pchMessage 至少包含 dwLength 个字节。
* @param dwLength 帧总字节数，至少为3，且包含校验字节。
* @return 校验正确返回1；空指针、长度不合要求或校验不符时返回0。
*/
uint32_t Verify_CRC8_Check_Sum(uint8_t *pchMessage, uint16_t dwLength);
/**
* @brief 根据给定初值计算一段数据的 CRC-8。
* @param pchMessage 至少可读 dwLength 个字节；长度大于0时不能为 NULL。
* @param dwLength 参与计算的字节数。
* @param ucCRC8 计算开始前的 CRC 值。
* @return 计算后的8位 CRC。
*/
uint8_t Get_CRC8_Check_Sum(uint8_t *pchMessage, uint16_t dwLength, uint8_t ucCRC8);

/* CRC-16 */
/**
* @brief 计算并写入 CRC-16。
* @param pchMessage 可写帧缓冲区；校验值按低字节、高字节写入末尾2字节。
* @param dwLength 缓冲区总字节数，至少为3，且包含校验字节。
* @note 空指针或长度不大于2时直接返回。
*/
void Append_CRC16_Check_Sum(uint8_t *pchMessage, uint32_t dwLength);
/**
* @brief 检查缓冲区末尾的 CRC-16。
* @param pchMessage 至少包含 dwLength 个字节。
* @param dwLength 帧总字节数，至少为3，且包含校验字节。
* @return 校验正确返回1；空指针、长度不合要求或校验不符时返回0。
*/
uint32_t Verify_CRC16_Check_Sum(uint8_t *pchMessage, uint32_t dwLength);
/**
* @brief 从给定 CRC 值继续计算 CRC-16。
* @param pchMessage 至少可读 dwLength 个字节。
* @param dwLength 参与计算的字节数。
* @param wCRC 计算开始前的 CRC 值。
* @return 计算后的16位 CRC；pchMessage 为空时返回0xFFFF。
*/
uint16_t Get_CRC16_Check_Sum(uint8_t *pchMessage, uint32_t dwLength, uint16_t wCRC);

#endif
