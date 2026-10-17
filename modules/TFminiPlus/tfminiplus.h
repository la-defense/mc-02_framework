/** @file tfminiplus.h
* @brief 保留的 TFmini I²C 测距入口，距离和强度只保存原始整数，未做单位换算。
* 当前读取请求 11 字节、成员数组却仅 9 字节，接收长度尚待修正，不能直接作为已完成的设备驱动使用。
*/
#ifndef __TFMINIPLUS_H__
#define __TFMINIPLUS_H__

#include "stdint.h"
#include "bsp_iic.h"

typedef struct
{
	IICInstance *iic;
	uint8_t Mode;	   // 保存下标 6 的原始字节，本实现未进一步解释其含义。
	uint16_t Dist;	   // 下标 2、3 合成的原始距离，实际单位须与设备配置核对。
	uint32_t Strength; // 下标 4、5 合成的信号强度，未换算。
	uint8_t buf[9];	
} TFMiniInstance;

/** @brief 分配对象、注册地址 0x10 的阻塞 I²C，等待 0.5 s。
* @param[in] hi2c 已配置的有效 I²C 句柄，DWT 须已初始化。
* @return 分配对象；未检查分配和注册失败，没有注销入口，只应初始化一次。
*/
TFMiniInstance *TFMiniRegister(I2C_HandleTypeDef *hi2c);

/** @brief 发送请求、读取数据并返回原始距离值，未检查帧头、校验或 I²C 成败。
* @param[in,out] tfmini 有效注册对象，不能为 NULL。
* @return buf[2]、buf[3] 合成的整数转为 float；读失败仍会处理数组，不提供错误返回。
* @note 读取长度与成员容量不一致的问题须先处理，参见文件头；距离单位未在此驱动中确认。
*/
float GetDistance(TFMiniInstance *tfmini);

#endif
