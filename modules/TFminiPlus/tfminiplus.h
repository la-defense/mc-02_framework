/** @file tfminiplus.h
* @brief 保留的 TFmini I²C 测距入口，距离和强度只保存原始整数，未做单位换算。
* I²C 毫米数据响应为 9 字节；本文件只保证请求长度与接收数组一致，不代表已完成设备帧校验。
*/
#ifndef __TFMINIPLUS_H__
#define __TFMINIPLUS_H__

#include "stdint.h"
#include "bsp_iic.h"

#define TFMINI_DATA_FRAME_SIZE 9u // TFmini Plus I²C 毫米数据响应长度，见用户手册表 8。

typedef struct
{
	IICInstance *iic;
	uint8_t Mode;	   // 保存下标 6 的原始字节，本实现未进一步解释其含义。
	uint16_t Dist;	   // 下标 2、3 合成的原始距离，实际单位须与设备配置核对。
	uint32_t Strength; // 下标 4、5 合成的信号强度，未换算。
	uint8_t buf[TFMINI_DATA_FRAME_SIZE];
} TFMiniInstance;

/**
 * @brief 分配 TFmini 对象并登记 7 位地址 0x10 的阻塞式 I²C 设备。
 * @param[in] hi2c HAL 句柄；为空时登记失败，非空时须已初始化且生命周期长于登记对象。
 * @return 成功返回由调用方持有、不可释放的 TFmini 对象；外层分配或 I²C 登记失败返回 NULL。
 * @note 登记失败时释放本次对象，且不会执行 DWT 延时。成功后等待0.5秒，调用前须初始化 DWT。
 *       GetDistance 的接收长度问题仍未修复，不应据此认定该设备驱动完整可用。
 */
TFMiniInstance *TFMiniRegister(I2C_HandleTypeDef *hi2c);

/** @brief 发送毫米数据请求，读取 9 字节响应并组合原始距离值。
* @param[in,out] tfmini 有效注册对象，不能为 NULL；须先由 TFMiniRegister() 登记。
* @return 将 buf[2]、buf[3] 按小端组合后转为 float；本接口不报告传输失败。
* @note 请求和缓冲区长度与用户手册表 8 一致。当前 IICReceive() 不返回实际接收长度，
*       本函数也不检查帧头、校验、短帧或单位；调用方不能据此认定读数有效。
*/
float GetDistance(TFMiniInstance *tfmini);

#endif
