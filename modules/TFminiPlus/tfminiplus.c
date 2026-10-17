/** @file tfminiplus.c
* @brief 保留的测距请求和原始字段组合；已知接收长度问题见头文件，当前不保证设备数据有效。
*/
#include "tfminiplus.h"
#include "stdlib.h"
#include "bsp_dwt.h"

static TFMiniInstance *tfmini;

TFMiniInstance *TFMiniRegister(I2C_HandleTypeDef *hi2c)
{
	tfmini = (TFMiniInstance *)malloc(sizeof(TFMiniInstance));
	IIC_Init_Config_s conf = {
		.handle = hi2c,
		.dev_address = 0x10,
		.id = tfmini,
		.work_mode = IIC_BLOCK_MODE,
	};
	tfmini->iic = IICRegister(&conf);
	DWT_Delay(0.5);
	return tfmini;
}

float GetDistance(TFMiniInstance *tfmini)
{
	uint8_t cmd_buf[5] = {0x5A, 0x05, 0x00, 0x06, 0x65}; 
	IICTransmit(tfmini->iic, cmd_buf, 5, IIC_SEQ_RELEASE);
	// 当前请求 11 字节而 buf 仅 9 字节，尚需在设备接入前修正；这里沿用现有实现。
	IICReceive(tfmini->iic, tfmini->buf, 11, IIC_SEQ_RELEASE);
	// 按低字节在前组合距离和强度，没有单位转换，也没有检查本次接收是否成功。
	tfmini->Mode = tfmini->buf[6];
	tfmini->Dist = (uint16_t)tfmini->buf[2] + (((uint16_t)tfmini->buf[3]) << 8);
	tfmini->Strength = tfmini->buf[4] | (tfmini->buf[5] << 8);
	return tfmini->Dist;
}
