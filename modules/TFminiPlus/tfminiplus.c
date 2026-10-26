/** @file tfminiplus.c
* @brief 发送 TFmini Plus I²C 取数请求并组合响应中的原始字段；设备数据有效性仍未验证。
* @note 主要入口为 GetDistance()；使用前先初始化 HAL I²C 与 DWT，再通过 TFMiniRegister() 登记实例。
*/
#include "tfminiplus.h"
#include "bsp_dwt.h"
#include <stdlib.h>

TFMiniInstance *TFMiniRegister(I2C_HandleTypeDef *hi2c)
{
	TFMiniInstance *tfmini = (TFMiniInstance *)malloc(sizeof(TFMiniInstance));
	if (tfmini == NULL)
		return NULL;

	IIC_Init_Config_s conf = {
		.handle = hi2c,
		.dev_address = 0x10,
		.id = tfmini,
		.work_mode = IIC_BLOCK_MODE,
	};
	tfmini->iic = IICRegister(&conf);
	if (tfmini->iic == NULL)
	{
		free(tfmini);
		return NULL;
	}

	DWT_Delay(0.5);
	return tfmini;
}

float GetDistance(TFMiniInstance *tfmini)
{
	uint8_t cmd_buf[5] = {0x5A, 0x05, 0x00, 0x06, 0x65};
	IICTransmit(tfmini->iic, cmd_buf, 5, IIC_SEQ_RELEASE);
	// 用户手册中的毫米数据响应为 9 字节，长度必须与成员数组使用同一常量。
	IICReceive(tfmini->iic, tfmini->buf, TFMINI_DATA_FRAME_SIZE, IIC_SEQ_RELEASE);
	// 保留原有字段偏移；IICReceive 不提供实际长度和传输状态，不能在此确认数据有效。
	tfmini->Mode = tfmini->buf[6];
	tfmini->Dist = (uint16_t)tfmini->buf[2] + (((uint16_t)tfmini->buf[3]) << 8);
	tfmini->Strength = tfmini->buf[4] | (tfmini->buf[5] << 8);
	return tfmini->Dist;
}
