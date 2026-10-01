/**
 * @file bsp_usb.h
 * @author your name (you@domain.com)
 * @brief 提供对usb vpc(virtal com port)的操作接口,hid和msf考虑后续添加
 * @attention 这一版usb修改了usbd_cdc_if.c中的CDC_Receive_FS函数,若使用cube生成后会被覆盖.后续需要由usbcdciftemplate创建一套自己的模板
 * @version 0.1
 * @date 2023-02-09
 *
 * @copyright Copyright (c) 2023
 *
 */
#pragma once
#include "usb_device.h"
#include "usbd_cdc.h"
#include "usbd_conf.h"
#include "usbd_desc.h"
#include "usbd_cdc_if.h"

typedef struct
{
    USBCallback tx_cbk;
    USBCallback rx_cbk;
} USB_Init_Config_s;

/* @note 虚拟串口的波特率/校验位/数据位等动态可变,取决于上位机的设定 */
/* 使用时不需要关心这些设置(作为从机) */

uint8_t *USBInit(USB_Init_Config_s usb_conf); // bsp初始化时调用会重新枚举设备

uint8_t USBTransmit(const uint8_t *buffer, uint16_t len); // 通过 USB 发送数据，返回 USBD 状态

/**
 * @brief 把 USB 收到的一包数据压进队列(由 CDC_Receive_HS 在中断上下文调用)
 * @note  接收中断不再直接解析: 由 VCPProcessRx() 在任务里按到达顺序逐包调用
 *        rx_cbk, 避免在中断里做滑动找帧 + CRC 校验这类重活。
 */
void VCPQueuePush(const uint8_t *buf, uint16_t len);

/**
 * @brief 逐包派发 VCP 收到的数据(必须在任务上下文调用, 全工程只允许一个调用者)
 */
void VCPProcessRx(void);
