/**
 * @file bsp_usb.h
* @brief USB CDC 虚拟串口收发，接收中断入队，任务派发数据；不提供 HID 或存储设备接口。
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

/** @brief 设置收发回调并取得 CDC 接收数组，不执行设备重新枚举。
* @param[in] usb_conf 回调配置，按值传入；模块长期保存回调地址。
* @return CDC 内部接收数组地址，不可释放；本函数不检查枚举是否成功，外设初始化由 USB 设备代码完成。
 */
uint8_t *USBInit(USB_Init_Config_s usb_conf);

/** @brief 尝试提交 USB 发送，不复制数据到自己的发送队列。
* @param[in] buffer 至少可读 len 字节，不能为 NULL；异步发送完成前保持不变。
* @param[in] len 字节数，不能为 0。
* @return USBD_OK 为已提交，USBD_BUSY 为仍在发送，USBD_FAIL 为参数或底层失败。
 */
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
/** @brief 取得本次模块接收回调的数据地址。
* @return 回调期间可读本包长度的地址，不可改写或保存；回调外为 NULL。
 */
const uint8_t *VCPGetCurrentRxData(void);
/** @brief 取得当前派发包入队时的 HAL 毫秒值。
* @return 当前包接收时间，回调外为 0；派发不会刷新该时间。
 */
uint32_t VCPGetCurrentRxTimestamp(void);
