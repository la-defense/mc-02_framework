/** @file bsp_usb.h
 * @brief CRC 测试的 USB 收发替身，由测试提供接收字节和时间。
 */
#ifndef MC02_CRC_TEST_USB_H
#define MC02_CRC_TEST_USB_H
#include <stdint.h>
#define USBD_OK 0u
#define USBD_BUSY 1u
#define UNUSED(value) ((void)(value))
typedef struct { void (*rx_cbk)(uint16_t); } USB_Init_Config_s;
/** @brief 保存接收回调。
 * @param[in] config 回调配置，替身保存函数地址。
 * @return 测试提供的64字节接收数组，生命周期覆盖测试进程。
 */
uint8_t *USBInit(USB_Init_Config_s config);
/** @brief 将发送字节复制到测试记录中。
 * @param[in] data 至少可读 length 字节，复制后可复用。
 * @param[in] length 字节数，不得超过测试记录容量512字节。
 * @return USBD_OK；CRC 失败发生在本入口调用之前。
 */
uint8_t USBTransmit(const uint8_t *data, uint16_t length);
/** @brief 取得测试接收数组；返回地址持续有效，内容由测试设置。 */
const uint8_t *VCPGetCurrentRxData(void);
/** @brief 取得测试当前接收时间，返回毫秒数，由测试设置。 */
uint32_t VCPGetCurrentRxTimestamp(void);
#endif
