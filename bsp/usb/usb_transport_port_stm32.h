/** @file usb_transport_port_stm32.h
 * @brief H723生成代码的最小USB接入声明；仅USB_DEVICE后端包含。
 * @note 普通业务只包含usb_transport.h，不调用这些中断接入入口。
 */
#ifndef MC02_USB_TRANSPORT_PORT_STM32_H
#define MC02_USB_TRANSPORT_PORT_STM32_H
#include "usb_transport_port.h"

/** @brief CDC完成配置后登记新连接；只在旧IN端点已停止时释放旧缓冲。 */
void USBPortOnConfigured(void);
/** @brief 总线复位／暂停等回调登记变化，不解析业务或调用用户完成回调。 */
void USBPortOnLifecycle(USBLinkReason_t reason);
#endif
