#ifndef MC02_VCP_FAKE_USBD_CDC_IF_H
#define MC02_VCP_FAKE_USBD_CDC_IF_H
#include "usb_device.h"
uint8_t *CDCInitRxbufferNcallback(USBCallback tx_cbk, USBCallback rx_cbk);
uint8_t CDC_Transmit_HS(uint8_t *buffer, uint16_t len);
#endif
