#ifndef MC02_VCP_FAKE_USB_DEVICE_H
#define MC02_VCP_FAKE_USB_DEVICE_H
#include <stdint.h>
#include "main.h"
typedef void (*USBCallback)(uint16_t len);
uint32_t HAL_GetTick(void);
#define USBD_OK 0u
#define USBD_BUSY 1u
#define USBD_FAIL 2u
#endif
