/** @file usb_hardware_model.h
 * @brief H723端点和ST库调用模型；仅生产后端测试包含，不复制工程USB业务类型。
 * @note 模型用于控制返回值和观察停止顺序，不代表实际USB电平或FIFO停止时序。
 */
#ifndef MC02_USB_HARDWARE_MODEL_H
#define MC02_USB_HARDWARE_MODEL_H
#include <stddef.h>
#include <stdint.h>
#define UNUSED(value) ((void)(value))
typedef enum { HAL_OK, HAL_ERROR, HAL_BUSY, HAL_TIMEOUT } HAL_StatusTypeDef;
typedef enum { USBD_OK, USBD_BUSY, USBD_FAIL } USBD_StatusTypeDef;
typedef struct { uint32_t DIEPCTL; } USB_InEndpoint;
typedef struct { USB_InEndpoint in[2]; uint8_t connected; } USB_Hardware;
typedef struct { USB_Hardware *Instance; unsigned State; } PCD_HandleTypeDef;
typedef struct { uint32_t TxState; const uint8_t *buffer; size_t length; } USBD_CDC_HandleTypeDef;
typedef struct { unsigned dev_state; void *pClassData; } USBD_HandleTypeDef;
typedef struct { unsigned unused; } USBD_DescriptorsTypeDef;
typedef struct { unsigned unused; } USBD_ClassTypeDef;
typedef struct {
    int8_t (*Init)(void);
    int8_t (*DeInit)(void);
    int8_t (*Control)(uint8_t,uint8_t *,uint16_t);
    int8_t (*Receive)(uint8_t *,uint32_t *);
    int8_t (*TransmitCplt)(uint8_t *,uint32_t *,uint8_t);
} USBD_CDC_ItfTypeDef;
#define USBx_INEP(index) (&((USB_Hardware *)(uintptr_t)USBx_BASE)->in[(index)])
#define USB_OTG_DIEPCTL_EPENA (1u<<31)
#define CDC_IN_EP 0x81u
#define DEVICE_HS 1u
#define USB_DEVICE_MODE 0u
#define HAL_PCD_STATE_RESET 0u
#define HAL_PCD_STATE_READY 1u
#define USBD_STATE_CONFIGURED 3u
#define OTG_HS_IRQn 77u
#define CDC_SEND_ENCAPSULATED_COMMAND 0u
#define CDC_GET_ENCAPSULATED_RESPONSE 1u
#define CDC_SET_COMM_FEATURE 2u
#define CDC_GET_COMM_FEATURE 3u
#define CDC_CLEAR_COMM_FEATURE 4u
#define CDC_SET_LINE_CODING 5u
#define CDC_GET_LINE_CODING 6u
#define CDC_SET_CONTROL_LINE_STATE 7u
#define CDC_SEND_BREAK 8u

extern USBD_DescriptorsTypeDef HS_Desc;
extern USBD_ClassTypeDef USBD_CDC;
uint32_t HAL_GetTick(void);
uint32_t __get_PRIMASK(void);
void __disable_irq(void);
void __set_PRIMASK(uint32_t value);
uint32_t __get_IPSR(void);
uint8_t BSP_HALTickTryAcquireBlockingWait(void);
void BSP_HALTickReleaseBlockingWait(void);
void HAL_NVIC_DisableIRQ(unsigned irq);
void HAL_NVIC_ClearPendingIRQ(unsigned irq);
void USBModelDisablePCD(PCD_HandleTypeDef *pcd);
#define __HAL_PCD_DISABLE(pcd) USBModelDisablePCD(pcd)
HAL_StatusTypeDef USB_DevDisconnect(USB_Hardware *hardware);
HAL_StatusTypeDef USB_StopDevice(USB_Hardware *hardware);
uint32_t USB_GetMode(USB_Hardware *hardware);
USBD_StatusTypeDef USBD_Init(USBD_HandleTypeDef *,USBD_DescriptorsTypeDef *,uint8_t);
USBD_StatusTypeDef USBD_DeInit(USBD_HandleTypeDef *);
USBD_StatusTypeDef USBD_RegisterClass(USBD_HandleTypeDef *,USBD_ClassTypeDef *);
USBD_StatusTypeDef USBD_CDC_RegisterInterface(USBD_HandleTypeDef *,USBD_CDC_ItfTypeDef *);
USBD_StatusTypeDef USBD_Start(USBD_HandleTypeDef *);
uint8_t USBD_CDC_SetTxBuffer(USBD_HandleTypeDef *,uint8_t *,uint32_t);
uint8_t USBD_CDC_SetRxBuffer(USBD_HandleTypeDef *,uint8_t *);
uint8_t USBD_CDC_TransmitPacket(USBD_HandleTypeDef *);
uint8_t USBD_CDC_ReceivePacket(USBD_HandleTypeDef *);
#endif
