/** @file usb_transport_port_stm32.c
 * @brief H723单CDC后端；在USB任务内启动、确认停止，在ISR内登记事件。
 * @note 沿用Full Speed内置PHY且禁用DMA，不支持绕过本模块直接发送。
 * @copyright MC-02 contributors; 保留ST底层文件各自许可。
 */
#include "usb_transport_port_stm32.h"
#include "usb_device.h"
#include "usbd_desc.h"
#include "usbd_cdc_if.h"
#include "stm32h7xx_ll_usb.h"
#include "bsp_hal_tick.h"

extern USBD_HandleTypeDef hUsbDeviceHS;
extern PCD_HandleTypeDef hpcd_USB_OTG_HS;

uint32_t USBPortNowMs(void) { return HAL_GetTick(); }
uint32_t USBPortEnter(void)
{
    uint32_t saved = __get_PRIMASK();
    __disable_irq();
    return saved;
}
void USBPortExit(uint32_t saved) { __set_PRIMASK(saved); }
uint8_t USBPortInISR(void) { return (uint8_t)(__get_IPSR() != 0u); }
uint8_t USBPortAcquireTick(void) { return BSP_HALTickTryAcquireBlockingWait(); }
void USBPortReleaseTick(void) { BSP_HALTickReleaseBlockingWait(); }

// CDC只使用一个IN数据端点；重新配置时必须确认旧端点已不再执行发送。
static uint8_t BuffersReleased(void)
{
    if (hpcd_USB_OTG_HS.Instance == NULL || hpcd_USB_OTG_HS.State == HAL_PCD_STATE_RESET)
        return 1u;
    uintptr_t USBx_BASE = (uintptr_t)hpcd_USB_OTG_HS.Instance;
    return (uint8_t)((USBx_INEP(CDC_IN_EP & 0x7fu)->DIEPCTL & USB_OTG_DIEPCTL_EPENA) == 0u);
}

void USBPortOnConfigured(void)
{
    USBTransportOnLink(USB_LINK_CONFIGURED, BuffersReleased());
}
void USBPortOnLifecycle(USBLinkReason_t reason)
{
    // 暂停并不等于取消旧IN传输；未确认停止时继续保留数组。
    USBTransportOnLink(reason, BuffersReleased());
}

USBStatus_t USBPortStop(void)
{
    if (hpcd_USB_OTG_HS.Instance == NULL || hpcd_USB_OTG_HS.State == HAL_PCD_STATE_RESET)
        return USB_OK;
    // 仅屏蔽USB中断，其余任务和HAL毫秒计数继续推进；等待不包在全局临界区。
    HAL_NVIC_DisableIRQ(OTG_HS_IRQn);
    __HAL_PCD_DISABLE(&hpcd_USB_OTG_HS);
    if (USB_DevDisconnect(hpcd_USB_OTG_HS.Instance) != HAL_OK ||
        USB_StopDevice(hpcd_USB_OTG_HS.Instance) != HAL_OK)
        return USB_ISOLATED;
    // 先核对FIFO结果，再让库释放类对象；不依赖会吞掉FIFO错误的HAL_PCD_Stop返回值。
    if (USBD_DeInit(&hUsbDeviceHS) != USBD_OK ||
        hpcd_USB_OTG_HS.State != HAL_PCD_STATE_RESET)
        return USB_ISOLATED;
    HAL_NVIC_ClearPendingIRQ(OTG_HS_IRQn);
    return USB_OK;
}

USBStatus_t USBPortStart(void)
{
    USBStatus_t result = USB_OK;
    if (USBD_Init(&hUsbDeviceHS, &HS_Desc, DEVICE_HS) != USBD_OK ||
        hpcd_USB_OTG_HS.State != HAL_PCD_STATE_READY ||
        USB_GetMode(hpcd_USB_OTG_HS.Instance) != (uint32_t)USB_DEVICE_MODE ||
        USBD_RegisterClass(&hUsbDeviceHS, &USBD_CDC) != USBD_OK ||
        USBD_CDC_RegisterInterface(&hUsbDeviceHS, &USBD_Interface_fops_HS) != USBD_OK ||
        USBD_Start(&hUsbDeviceHS) != USBD_OK)
        result = USB_SEND_ERROR;
    if (result != USB_OK)
    {
        // 部分启动失败后也要确认停止；不能让下一次启动覆盖仍在使用的类对象。
        if (USBPortStop() != USB_OK) return USB_ISOLATED;
    }
    return result;
}

USBStatus_t USBPortSubmit(const uint8_t *data, size_t length)
{
    if (hUsbDeviceHS.dev_state != USBD_STATE_CONFIGURED) return USB_NOT_CONFIGURED;
    USBD_CDC_HandleTypeDef *cdc = (USBD_CDC_HandleTypeDef *)hUsbDeviceHS.pClassData;
    if (cdc == NULL) return USB_NOT_CONFIGURED;
    if (cdc->TxState != 0u) return USB_BUSY;
    if (data == NULL || length == 0u || length > USB_MAX_BYTES) return USB_BAD_ARGUMENT;
    if (USBD_CDC_SetTxBuffer(&hUsbDeviceHS, (uint8_t *)data, (uint32_t)length) != USBD_OK)
        return USB_SEND_ERROR;
    uint8_t result = USBD_CDC_TransmitPacket(&hUsbDeviceHS);
    if (result == USBD_OK) return USB_ACCEPTED;
    return result == USBD_BUSY ? USB_BUSY : USB_SEND_ERROR;
}
