/** @file usb_backend_test.c
 * @brief 链接生产USB核心、H723后端和CDC回调，核对发送归属及停止失败不释放。
 * @note HAL／寄存器为本机模型；实际接线、枚举和物理停止另由裸板验证。
 */
#include "usb_transport_port_stm32.h"
#include "usb_hardware_model.h"
#include "usbd_cdc_if.h"
#include "usb_diagnostic.h"
#include "test_check.h"
#include <string.h>

USBD_HandleTypeDef hUsbDeviceHS;
PCD_HandleTypeDef hpcd_USB_OTG_HS;
USBD_DescriptorsTypeDef HS_Desc;
USBD_ClassTypeDef USBD_CDC;
static USB_Hardware hardware;
static USBD_CDC_HandleTypeDef cdc;
static uint32_t now_ms, primask, ipsr;
static unsigned leases, deinitializations, fifo_stops, irq_disabled, irq_cleared;
static unsigned frames, notifications;
static uint8_t *rx_buffer;
static uint8_t receive_status = USBD_OK;
static HAL_StatusTypeDef fifo_status = HAL_OK;
static USBTxEvent_t notified;
uint32_t HAL_GetTick(void) { return now_ms; }
uint32_t __get_PRIMASK(void) { return primask; }
void __disable_irq(void) { primask=1u; }
void __set_PRIMASK(uint32_t value) { primask=value; }
uint32_t __get_IPSR(void) { return ipsr; }
uint8_t BSP_HALTickTryAcquireBlockingWait(void) { ++leases; return 1u; }
void BSP_HALTickReleaseBlockingWait(void) { TEST_CHECK(leases>0u); --leases; }
void HAL_NVIC_DisableIRQ(unsigned irq) { TEST_CHECK(irq==OTG_HS_IRQn); irq_disabled=1u; }
void HAL_NVIC_ClearPendingIRQ(unsigned irq) { TEST_CHECK(irq==OTG_HS_IRQn); ++irq_cleared; }
void USBModelDisablePCD(PCD_HandleTypeDef *pcd) { TEST_CHECK(pcd==&hpcd_USB_OTG_HS); }
HAL_StatusTypeDef USB_DevDisconnect(USB_Hardware *value)
{ TEST_CHECK(value==&hardware && irq_disabled); value->connected=0u; return HAL_OK; }
HAL_StatusTypeDef USB_StopDevice(USB_Hardware *value)
{ TEST_CHECK(value==&hardware && irq_disabled); ++fifo_stops; return fifo_status; }
uint32_t USB_GetMode(USB_Hardware *value) { TEST_CHECK(value==&hardware); return USB_DEVICE_MODE; }
USBD_StatusTypeDef USBD_Init(USBD_HandleTypeDef *device,USBD_DescriptorsTypeDef *desc,uint8_t id)
{
    TEST_CHECK(device==&hUsbDeviceHS && desc==&HS_Desc && id==DEVICE_HS);
    hpcd_USB_OTG_HS.Instance=&hardware; hpcd_USB_OTG_HS.State=HAL_PCD_STATE_READY;
    return USBD_OK;
}
USBD_StatusTypeDef USBD_DeInit(USBD_HandleTypeDef *device)
{
    TEST_CHECK(device==&hUsbDeviceHS && fifo_stops>0u && fifo_status==HAL_OK && irq_disabled);
    ++deinitializations; cdc.TxState=0u; hardware.in[1].DIEPCTL=0u;
    device->dev_state=0u; device->pClassData=NULL;
    TEST_CHECK(USBD_Interface_fops_HS.DeInit()==USBD_OK);
    hpcd_USB_OTG_HS.State=HAL_PCD_STATE_RESET;
    return USBD_OK;
}
USBD_StatusTypeDef USBD_RegisterClass(USBD_HandleTypeDef *device,USBD_ClassTypeDef *klass)
{ TEST_CHECK(device==&hUsbDeviceHS && klass==&USBD_CDC); return USBD_OK; }
USBD_StatusTypeDef USBD_CDC_RegisterInterface(USBD_HandleTypeDef *device,USBD_CDC_ItfTypeDef *interface)
{ TEST_CHECK(device==&hUsbDeviceHS && interface==&USBD_Interface_fops_HS); return USBD_OK; }
USBD_StatusTypeDef USBD_Start(USBD_HandleTypeDef *device)
{ TEST_CHECK(device==&hUsbDeviceHS); hardware.connected=1u; irq_disabled=0u; return USBD_OK; }
uint8_t USBD_CDC_SetTxBuffer(USBD_HandleTypeDef *device,uint8_t *bytes,uint32_t length)
{ TEST_CHECK(device==&hUsbDeviceHS); cdc.buffer=bytes; cdc.length=length; return USBD_OK; }
uint8_t USBD_CDC_SetRxBuffer(USBD_HandleTypeDef *device,uint8_t *bytes)
{ TEST_CHECK(device==&hUsbDeviceHS); rx_buffer=bytes; return USBD_OK; }
uint8_t USBD_CDC_TransmitPacket(USBD_HandleTypeDef *device)
{
    TEST_CHECK(device==&hUsbDeviceHS && cdc.TxState==0u);
    cdc.TxState=1u; hardware.in[1].DIEPCTL=USB_OTG_DIEPCTL_EPENA; return USBD_OK;
}
uint8_t USBD_CDC_ReceivePacket(USBD_HandleTypeDef *device)
{ TEST_CHECK(device==&hUsbDeviceHS); return receive_status; }
static void Received(void *context,const USBRxBlock_t *block)
{
    (void)context; TEST_CHECK(ipsr==0u && block->length==4u && block->data!=rx_buffer);
    TEST_CHECK(block->data[0]==0x53u && block->received_ms==0u); ++frames;
}
static void Completed(void *context,const USBTxEvent_t *event)
{ (void)context; TEST_CHECK(ipsr==0u); notified=*event; ++notifications; }
static USBTransportState_t State(void)
{ USBTransportState_t output; TEST_CHECK(USBTransportGetState(&output)==USB_OK); return output; }
static void Configured(void)
{
    hUsbDeviceHS.dev_state=USBD_STATE_CONFIGURED; hUsbDeviceHS.pClassData=&cdc;
    cdc.TxState=0u; hardware.in[1].DIEPCTL=0u;
    ipsr=1u; TEST_CHECK(USBD_Interface_fops_HS.Init()==USBD_OK); ipsr=0u;
    TEST_CHECK(USBTransportProcess()==USB_OK);
}
#if MC02_BENCH_USB_DIAGNOSTICS
// 通过真实CDC完成入口产生或丢弃通知，不直接改生产核心的计数或恢复状态。
static void HardwareCompleted(void)
{
    uint32_t length=(uint32_t)cdc.length;
    cdc.TxState=0u; hardware.in[1].DIEPCTL=0u;
    ipsr=1u; TEST_CHECK(USBD_Interface_fops_HS.TransmitCplt(NULL,&length,CDC_IN_EP)==USBD_OK); ipsr=0u;
}
static void HealthyCycle(void)
{
    const uint8_t byte=0u; uint32_t id;
    TEST_CHECK(USBTransportSubmit(&byte,1u,&id)==USB_ACCEPTED);
    USBTransportReportValidCommand(State().generation,now_ms);
    HardwareCompleted(); TEST_CHECK(USBTransportProcess()==USB_OK);
    USBTransportDiagnosticPoll();
}
static void Diagnostic(void)
{
    // 5秒双向正常前不注入；每次运行只登记一轮四个丢失通知。
    for(now_ms=0u;now_ms<=5000u;now_ms+=20u) HealthyCycle();
    TEST_CHECK(usb_diagnostic_record.phase==USB_DIAG_INJECTING && usb_diagnostic_record.remaining==4u);
    for(unsigned failure=0u;failure<4u;++failure)
    {
        now_ms+=1000u;
        const uint8_t byte=0u; uint32_t id;
        TEST_CHECK(USBTransportSubmit(&byte,1u,&id)==USB_ACCEPTED);
        HardwareCompleted(); USBTransportDiagnosticPoll();
        TEST_CHECK(State().tx_buffer_held && usb_diagnostic_record.suppressed==failure+1u);
        now_ms+=100u; USBTransportProcess(); USBTransportDiagnosticPoll();
        if(failure<3u)
        {
            TEST_CHECK(State().phase==USB_PHASE_DISCONNECT_WAIT && State().restart_attempts==failure+1u);
            now_ms+=250u; USBTransportProcess(); Configured(); USBTransportDiagnosticPoll();
        }
    }
    TEST_CHECK(usb_diagnostic_record.phase==USB_DIAG_WAIT_RECONFIGURE &&
               State().phase==USB_PHASE_EXHAUSTED && State().restart_total==3u);
    Configured(); USBTransportDiagnosticPoll();
    TEST_CHECK(State().restart_attempts==3u && usb_diagnostic_record.phase==USB_DIAG_WAIT_QUOTA);
    uint32_t base=now_ms;
    for(uint32_t elapsed=0u;elapsed<=65000u;elapsed+=20u)
    { now_ms=base+elapsed; HealthyCycle(); }
    TEST_CHECK(State().restart_attempts==0u && usb_diagnostic_record.phase==USB_DIAG_COMPLETE);
    TEST_CHECK(usb_diagnostic_record.suppressed==4u && usb_diagnostic_record.remaining==0u);
    TEST_CHECK(usb_diagnostic_record.magic==USB_DIAGNOSTIC_MAGIC &&
               usb_diagnostic_record.size_bytes==sizeof(USBDiagnosticRecord_t) &&
               (usb_diagnostic_record.publication_generation&1u)==0u && leases==0u);
}
#endif
int main(int argc,char **argv)
{
    TEST_CHECK(argc==2);
    USBTransportConfig_t config={Received,Completed,NULL,NULL};
    TEST_CHECK(USBTransportInit(&config)==USB_OK && USBTransportProcess()==USB_OK);
    Configured();
#if MC02_BENCH_USB_DIAGNOSTICS
    if(strcmp(argv[1],"diagnostic")==0) { Diagnostic(); return 0; }
#endif
    if(strcmp(argv[1],"completion")==0)
    {
        uint8_t bytes[4]={0x53u,0x50u,0u,0u}; uint32_t length=4u;
        ipsr=1u; TEST_CHECK(USBD_Interface_fops_HS.Receive(bytes,&length)==USBD_OK); ipsr=0u;
        memset(bytes,0xee,sizeof(bytes)); USBTransportProcessRx(); TEST_CHECK(frames==1u);
        uint32_t id; bytes[0]=0x53u;
        TEST_CHECK(USBTransportSubmit(bytes,sizeof(bytes),&id)==USB_ACCEPTED);
        memset(bytes,0xaa,sizeof(bytes)); TEST_CHECK(cdc.buffer!=bytes && cdc.buffer[0]==0x53u);
        TEST_CHECK(USBTransportSubmit(bytes,sizeof(bytes),&id)==USB_BUSY);
        hardware.in[1].DIEPCTL=0u; cdc.TxState=0u; length=4u;
        ipsr=1u; TEST_CHECK(USBD_Interface_fops_HS.TransmitCplt(NULL,&length,CDC_IN_EP)==USBD_OK); ipsr=0u;
        TEST_CHECK(notifications==0u && leases==0u); USBTransportProcess();
        TEST_CHECK(notifications==1u && notified.result==USB_TX_COMPLETE);
    }
    else if(strcmp(argv[1],"fifo-failure")==0 || strcmp(argv[1],"stop")==0)
    {
        uint32_t id; const uint8_t byte=1u;
        TEST_CHECK(USBTransportSubmit(&byte,1u,&id)==USB_ACCEPTED);
        if(strcmp(argv[1],"fifo-failure")==0) fifo_status=HAL_TIMEOUT;
        now_ms=100u; USBStatus_t result=USBTransportProcess();
        if(fifo_status==HAL_TIMEOUT)
            TEST_CHECK(result==USB_ISOLATED && deinitializations==0u && irq_cleared==0u &&
                       State().phase==USB_PHASE_ISOLATED && State().tx_buffer_held && leases==2u);
        else
            TEST_CHECK(result==USB_OK && deinitializations==1u && irq_cleared==1u &&
                       State().phase==USB_PHASE_DISCONNECT_WAIT && !State().tx_buffer_held && leases==1u);
        TEST_CHECK(notifications==1u && notified.result==USB_TX_TIMEOUT);
    }
    else if(strcmp(argv[1],"rearm-error")==0)
    {
        uint8_t bytes[4]={0x53u}; uint32_t length=4u; receive_status=USBD_FAIL;
        ipsr=1u; TEST_CHECK(USBD_Interface_fops_HS.Receive(bytes,&length)==USBD_FAIL); ipsr=0u;
        TEST_CHECK(!USBTransportGenerationIsCurrent(1u));
        USBTransportProcess(); TEST_CHECK(State().restart_attempts==1u && frames==0u);
    }
    else TEST_CHECK(0);
    return 0;
}
