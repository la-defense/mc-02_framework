#include <assert.h>
#include <stdint.h>
#include <string.h>
#include "bsp_usb.h"

static uint8_t endpoint_buffer[128];
static USBCallback registered_rx;
static uint32_t now_ms;
static uint32_t expected_timestamp;
static unsigned callback_count;

uint32_t HAL_GetTick(void) { return now_ms; }
uint32_t __get_PRIMASK(void) { return 0u; }
void __disable_irq(void) {}
void __set_PRIMASK(uint32_t value) { (void)value; }
void __DMB(void) {}
uint8_t *CDCInitRxbufferNcallback(USBCallback tx_cbk, USBCallback rx_cbk)
{ (void)tx_cbk; registered_rx=rx_cbk; return endpoint_buffer; }
uint8_t CDC_Transmit_HS(uint8_t *buffer, uint16_t len)
{ (void)buffer; (void)len; return USBD_OK; }

static void receive_callback(uint16_t len)
{
    const uint8_t *frame=VCPGetCurrentRxData();
    assert(frame != NULL && frame != endpoint_buffer && len == 4u);
    assert(frame[0] == 0x53u && frame[3] == 0x50u);
    assert(VCPGetCurrentRxTimestamp() == expected_timestamp);
    memset(endpoint_buffer, 0xff, 4u); /* Re-armed endpoint cannot alter queued bytes. */
    assert(frame[0] == 0x53u && frame[3] == 0x50u);
    callback_count++;
}

int main(void)
{
    assert(USBInit((USB_Init_Config_s){.rx_cbk=receive_callback}) == endpoint_buffer);
    assert(registered_rx == receive_callback);
    const uint8_t packet[4]={0x53u,0x50u,0x01u,0x50u};

    now_ms=0u;
    VCPQueuePush(packet,sizeof(packet));
    now_ms=51u;
    VCPProcessRx();
    assert(callback_count==0u && VCPGetCurrentRxData()==NULL);

    now_ms=100u;
    expected_timestamp=now_ms;
    VCPQueuePush(packet,sizeof(packet));
    now_ms=150u;
    VCPProcessRx();
    assert(callback_count==1u && VCPGetCurrentRxData()==NULL);
    assert(VCPGetCurrentRxTimestamp()==0u);
    return 0;
}
