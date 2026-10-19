/** @file crc_protocol_test.c
 * @brief 经生产接收回调和发送入口检查 SP、裁判及 UI 的 CRC 兼容和失败处理。
 * @note 替身只模拟传输与时间；字节校验和解析运行生产代码。
 */
#include "crc.h"
#include "crc_injection.h"
#include "test_check.h"
#include "master_process.h"
#include "referee_UI.h"
#include "daemon.h"
#include "bsp_usb.h"
#include <string.h>

uint8_t UI_Seq;
static uint32_t now_ms, reloads, usb_sends, uart_sends;
static uint8_t usb_bytes[64], uart_bytes[255], sent[512];
static uint16_t sent_length;
static void (*usb_callback)(uint16_t);
static void (*uart_callback)(void);
static USARTInstance uart_instance = {uart_bytes, 0u};
static DaemonInstance daemon_instance;

uint32_t HAL_GetTick(void) { return now_ms; }
DaemonInstance *DaemonRegister(Daemon_Init_Config_s *config) { (void)config; return &daemon_instance; }
uint8_t DaemonUnregister(DaemonInstance *instance) { (void)instance; return 1u; }
void DaemonReload(DaemonInstance *instance) { (void)instance; ++reloads; }
uint8_t DaemonIsOnline(DaemonInstance *instance) { (void)instance; return 1u; }
USARTInstance *USARTRegister(USART_Init_Config_s *config)
{ uart_callback = config->module_callback; return &uart_instance; }
void USARTServiceInit(USARTInstance *instance) { (void)instance; }
void USARTSend(USARTInstance *instance, uint8_t *data, uint16_t length, int mode)
{
    (void)instance; (void)mode;
    TEST_CHECK(length <= sizeof(sent));
    memcpy(sent, data, length); sent_length = length; ++uart_sends;
}
uint8_t USARTIsReady(USARTInstance *instance) { (void)instance; return 1u; }
USARTSendStatus_t USARTSendChecked(USARTInstance *instance, const uint8_t *data, uint16_t length, int mode)
{
    USARTSend(instance, (uint8_t *)data, length, mode);
    return USART_SEND_ACCEPTED;
}
void osDelay(uint32_t milliseconds) { (void)milliseconds; }
uint8_t *USBInit(USB_Init_Config_s config) { usb_callback = config.rx_cbk; return usb_bytes; }
const uint8_t *VCPGetCurrentRxData(void) { return usb_bytes; }
uint32_t VCPGetCurrentRxTimestamp(void) { return now_ms; }
uint8_t USBTransmit(const uint8_t *data, uint16_t length)
{
    TEST_CHECK(length <= sizeof(sent));
    memcpy(sent, data, length); sent_length = length; ++usb_sends; return USBD_OK;
}

// 用固定协议参数逐位计算对照值，避免组包与验包共享查表错误。
static uint32_t reference(const uint8_t *bytes, size_t length, uint32_t poly, uint32_t initial)
{
    uint32_t crc = initial;
    for (size_t index = 0u; index < length; ++index)
    {
        crc ^= bytes[index];
        for (unsigned bit = 0u; bit < 8u; ++bit) crc = (crc >> 1u) ^ ((crc & 1u) ? poly : 0u);
    }
    return crc;
}

static void check_referee_frame(void)
{
    TEST_CHECK(sent[4] == reference(sent, 4u, 0x8cu, 0xffu));
    TEST_CHECK(reference(sent, sent_length - 2u, 0x8408u, 0xffffu) ==
               ((uint16_t)sent[sent_length - 2u] | ((uint16_t)sent[sent_length - 1u] << 8u)));
}

int main(void)
{
    // 上一轮 1b10845 裸板被动采集的43字节状态帧，保留历史线上字节。
    const uint8_t legacy_status[] = {0x53, 0x50, 0x00, 0x91, 0x47, 0x7f, 0x3f, 0x2c, 0x1f, 0xe4, 0xbb, 0xe9, 0x6d, 0x4b, 0xbc, 0xa8, 0x5d, 0x39, 0xbd, 0x45, 0x0e, 0xba, 0xbd, 0x58, 0xa0, 0x8b, 0xba, 0x6a, 0x8f, 0x52, 0xbc, 0x58, 0xa0, 0x0b, 0xbb, 0x00, 0x00, 0xf0, 0x41, 0x00, 0x00, 0xd9, 0x8e};
    TEST_CHECK(VisionProtocolCheckCRC16(legacy_status, sizeof(legacy_status)));
    TEST_CHECK(VisionInit(NULL) != NULL);
    Vision_Recv_s command = {0};
    command.head[0] = 'S'; command.head[1] = 'P';
    command.crc16 = (uint16_t)reference((const uint8_t *)&command, sizeof(command) - 2u, 0x8408u, 0xffffu);
    memcpy(usb_bytes, &command, sizeof(command));
    now_ms = 10u; usb_callback(sizeof(command));
    Vision_Status_t status;
    VisionGetStatus(&status);
    TEST_CHECK(status.last_rx_ms == 10u && status.rx_count == 1u);
    uint32_t old_reloads = reloads;
    crc_fail_after = 0; now_ms = 11u; usb_callback(sizeof(command));
    VisionGetStatus(&status);
    TEST_CHECK(reloads == old_reloads && status.last_rx_ms == 10u && status.rx_count == 1u);
    crc_fail_after = 0; VisionSend(); TEST_CHECK(usb_sends == 0u);
    VisionSend(); TEST_CHECK(usb_sends == 1u && sent_length == 43u);
    TEST_CHECK(reference(sent, 41u, 0x8408u, 0xffffu) == ((uint16_t)sent[41] | ((uint16_t)sent[42] << 8u)));

    UART_HandleTypeDef handle = {0};
    referee_info_t *info = RefereeInit(&handle);
    TEST_CHECK(info != NULL);
    uint8_t frame[] = {0xa5, 1, 0, 7, 0, 2, 0, 5, 0, 0};
    frame[4] = (uint8_t)reference(frame, 4u, 0x8cu, 0xffu);
    uint16_t tail = (uint16_t)reference(frame, 8u, 0x8408u, 0xffffu);
    frame[8] = (uint8_t)tail; frame[9] = (uint8_t)(tail >> 8u);
    memcpy(uart_bytes, frame, sizeof(frame)); uart_instance.last_recv_size = sizeof(frame);
    now_ms = 20u; uart_callback();
    TEST_CHECK(RefereeGetLastRxMs() == 20u && info->CmdID == 2u);
    referee_info_t saved = *info; old_reloads = reloads;
    // 分别让帧头与整帧计算失败，都不能刷新时间或发布字段。
    for (int failure = 0; failure < 2; ++failure)
    {
        crc_fail_after = failure; now_ms = 21u; uart_callback();
        TEST_CHECK(RefereeGetLastRxMs() == 20u && reloads == old_reloads);
        TEST_CHECK(memcmp(&saved, info, sizeof(saved)) == 0);
    }
    uart_bytes[7] ^= 1u; now_ms = 22u; uart_callback();
    TEST_CHECK(RefereeGetLastRxMs() == 20u && reloads == old_reloads);

    referee_id_t id = {.Robot_ID = 1u, .Cilent_ID = 0x101u};
    Graph_Data_t graph = {0}; String_Data_t text = {0};
    UIDelete(&id, 1u, 0u); check_referee_frame();
    UIGraphRefresh(&id, &graph, 1u, 1u); check_referee_frame();
    TEST_CHECK(UICharDraw(&text, "crc", 1u, 0u, 0u, 15u, 2u, 0u, 0u, "CRC") == UI_OK);
    UICharRefresh(&id, &text); check_referee_frame();
    for (int failure = 0; failure < 2; ++failure)
    {
        uint32_t previous = uart_sends; uint8_t sequence = UI_Seq;
        crc_fail_after = failure; UIDelete(&id, 1u, 0u);
        TEST_CHECK(uart_sends == previous && UI_Seq == sequence);
        crc_fail_after = failure; UIGraphRefresh(&id, &graph, 1u, 1u);
        TEST_CHECK(uart_sends == previous && UI_Seq == sequence);
        crc_fail_after = failure; UICharRefresh(&id, &text);
        TEST_CHECK(uart_sends == previous && UI_Seq == sequence);
    }
    return 0;
}
