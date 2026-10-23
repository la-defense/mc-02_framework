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
#include "usb_transport_port.h"
#include <string.h>
#include <math.h>

// 序号由生产共享入口维护；测试通过只读副本查询。
static uint8_t current_sequence(void)
{
    RefereeTxState_t value;
    return RefereeGetTxState(&value) == USART_SEND_ACCEPTED ? value.next_sequence : 0u;
}
static uint32_t now_ms, reloads, usb_sends, uart_sends;
static uint8_t usb_bytes[64], uart_bytes[255], sent[512];
static uint16_t sent_length;
static usart_module_callback uart_callback;
static void *uart_context;
static uint32_t uart_sequence;
static void deliver_uart(size_t length)
{
    const USART_RxBlock_t block = {uart_bytes,length,now_ms,++uart_sequence};
    uart_callback(uart_context,&block);
}
static USARTInstance uart_instance = {0};
static DaemonInstance daemon_instance;

uint32_t HAL_GetTick(void) { return now_ms; }
DaemonInstance *DaemonRegister(Daemon_Init_Config_s *config) { (void)config; return &daemon_instance; }
uint8_t DaemonUnregister(DaemonInstance *instance) { (void)instance; return 1u; }
void DaemonReload(DaemonInstance *instance) { (void)instance; ++reloads; }
uint8_t DaemonIsOnline(DaemonInstance *instance) { (void)instance; return 1u; }
USARTInstance *USARTRegister(USART_Init_Config_s *config)
{ uart_callback = config->module_callback; uart_context=config->context; return &uart_instance; }
uint8_t USARTServiceInit(USARTInstance *instance) { (void)instance; return 1u; }
USARTSendStatus_t USARTSubmitTx(USARTInstance *instance,const USARTTxRequest_t *request,uint32_t *id)
{
    (void)instance;TEST_CHECK(request->length<=sizeof(sent));
    memcpy(sent,request->data,request->length);sent_length=(uint16_t)request->length;++uart_sends;*id=uart_sends;
    USARTTxEvent_t event={.id=*id,.result=USART_TX_COMPLETE,.started=1u,.started_ms=now_ms};
    request->complete(request->context,&event);return USART_SEND_ACCEPTED;
}
// 该目标只检查编码与状态选择；真实发送统计由uart_ui_integration_test覆盖。
USARTSendStatus_t USARTGetTxState(const USARTInstance *instance, USARTTxState_t *output)
{
    (void)instance; memset(output,0,sizeof(*output));
    return USART_SEND_ACCEPTED;
}
USARTSendStatus_t USARTCancelTx(USARTInstance *instance,uint32_t id)
{ (void)instance;(void)id;return USART_SEND_NOT_FOUND; }
void osDelay(uint32_t milliseconds) { (void)milliseconds; }
uint32_t USBPortNowMs(void) { return now_ms; }
uint32_t USBPortEnter(void) { return 0u; }
void USBPortExit(uint32_t saved) { (void)saved; }
uint8_t USBPortInISR(void) { return 0u; }
uint8_t USBPortAcquireTick(void) { return 1u; }
void USBPortReleaseTick(void) {}
USBStatus_t USBPortStart(void) { return USB_OK; }
USBStatus_t USBPortStop(void) { return USB_OK; }
USBStatus_t USBPortSubmit(const uint8_t *data, size_t length)
{
    TEST_CHECK(length <= sizeof(sent));
    memcpy(sent, data, length); sent_length = (uint16_t)length; ++usb_sends; return USB_ACCEPTED;
}
static void deliver_usb(size_t length)
{
    USBTransportOnReceive(usb_bytes, length);
    TEST_CHECK(USBTransportProcessRx() == USB_OK);
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

static size_t referee_frame(uint8_t *frame, uint16_t command, const uint8_t *payload, size_t length)
{
    memset(frame, 0, length+9u);
    frame[0]=0xa5u; frame[1]=(uint8_t)length; frame[2]=(uint8_t)(length>>8u); frame[3]=7u;
    frame[4]=(uint8_t)reference(frame,4u,0x8cu,0xffu);
    frame[5]=(uint8_t)command; frame[6]=(uint8_t)(command>>8u);
    memcpy(frame+7u,payload,length);
    uint16_t crc=(uint16_t)reference(frame,length+7u,0x8408u,0xffffu);
    frame[length+7u]=(uint8_t)crc; frame[length+8u]=(uint8_t)(crc>>8u);
    return length+9u;
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
    TEST_CHECK(USBTransportProcess() == USB_OK);
    USBTransportOnLink(USB_LINK_CONFIGURED, 1u);
    TEST_CHECK(USBTransportProcess() == USB_OK);
    Vision_Recv_s command = {0};
    command.head[0] = 'S'; command.head[1] = 'P';
    command.crc16 = (uint16_t)reference((const uint8_t *)&command, sizeof(command) - 2u, 0x8408u, 0xffffu);
    memcpy(usb_bytes, &command, sizeof(command));
    now_ms = 10u; deliver_usb(sizeof(command));
    Vision_Status_t status;
    VisionGetStatus(&status);
    TEST_CHECK(status.last_rx_ms == 10u && status.rx_count == 1u);
    uint32_t old_reloads = reloads;
    crc_fail_after = 0; now_ms = 11u; deliver_usb(sizeof(command));
    VisionGetStatus(&status);
    TEST_CHECK(reloads == old_reloads && status.last_rx_ms == 10u && status.rx_count == 1u);
    crc_fail_after = 0; VisionSend(); TEST_CHECK(usb_sends == 0u);
    VisionSend(); TEST_CHECK(usb_sends == 1u && sent_length == 43u);
    TEST_CHECK(reference(sent, 41u, 0x8408u, 0xffffu) == ((uint16_t)sent[41] | ((uint16_t)sent[42] << 8u)));
    USBTransportOnTxComplete();
    TEST_CHECK(USBTransportProcess() == USB_OK);

    // 连接通知未执行时，查询也必须立即撤销旧连接的命令。
    USBTransportOnLink(USB_LINK_RESET, 1u);
    VisionGetStatus(&status); TEST_CHECK(!status.has_valid_frame);
    USBTransportOnLink(USB_LINK_CONFIGURED, 1u);
    memcpy(usb_bytes,&command,sizeof(command)); now_ms=12u; deliver_usb(sizeof(command));
    // 新连接首帧先于普通任务通知到来，迟来的通知不能把新帧清掉。
    TEST_CHECK(USBTransportProcess()==USB_OK);
    VisionGetStatus(&status); TEST_CHECK(status.has_valid_frame && status.last_rx_ms==12u && status.rx_count==2u);
    command.yaw=NAN;
    command.crc16=(uint16_t)reference((const uint8_t *)&command,sizeof(command)-2u,0x8408u,0xffffu);
    memcpy(usb_bytes,&command,sizeof(command)); now_ms=13u; deliver_usb(sizeof(command));
    VisionGetStatus(&status); TEST_CHECK(status.last_rx_ms==12u && status.rx_count==2u);
    command.yaw=0.0f;
    command.crc16=(uint16_t)reference((const uint8_t *)&command,sizeof(command)-2u,0x8408u,0xffffu);
    memcpy(usb_bytes,&command,10u); now_ms=14u; deliver_usb(10u);
    memcpy(usb_bytes,((const uint8_t *)&command)+10u,sizeof(command)-10u); now_ms=15u;
    deliver_usb(sizeof(command)-10u);
    VisionGetStatus(&status); TEST_CHECK(status.last_rx_ms==14u && status.rx_count==3u);
    memcpy(usb_bytes,&command,sizeof(command)); memcpy(usb_bytes+sizeof(command),&command,sizeof(command));
    now_ms=16u; deliver_usb(2u*sizeof(command));
    VisionGetStatus(&status); TEST_CHECK(status.last_rx_ms==16u && status.rx_count==5u);

    UART_HandleTypeDef handle = {0};
    TEST_CHECK(RefereeInit(&handle, NULL) == REFEREE_OK);
    uint8_t payload[17]={1u};
    uint8_t frame[64];
    size_t frame_len=referee_frame(frame,ID_game_robot_state,payload,sizeof(payload));
    memcpy(uart_bytes, frame, frame_len);
    now_ms = 20u; deliver_uart(frame_len);
    RefereeSnapshot_t saved;
    TEST_CHECK(RefereeGetSnapshot(&saved)==REFEREE_OK);
    TEST_CHECK(saved.command[REF_CMD_ROBOT_STATE].source_rx_ms==20u &&
               saved.command[REF_CMD_ROBOT_STATE].data.robot_state.robot_id==1u);
    payload[0]=2u;
    frame_len=referee_frame(frame,ID_game_result,payload,1u);
    memcpy(uart_bytes,frame,frame_len); now_ms=21u; deliver_uart(frame_len);
    TEST_CHECK(RefereeGetSnapshot(&saved)==REFEREE_OK);
    TEST_CHECK(saved.command[REF_CMD_GAME_RESULT].state==REFEREE_DATA_EVENT_RECORDED &&
               saved.command[REF_CMD_GAME_RESULT].data.game_result.winner==2u);
    old_reloads = reloads;
    // 分别让帧头与整帧计算失败，都不能刷新时间或发布字段。
    for (int failure = 0; failure < 2; ++failure)
    {
        crc_fail_after = failure; now_ms = 22u; deliver_uart(frame_len);
        RefereeSnapshot_t after;
        TEST_CHECK(RefereeGetSnapshot(&after)==REFEREE_OK);
        TEST_CHECK(after.command[REF_CMD_GAME_RESULT].source_rx_ms==21u && reloads == old_reloads);
        TEST_CHECK(memcmp(&saved.command[REF_CMD_GAME_RESULT].data,
                          &after.command[REF_CMD_GAME_RESULT].data,
                          sizeof(saved.command[REF_CMD_GAME_RESULT].data)) == 0);
        TEST_CHECK(saved.command[REF_CMD_GAME_RESULT].receive_count ==
                   after.command[REF_CMD_GAME_RESULT].receive_count);
    }
    uart_bytes[7] ^= 1u; now_ms = 23u; deliver_uart(frame_len);
    RefereeSnapshot_t after;
    TEST_CHECK(RefereeGetSnapshot(&after)==REFEREE_OK &&
               after.command[REF_CMD_GAME_RESULT].source_rx_ms==21u && reloads == old_reloads);

    referee_id_t id = {.Robot_Color=Robot_Red,.Robot_ID = 1u, .Cilent_ID = 0x101u};
    Graph_Data_t graph = {0}; String_Data_t text = {0};
    UIDelete(&id, 1u, 0u,NULL); check_referee_frame();
    UIGraphRefresh(&id, &graph, 1u, 1u,NULL); check_referee_frame();
    TEST_CHECK(UICharDraw(&text, "crc", 1u, 0u, 0u, 15u, 2u, 0u, 0u, "CRC") == UI_OK);
    UICharRefresh(&id, &text,NULL); check_referee_frame();
    for (int failure = 0; failure < 2; ++failure)
    {
        uint32_t previous = uart_sends; uint8_t sequence = current_sequence();
        crc_fail_after = failure; UIDelete(&id, 1u, 0u,NULL);
        TEST_CHECK(uart_sends == previous && current_sequence() == sequence);
        crc_fail_after = failure; UIGraphRefresh(&id, &graph, 1u, 1u,NULL);
        TEST_CHECK(uart_sends == previous && current_sequence() == sequence);
        crc_fail_after = failure; UICharRefresh(&id, &text,NULL);
        TEST_CHECK(uart_sends == previous && current_sequence() == sequence);
    }
    return 0;
}
