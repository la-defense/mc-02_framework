/** @file referee_budget_test.c
 * @brief 经生产裁判、CRC、队列和H723后端验证共享额度；HAL与时间使用主机模型。
 * @note CTest每场景独立进程；不连接硬件，Release检查仍执行。
 */
#include "protocol_uart_binding_fake.h"
#include "bsp_usart.h"
#include "referee_task.h"
#include "referee_tx_private.h"
#include "uart_tx_hal_model.h"
#include "test_check.h"
#include "crc.h"
#include "daemon.h"
#include <limits.h>
#include <pthread.h>
#include <string.h>
static DaemonInstance daemon_instance;
static const RefereeUIFieldConfig_t ui_fields[] = {
    {{'h', 'p', '0'}, "HP:", 120u, 195u, 20u, 2u, UI_Color_Green,
     REFEREE_UI_SOURCE_ROBOT_HP},
};
static const RefereeUIConfig_t ui_config = {
    .fields = ui_fields,
    .field_count = (uint8_t)(sizeof(ui_fields) / sizeof(ui_fields[0])),
    .frame_interval_ms = 115u,
};

DaemonInstance *DaemonRegister(const Daemon_Init_Config_s *config)
{
    (void)config;
    return &daemon_instance;
}
uint8_t DaemonUnregister(DaemonInstance *instance)
{
    (void)instance;
    return 1u;
}
void DaemonReload(DaemonInstance *instance)
{
    (void)instance;
}
DaemonStatus_t DaemonReportAt(DaemonInstance *instance, uint32_t received_ms, uint8_t flags)
{
    (void)instance;
    (void)received_ms;
    (void)flags;
    return DAEMON_OK;
}
uint8_t DaemonIsOnline(DaemonInstance *instance)
{
    (void)instance;
    return 1u;
}
static void identity(uint8_t robot)
{
    uint8_t frame[26] = {0xa5u, 17u, 0u, 0u, 0u, 1u, 2u};
    frame[7] = robot;
    uint32_t crc;
    TEST_CHECK(CRCCompute(CRC8_REFEREE, frame, 4u, &crc) == CRC_OK);
    frame[4] = (uint8_t)crc;
    TEST_CHECK(CRCCompute(CRC16_SP_REFEREE, frame, 24u, &crc) == CRC_OK);
    frame[24] = (uint8_t)crc;
    frame[25] = (uint8_t)(crc >> 8u);
    memcpy(uart_model_rx[0], frame, sizeof(frame));
    HAL_UARTEx_RxEventCallback(&uart_handles[0], sizeof(frame));
    USARTProcessRx();
}

static USARTTxEvent_t events[300];
static unsigned notifications;
static void done(void *context, const USARTTxEvent_t *event)
{
    (void)context;
    TEST_CHECK(mc02_test_ipsr == 0u && mc02_test_primask == 0u && notifications < 300u);
    events[notifications++] = *event;
}
static RefereeTxState_t tx_state(void)
{
    RefereeTxState_t s;
    TEST_CHECK(RefereeGetTxState(&s) == USART_SEND_ACCEPTED);
    return s;
}
static void complete(void)
{
    UARTModelDMAComplete(0u);
    UARTModelComplete(0u);
    USARTProcessTx();
}
static uint32_t submit(uint16_t receiver, size_t length, uint32_t ttl)
{
    uint8_t data[112];
    memset(data, 0x69, sizeof(data));
    RefereeTxRequest_t r = {receiver, 0x0200u, data, length, ttl, done, NULL, NULL};
    uint32_t id = 0u;
    TEST_CHECK(RefereeSubmit(&r, &id) == REFEREE_SEND_ACCEPTED && id != 0u);
    memset(data, 0xee, sizeof(data));
    return id;
}
static void advance(uint32_t ms)
{
    uart_model_ms = ms;
    identity(3u);
}
static void *producer(void *context)
{
    uint8_t content = (uint8_t)(uintptr_t)context;
    uint32_t id = 0u;
    RefereeTxRequest_t r = {4u, 0x0200u, &content, 1u, 200u, done, NULL, NULL};
    RefereeSendStatus_t s = RefereeSubmit(&r, &id);
    TEST_CHECK(s == REFEREE_SEND_ACCEPTED || s == REFEREE_SEND_BUSY);
    TEST_CHECK((s == REFEREE_SEND_ACCEPTED) == (id != 0u));
    return NULL;
}
static void ignored_rx(void *context, const USART_RxBlock_t *block)
{
    (void)context;
    (void)block;
}
int main(int argc, char **argv)
{
    TEST_CHECK(argc == 2);
    UARTModelInit();
    TestRefereeUartBind(&uart_handles[0]);
    USARTTxSetServiceReady(1u);
    // 测试用掩码只用于检查配置机制，不代表任一赛项官方允许表。
    RefereeConfig_t config = { .timeout_10hz_ms=300u, .timeout_3hz_ms=1000u,
        .timeout_1hz_ms=3000u, .timeout_map_ms=3000u,
        .interaction = { REFEREE_COMPETITION_RMUC_2026, REFEREE_PROTOCOL_2026_V2, 3u, 0x01ffu, 0x01ffu } };
    if (strcmp(argv[1], "policy") == 0)
    {
        RefereeConfig_t bad = config;
        bad.interaction.competition = (RefereeCompetition_t)99;
        TEST_CHECK(RefereeInit(PLATFORM_UART_1, &bad) == REFEREE_ERROR_CONFIG);
        bad = config; bad.interaction.local_role = 0u;
        TEST_CHECK(RefereeInit(PLATFORM_UART_1, &bad) == REFEREE_ERROR_CONFIG);
        bad = config; bad.interaction.protocol = (RefereeInteractionProtocol_t)99;
        TEST_CHECK(RefereeInit(PLATFORM_UART_1, &bad) == REFEREE_ERROR_CONFIG);
        bad = config; bad.interaction.tx_roles = 0x0200u;
        TEST_CHECK(RefereeInit(PLATFORM_UART_1, &bad) == REFEREE_ERROR_CONFIG);
        bad = config; bad.interaction.competition = REFEREE_COMPETITION_NONE;
        TEST_CHECK(RefereeInit(PLATFORM_UART_1, &bad) == REFEREE_ERROR_CONFIG);
        config.interaction.competition = REFEREE_COMPETITION_RMUL_2026;
        config.interaction.tx_roles = 1u << 3u;
    }
    TEST_CHECK(RefereeInit(PLATFORM_UART_1, strcmp(argv[1], "no-policy") == 0 ? NULL : &config) == REFEREE_OK);
    identity(3u);
    if (strcmp(argv[1], "policy") == 0)
    {
        uint8_t data = 0u;
        uint32_t id = 77u;
        RefereeTxRequest_t request = {5u, 0x0200u, &data, 1u, 100u, done, NULL, NULL};
        config.interaction.tx_roles = 0x01ffu; // 修改原配置不能放宽运行中的权限。
        TEST_CHECK(RefereeSubmit(&request, &id) == REFEREE_SEND_IDENTITY && id == 77u);
        request.receiver_id = 4u;
        TEST_CHECK(RefereeSubmit(&request, &id) == REFEREE_SEND_ACCEPTED);
        USARTProcessTx(); complete();
        identity(2u); id = 77u;
        TEST_CHECK(RefereeSubmit(&request, &id) == REFEREE_SEND_IDENTITY && id == 77u);
        identity(103u); request.receiver_id = 104u;
        TEST_CHECK(RefereeSubmit(&request, &id) == REFEREE_SEND_ACCEPTED);
        return 0;
    }
    if (strcmp(argv[1], "no-policy") == 0)
    {
        uint8_t data = 0u;
        uint32_t id = 77u;
        RefereeTxRequest_t request = {4u, 0x0200u, &data, 1u, 100u, done, NULL, NULL};
        TEST_CHECK(RefereeSubmit(&request, &id) == REFEREE_SEND_IDENTITY && id == 77u);
        uint8_t frame[15] = {0xa5u, 6u, 0u, 1u, 0u, 1u, 3u, 0u, 2u, 4u, 0u, 3u, 0u, 0u, 0u};
        uint32_t crc;
        TEST_CHECK(CRCCompute(CRC8_REFEREE, frame, 4u, &crc) == CRC_OK);
        frame[4] = (uint8_t)crc;
        TEST_CHECK(CRCCompute(CRC16_SP_REFEREE, frame, 13u, &crc) == CRC_OK);
        frame[13] = (uint8_t)crc; frame[14] = (uint8_t)(crc >> 8u);
        memcpy(uart_model_rx[0], frame, sizeof(frame));
        HAL_UARTEx_RxEventCallback(&uart_handles[0], sizeof(frame));
        USARTProcessRx();
        RefereeInteractionRecord_t record;
        TEST_CHECK(RefereeGetInteraction(&record) == REFEREE_OK && !record.received);
        return 0;
    }
    USARTInstance *other = NULL;
    if (strcmp(argv[1], "busy-isolate") == 0)
    {
        USART_Init_Config_s conf = {
            .usart_handle = &uart_handles[1], .recv_buff_size = 16u, .module_callback = ignored_rx};
        other = USARTRegister(&conf);
        TEST_CHECK(other != NULL);
    }
    UARTModelCheckRuntimeAllocation();
    if (strcmp(argv[1], "fixed-interaction") == 0)
    {
        const uint8_t data[] = {0x11u, 0x22u, 0x33u};
        const uint8_t expected[] = {0xa5u,0x09u,0x00u,0x00u,0x4du,0x01u,0x03u,0x34u,0x02u,
            0x03u,0x00u,0x01u,0x00u,0x11u,0x22u,0x33u,0xe0u,0x2fu};
        RefereeTxRequest_t request = {1u, 0x0234u, data, sizeof(data), 100u, done, NULL, NULL};
        uint32_t id = 0u;
        TEST_CHECK(RefereeSubmit(&request, &id) == REFEREE_SEND_ACCEPTED && id != 0u);
        USARTProcessTx();
        TEST_CHECK(uart_model_tx_length[0] == sizeof(expected) &&
                   memcmp(uart_model_tx[0], expected, sizeof(expected)) == 0);
        complete();
        TEST_CHECK(notifications == 1u && events[0].result == USART_TX_COMPLETE);
        return 0;
    }
    if (strcmp(argv[1], "arguments") == 0)
    {
        uint8_t bytes[113] = {0};
        uint32_t id = 0x1234u;
        RefereeTxRequest_t r = {4u, 0x0200u, bytes, 112u, 100u, done, NULL, NULL};
        TEST_CHECK(RefereeSubmit(NULL, &id) == REFEREE_SEND_BAD_ARGUMENT);
        TEST_CHECK(RefereeSubmit(&r, NULL) == REFEREE_SEND_BAD_ARGUMENT);
        r.length = 113u;
        TEST_CHECK(RefereeSubmit(&r, &id) == REFEREE_SEND_BAD_ARGUMENT);
        r.length = 112u;
        r.receiver_id = 104u;
        TEST_CHECK(RefereeSubmit(&r, &id) == REFEREE_SEND_IDENTITY);
        r.receiver_id = 4u;
        r.content_id = 0x0120u;
        TEST_CHECK(RefereeSubmit(&r, &id) == REFEREE_SEND_BAD_ARGUMENT);
        r.content_id = 0x0200u;
        r.timeout_ms = 0u;
        TEST_CHECK(RefereeSubmit(&r, &id) == REFEREE_SEND_BAD_ARGUMENT);
        r.timeout_ms = UINT32_MAX;
        TEST_CHECK(RefereeSubmit(&r, &id) == REFEREE_SEND_BAD_ARGUMENT);
        r.timeout_ms = 100u;
        r.content = NULL;
        TEST_CHECK(RefereeSubmit(&r, &id) == REFEREE_SEND_BAD_ARGUMENT);
        r.content = bytes;
        r.complete = NULL;
        TEST_CHECK(RefereeSubmit(&r, &id) == REFEREE_SEND_BAD_ARGUMENT);
        r.complete = done;
        TEST_CHECK(id == 0x1234u && tx_state().next_sequence == 0u && uart_model_calls == 0u);
        TEST_CHECK(RefereeSubmit(&r, &id) == REFEREE_SEND_ACCEPTED);
        memset(bytes, 0xee, sizeof(bytes));
        USARTProcessTx();
        TEST_CHECK(uart_model_tx_length[0] == 127u && uart_model_tx[0][13] == 0u);
        complete();
        TEST_CHECK(notifications == 1u && events[0].result == USART_TX_COMPLETE);
        return 0;
    }
    if (strcmp(argv[1], "fifo") == 0 || strcmp(argv[1], "threads") == 0)
    {
        uint32_t ids[4];
        if (strcmp(argv[1], "threads") == 0)
        {
            pthread_t threads[4];
            for (uintptr_t i = 0u; i < 4u; ++i)
                TEST_CHECK(pthread_create(&threads[i], NULL, producer, (void *)i) == 0);
            for (unsigned i = 0u; i < 4u; ++i)
                TEST_CHECK(pthread_join(threads[i], NULL) == 0);
        }
        else
        {
            for (unsigned i = 0u; i < 4u; ++i)
                ids[i] = submit(4u, 1u, 200u);
            uint32_t id = 99u;
            uint8_t data = 0;
            RefereeTxRequest_t r = {4u, 0x0200u, &data, 1u, 200u, done, NULL, NULL};
            TEST_CHECK(RefereeSubmit(&r, &id) == REFEREE_SEND_BUSY && id == 99u);
        }
        unsigned accepted = tx_state().uart.used;
        if (strcmp(argv[1], "threads") == 0)
            TEST_CHECK(tx_state().rejected == 4u - accepted);
        TEST_CHECK(accepted >= 1u && accepted <= 4u);
        for (unsigned i = 0u; i < accepted; ++i)
        {
            advance(i * 34u);
            USARTProcessTx();
            TEST_CHECK(uart_model_calls == i + 1u);
            TEST_CHECK(uart_model_tx[0][3] == i);
            complete();
            if (strcmp(argv[1], "fifo") == 0)
                TEST_CHECK(events[i].id == ids[i]);
            if (i + 1u < accepted)
            {
                advance(i * 34u + 33u);
                USARTProcessTx();
                TEST_CHECK(uart_model_calls == i + 1u);
            }
        }
        TEST_CHECK(notifications == accepted && uart_model_leases == 0u);
        return 0;
    }
    if (strcmp(argv[1], "ui-expire") == 0)
    {
        submit(4u, 112u, 100u);
        USARTProcessTx();
        complete();
        TEST_CHECK(UITaskInit(&ui_config) == UI_OK && MyUIInit() == UI_SUBMITTED && UIRefresh() == UI_SUBMITTED);
        USARTProcessTx();
        TEST_CHECK(uart_model_calls == 1u && tx_state().wait_reason == REFEREE_TX_WAIT_RATE);
        advance(100u);
        USARTProcessTx();
        TEST_CHECK(UIRefresh() == UI_SUBMITTED);
        UIState_t s;
        TEST_CHECK(UIGetState(&s) == UI_OK);
        TEST_CHECK(s.init_step == 0u && s.phase == UI_PHASE_INITIALIZING);
        USARTProcessTx();
        TEST_CHECK(uart_model_calls == 2u);
        complete();
        TEST_CHECK(UIRefresh() == UI_OK && UIGetState(&s) == UI_OK && s.init_step == 1u);
        return 0;
    }
    if (strcmp(argv[1], "window") == 0 || strcmp(argv[1], "radar") == 0 || strcmp(argv[1], "receiver") == 0)
    {
        uint16_t receiver = strcmp(argv[1], "radar") == 0 ? 7u : 4u;
        for (unsigned i = 0u; i < 29u; ++i)
        {
            advance(i * 34u);
            submit(receiver, 112u, 100u);
            USARTProcessTx();
            TEST_CHECK(uart_model_calls == i + 1u && uart_model_tx[0][13] == 0x69u);
            complete();
        }
        advance(986u);
        submit(strcmp(argv[1], "receiver") == 0 ? 5u : receiver, 112u, 100u);
        USARTProcessTx();
        if (receiver == 7u || strcmp(argv[1], "receiver") == 0)
        {
            TEST_CHECK(uart_model_calls == 30u);
            complete();
            return 0;
        }
        TEST_CHECK(uart_model_calls == 29u && tx_state().wait_reason == REFEREE_TX_WAIT_BYTES);
        advance(999u);
        USARTProcessTx();
        TEST_CHECK(uart_model_calls == 29u);
        advance(1000u);
        USARTProcessTx();
        TEST_CHECK(uart_model_calls == 30u);
        complete();
        return 0;
    }
    if (strcmp(argv[1], "sequence") == 0)
    {
        for (unsigned i = 0u; i < 257u; ++i)
        {
            advance(i * 34u);
            submit(4u, 0u, 100u);
            USARTProcessTx();
            TEST_CHECK(uart_model_tx[0][3] == (uint8_t)i);
            complete();
        }
        TEST_CHECK(tx_state().next_sequence == 1u && notifications == 257u);
        return 0;
    }
    if (strcmp(argv[1], "cancel-expire") == 0)
    {
        uint32_t cancelled = submit(4u, 112u, 100u);
        TEST_CHECK(RefereeCancelSend(cancelled) == USART_SEND_ACCEPTED);
        USARTProcessTx();
        TEST_CHECK(tx_state().has_start == 0u && notifications == 1u);
        submit(4u, 112u, 1u);
        advance(1u);
        USARTProcessTx();
        TEST_CHECK(tx_state().has_start == 0u && notifications == 2u && uart_model_calls == 0u);
        submit(4u, 112u, 100u);
        USARTProcessTx();
        complete();
        TEST_CHECK(tx_state().cancelled == 1u && tx_state().expired == 1u && uart_model_calls == 1u);
        return 0;
    }
    if (strcmp(argv[1], "busy-isolate") == 0)
    {
        uart_model_start_result = HAL_BUSY;
        submit(4u, 112u, 100u);
        USARTProcessTx();
        TEST_CHECK(tx_state().has_start == 0u && notifications == 0u);
        uart_model_start_result = HAL_OK;
        advance(1u);
        USARTProcessTx();
        UARTModelDMAComplete(0u);
        USARTProcessTx();
        TEST_CHECK(notifications == 0u);
        // 在线端完成丢失后按总期限停止；停止失败保留本路缓冲和lease。
        uart_model_abort_result = HAL_ERROR;
        advance(100u);
        USARTProcessTx();
        TEST_CHECK(tx_state().uart.isolated && tx_state().errors == 1u && notifications == 1u);
        TEST_CHECK(events[0].started && events[0].result == USART_TX_ISOLATED);
        uint32_t id = 99u;
        uint8_t byte = 0u;
        RefereeTxRequest_t r = {4u, 0x0200u, &byte, 1u, 100u, done, NULL, NULL};
        TEST_CHECK(RefereeSubmit(&r, &id) == REFEREE_SEND_ISOLATED && id == 99u);
        USARTTxRequest_t request = {&byte, 1u, USART_TRANSFER_DMA, 100u, done, NULL, NULL};
        TEST_CHECK(USARTSubmitTx(other, &request, &id) == USART_SEND_ACCEPTED);
        USARTProcessTx();
        UARTModelDMAComplete(1u);
        UARTModelComplete(1u);
        USARTProcessTx();
        TEST_CHECK(notifications == 2u && events[1].result == USART_TX_COMPLETE);
        return 0;
    }
    if (strcmp(argv[1], "errors") == 0)
    {
        uart_model_start_result = HAL_ERROR;
        submit(4u, 0u, 100u);
        USARTProcessTx();
        USARTProcessTx();
        TEST_CHECK(tx_state().errors == 1u && uart_model_calls == 1u);
        uart_model_start_result = HAL_OK;
        submit(4u, 0u, 100u);
        USARTProcessTx();
        TEST_CHECK(uart_model_calls == 1u);
        advance(34u);
        USARTProcessTx();
        TEST_CHECK(uart_model_calls == 2u);
        complete();
        return 0;
    }
    TEST_CHECK(strcmp(argv[1], "wrap-identity") == 0);
    advance(UINT32_MAX - 20u);
    submit(4u, 0u, 100u);
    USARTProcessTx();
    complete();
    advance(12u);
    identity(103u);
    submit(104u, 0u, 100u);
    USARTProcessTx();
    TEST_CHECK(uart_model_calls == 1u);
    uart_model_ms = 13u;
    identity(103u);
    USARTProcessTx();
    TEST_CHECK(uart_model_calls == 2u);
    complete();
    advance(314u);
    uint8_t byte = 0;
    uint32_t id = 99u;
    // 本组停更超过300ms，拒绝不能清空已经记录的预算。
    uart_model_ms = 615u;
    RefereeTxRequest_t r = {4u, 0x0200u, &byte, 1u, 100u, done, NULL, NULL};
    TEST_CHECK(RefereeSubmit(&r, &id) == REFEREE_SEND_IDENTITY && id == 99u);
    return 0;
}
