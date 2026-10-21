/** @file uart_tx_callers_test.c
 * @brief 生产舵机、HC05和备用UART视觉发送，经真实队列及H723接入验证复制和结果。
 */
#include "uart_tx_hal_model.h"
#include "servo_motor.h"
#include "HC05.h"
#include "master_process.h"
#include "daemon.h"
#include "test_check.h"
#include <math.h>
#include <string.h>
static DaemonInstance daemon_instance;
DaemonInstance *DaemonRegister(Daemon_Init_Config_s *config)
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
uint8_t DaemonIsOnline(DaemonInstance *instance)
{
    (void)instance;
    return 1u;
}
PWMInstance *PWMRegister(PWM_Init_Config_s *config)
{
    (void)config;
    return NULL;
}
void PWMSetDutyRatio(PWMInstance *instance, float duty)
{
    instance->duty = duty;
}
void osDelay(uint32_t ms)
{
    (void)ms;
    TEST_CHECK(0);
}
static void rx(void *context, const USART_RxBlock_t *block)
{
    (void)context;
    (void)block;
}
int main(int argc, char **argv)
{
    TEST_CHECK(argc == 2);
    UARTModelInit();
    USARTTxSetServiceReady(1u);
    if (strcmp(argv[1], "servo") == 0)
    {
        USART_Init_Config_s config = {
            .recv_buff_size = 10u, .usart_handle = &uart_handles[5], .module_callback = rx};
        USARTInstance *instance = USARTRegister(&config);
        TEST_CHECK(instance);
        UARTModelCheckRuntimeAllocation();
        ServoInstance servo = {.servo_type = Bus_Servo, .usart_instance = instance};
        TEST_CHECK(ServoSetAngle(NULL, 1.0f) == USART_SEND_BAD_ARGUMENT);
        TEST_CHECK(ServoSetAngle(&servo, NAN) == USART_SEND_BAD_ARGUMENT &&
                   ServoSetAngle(&servo, 65536.0f) == USART_SEND_BAD_ARGUMENT);
        TEST_CHECK(ServoSetAngle(&servo, 1000.0f) == USART_SEND_ACCEPTED);
        USARTProcessTx();
        const uint8_t golden[] = {0x55, 0x55, 8, 3, 1, 0xf4, 1, 1, 0xe8, 3, 0x55, 0x55, 4, 0x15, 1, 1};
        TEST_CHECK(uart_model_tx_length[5] == 16u && memcmp(uart_model_tx[5], golden, sizeof(golden)) == 0);
        TEST_CHECK(ServoSetAngle(&servo, 2.0f) == USART_SEND_ACCEPTED);
        TEST_CHECK(memcmp(uart_model_tx[5], golden, sizeof(golden)) == 0);
        UARTModelDMAComplete(5u);
        UARTModelComplete(5u);
        USARTProcessTx();
        USARTTxEvent_t latest;
        uint32_t failures;
        TEST_CHECK(ServoGetTxState(&servo, &latest, &failures) == USART_SEND_ACCEPTED);
        TEST_CHECK(latest.result == USART_TX_COMPLETE && latest.id != 0u && failures == 0u);
        TEST_CHECK(uart_model_tx[5][8] == 2u);
        uart_model_ms = 50u;
        USARTProcessTx();
        TEST_CHECK(ServoGetTxState(&servo, &latest, &failures) == USART_SEND_ACCEPTED);
        TEST_CHECK(failures == 1u && latest.result == USART_TX_EXPIRED);
        return 0;
    }
    if (strcmp(argv[1], "hc05") == 0)
    {
        uint8_t bytes[4] = {0x11, 0x22, 0x33, 0x44};
        TEST_CHECK(HC05_SendData(bytes, 4u) == USART_SEND_NOT_READY);
        HC05 *message = HC05Init(&uart_handles[4]);
        TEST_CHECK(message);
        UARTModelCheckRuntimeAllocation();
        TEST_CHECK(HC05_SendData(bytes, 5u) == USART_SEND_BAD_ARGUMENT &&
                   HC05_SendData(NULL, 4u) == USART_SEND_BAD_ARGUMENT);
        TEST_CHECK(HC05_SendData(bytes, 4u) == USART_SEND_ACCEPTED);
        memset(bytes, 0xee, sizeof(bytes));
        USARTProcessTx();
        const uint8_t golden[] = {0xaa, 0x11, 0x22, 0x33, 0x44, 0x55};
        TEST_CHECK(uart_model_tx_length[4] == 6u && memcmp(uart_model_tx[4], golden, 6u) == 0);
        UARTModelComplete(4u);
        USARTProcessTx();
        USARTTxEvent_t latest;
        uint32_t failures;
        TEST_CHECK(HC05GetTxState(&latest, &failures) == USART_SEND_ACCEPTED);
        TEST_CHECK(latest.result == USART_TX_COMPLETE && latest.id != 0u && failures == 0u);
        TEST_CHECK(HC05_SendData(bytes, 4u) == USART_SEND_ACCEPTED);
        USARTProcessTx();
        uart_model_ms = 50u;
        USARTProcessTx();
        TEST_CHECK(HC05GetTxState(&latest, &failures) == USART_SEND_ACCEPTED);
        TEST_CHECK(failures == 1u && latest.result == USART_TX_EXPIRED && uart_model_leases == 0u);
        return 0;
    }
    TEST_CHECK(VisionInit(&uart_handles[6]) != NULL);
    UARTModelCheckRuntimeAllocation();
    VisionUpdateTx(2u, 1.0f, 0.0f, 0.0f, 0.0f, 0.5f, -0.25f, 1.5f, -2.0f, 30.0f, 0x1234u);
    VisionSend();
    USARTProcessTx();
    // 独立用小端IEEE754字段及0x8408逐位算法核对，固定41字节内容和两字节CRC。
    const uint8_t golden[] = {0x53, 0x50, 0x02, 0x00, 0x00, 0x80, 0x3f, 0x00, 0x00, 0x00, 0x00,
                              0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                              0x3f, 0x00, 0x00, 0x80, 0xbe, 0x00, 0x00, 0xc0, 0x3f, 0x00, 0x00,
                              0x00, 0xc0, 0x00, 0x00, 0xf0, 0x41, 0x34, 0x12, 0x73, 0x54};
    TEST_CHECK(uart_model_tx_length[6] == sizeof(golden) &&
               memcmp(uart_model_tx[6], golden, sizeof(golden)) == 0);
    uint8_t saved[43];
    memcpy(saved, uart_model_tx[6], 43u);
    VisionUpdateTx(0u, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0u);
    VisionSend();
    TEST_CHECK(memcmp(saved, uart_model_tx[6], 43u) == 0);
    UARTModelDMAComplete(6u);
    UARTModelComplete(6u);
    USARTProcessTx();
    Vision_Status_t state;
    VisionGetStatus(&state);
    TEST_CHECK(state.tx_count == 2u && state.uart_last_tx.result == USART_TX_COMPLETE);
    uart_model_ms = 20u;
    USARTProcessTx();
    VisionGetStatus(&state);
    TEST_CHECK(state.tx_drop_count == 1u && state.uart_last_tx.result == USART_TX_EXPIRED &&
               uart_model_leases == 0u);
    return 0;
}
