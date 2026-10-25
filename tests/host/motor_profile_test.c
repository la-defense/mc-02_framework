/** @file motor_profile_test.c
 * @brief 检查生产电机任务仍推进接收，并遵循裸板与robot的输出隔离。
 * @note 只替换收发和驱动入口，不连接执行器；USB声明使用生产头文件。
 */
#include <stdio.h>
#include "motor_task.h"
#include "profile_io.h"
#include "daemon.h"
#include "usb_transport.h"
static unsigned can_pending, can_tx_pumps, uart_pending, usb_pending, dji_outputs, lk_outputs;
static unsigned daemon_dispatches;
void CANProcessRx(void) { if (can_pending) --can_pending; }
void CANProcessTx(void) { ++can_tx_pumps; }
void USARTProcessRx(void) { if (uart_pending) --uart_pending; }
USBStatus_t USBTransportProcessRx(void) { if (usb_pending) --usb_pending; return USB_OK; }
void DJIMotorControl(void) { ++dji_outputs; }
void LKMotorControl(void) { ++lk_outputs; }
DaemonStatus_t DaemonProcessTaskEvents(uint8_t consumer_id, uint8_t max_events, uint8_t *processed)
{
    if (processed != NULL)
        *processed = 0u;
    if (consumer_id != DAEMON_CONSUMER_MOTOR_TASK || max_events == 0u)
        return DAEMON_ERR_ARGUMENT;
    ++daemon_dispatches;
    return DAEMON_OK;
}
#define CHECK(c) do { if (!(c)) { fprintf(stderr, "FAIL line %d: %s\n", __LINE__, #c); return 1; } } while (0)
int main(void)
{
    can_pending = uart_pending = usb_pending = 2;
    MotorControlTask();
    MotorControlTask();
    CHECK(can_pending == 0 && uart_pending == 0 && usb_pending == 0);
    CHECK(can_tx_pumps == 2);
    CHECK(daemon_dispatches == 2);
#if defined(MC02_PROFILE_BENCH_SAFE)
    CHECK(dji_outputs == 0 && lk_outputs == 0);
#else
    CHECK(dji_outputs == 2 && lk_outputs == 2);
#endif
    puts("PASS: production motor task keeps receive consumers and respects profile isolation");
    return 0;
}
