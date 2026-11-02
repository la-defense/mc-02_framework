/** @file motor_profile_test.c
 * @brief 检查生产电机任务仍推进接收，并遵循裸板与robot的输出隔离。
 * @note 只替换收发和驱动入口，不连接执行器；USB声明使用生产头文件。
 */
#include <stdio.h>
#include <string.h>
#include "motor_task.h"
#include "profile_io.h"
#include "daemon.h"
#include "usb_transport.h"
#include "../../bsp/dwt/platform_probe.h"
static unsigned can_pending, can_tx_pumps, uart_pending, usb_pending, dji_outputs, lk_outputs;
static unsigned daemon_dispatches;
static char order[32]; // 保存两轮实际派发顺序，便于发现接收与控制的先后变化。
static unsigned order_count;
static void record(char event) { if (order_count < sizeof(order)) order[order_count++] = event; }
uint32_t PlatformProbeStart(void) { return 0u; } // 测试不访问硬件周期计数。
void PlatformProbeDone(volatile PlatformProbe_t *probe, uint32_t start) { (void)probe; (void)start; }
void CANProcessRx(void) { record('C'); if (can_pending) --can_pending; }
void CANProcessTx(void) { record('T'); ++can_tx_pumps; }
void USARTProcessRx(void) { record('U'); if (uart_pending) --uart_pending; }
USBStatus_t USBTransportProcessRx(void) { record('B'); if (usb_pending) --usb_pending; return USB_OK; }
void DJIMotorControl(void) { record('D'); ++dji_outputs; }
void LKMotorControl(void) { record('L'); ++lk_outputs; }
void RobotCanBridgeService(void) { record('S'); } // 双板在 CAN 接收之后检查同步与话题。
void RobotCanBridgeServiceTx(void) { record('X'); } // 常规 CAN 发送之后推进桥接总线。
DaemonStatus_t DaemonProcessTaskEvents(uint8_t consumer_id, uint8_t max_events, uint8_t *processed)
{
    if (processed != NULL)
        *processed = 0u;
    if (consumer_id != DAEMON_CONSUMER_MOTOR_TASK || max_events == 0u)
        return DAEMON_ERR_ARGUMENT;
    record('E'); // 串口派发后才处理离线恢复事件。
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
    CHECK(order_count == 10u && memcmp(order, "CUBETCUBET", 10u) == 0); // 裸板无电机计算。
#else
    CHECK(dji_outputs == 2 && lk_outputs == 2);
#if defined(CHASSIS_BOARD)
    CHECK(order_count == 18u && memcmp(order, "CSUBEDLTXCSUBEDLTX", 18u) == 0); // 双板服务保持原有位置。
#else
    CHECK(order_count == 14u && memcmp(order, "CUBEDLTCUBEDLT", 14u) == 0); // 每轮先接收再控制和发送。
#endif
#endif
    puts("PASS: production motor task keeps receive consumers and respects profile isolation");
    return 0;
}
