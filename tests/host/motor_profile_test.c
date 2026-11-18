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
#if defined(MC02_CONFIG_GENERATED)
#include "../../application/config/vehicle_selection.h"
static uint8_t communication_started = 1u; // 模拟启动协调器发布的模块状态。
uint8_t RobotBuildModuleStarted(RobotModule_t module)
{
    return (uint8_t)(module == ROBOT_MODULE_COMMUNICATION && communication_started);
}
#endif
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

static int CheckSplitPeriods(void)
{
    MotorTaskSchedule_t schedule;
    MotorTaskPeriods_t periods = {
        .compute_us = 2000u, // 主机试验用 2 ms 计算拍点。
        .transmit_us = 3000u, // 主机试验用 3 ms 发送拍点。
    };
    const uint32_t start = UINT32_MAX - 1u; // 在测试过程中跨越计数回绕。
    CHECK(MotorTaskScheduleInit(&schedule, &periods, 1000u, 1000u, start) == PLATFORM_PERIODIC_READY);
    periods.transmit_us = 5000u; // 超出队列生存期应拒绝且保留已经建立的计划。
    CHECK(MotorTaskScheduleInit(&schedule, &periods, 1000u, 1000u, start) == PLATFORM_PERIODIC_CONFIG);
    CHECK(schedule.transmit.period_ticks == 3u);
    periods.transmit_us = 2500u; // 发送必须与最低轮询节拍对齐。
    CHECK(MotorTaskScheduleInit(&schedule, &periods, 1000u, 1000u, start) == PLATFORM_PERIODIC_CONFIG);
    periods.transmit_us = 3000u;
    periods.compute_us = 51000u; // 电机的 PID 计时会拒绝这样长的正常计算间隔。
    CHECK(MotorTaskScheduleInit(&schedule, &periods, 1000u, 1000u, start) == PLATFORM_PERIODIC_CONFIG);

    const uint32_t release_offsets[] = {0u, 1u, 2u, 3u, 6u}; // 首拍、仅接收、仅计算、仅发送、迟到后同拍。
    dji_outputs = lk_outputs = can_tx_pumps = daemon_dispatches = 0u;
#if defined(MC02_CONFIG_GENERATED)
    communication_started = 1u; // 使用正常启动状态验证发送和接收仍串行。
#endif
    for (unsigned i = 0u; i < sizeof(release_offsets) / sizeof(release_offsets[0]); ++i)
    {
        order_count = 0u;
        CHECK(MotorTaskRunScheduled(&schedule, start + release_offsets[i]) == PLATFORM_PERIODIC_READY);
        // 任意拍先接收 CAN，最后才允许已有的发送端调用。
        CHECK(order_count >= 4u && order[0] == 'C');
        const char *compute = memchr(order, 'D', order_count);
        const char *transmit = memchr(order, 'T', order_count);
#if defined(MC02_PROFILE_BENCH_SAFE)
        CHECK(compute == NULL);
#else
        CHECK((compute != NULL) == (i == 0u || i == 2u || i == 4u));
        if (compute != NULL && transmit != NULL) CHECK(compute < transmit);
#endif
        CHECK((transmit != NULL) == (i == 0u || i == 3u || i == 4u));
    }
    CHECK(can_tx_pumps == 3u && daemon_dispatches == 5u); // 跳过未执行的旧拍，接收在每个轮询运行一次。
#if !defined(MC02_PROFILE_BENCH_SAFE)
    CHECK(dji_outputs == 3u && lk_outputs == 3u);
#endif
    order_count = 0u;
    CHECK(MotorTaskRunScheduled(&schedule, start + 5u) == PLATFORM_PERIODIC_CLOCK); // 倒退的时刻不能触发任何访问。
    CHECK(order_count == 0u && schedule.compute.last_tick == start + 6u);
    return 0;
}

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
#if defined(MC02_CONFIG_GENERATED)
    communication_started = 0u; // 模拟启动诊断后辅助通信被标记失败。
    order_count = 0u; // 单独核对降级后的调用顺序。
    MotorControlTask();
    CHECK(order_count == 7u && memcmp(order, "CUBEDLT", 7u) == 0); // 接收与电机处理继续，桥接不再调用。
#endif
#else
    CHECK(order_count == 14u && memcmp(order, "CUBEDLTCUBEDLT", 14u) == 0); // 每轮先接收再控制和发送。
#endif
#endif
    CHECK(CheckSplitPeriods() == 0);
    puts("PASS: production motor task keeps receive consumers and respects profile isolation");
    return 0;
}
