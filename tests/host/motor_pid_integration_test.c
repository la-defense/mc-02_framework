/** @file motor_pid_integration_test.c
 * @brief 运行生产 DJI 串级 PID，检查首轮、实际间隔、失败零输出和初始化传播。
 * CAN 和时间由替身提供；没有连接或驱动真实电机。
 */
#include "dji_motor.h"
#include "bsp_dwt.h"
#include "test_check.h"
#include <math.h>
#include <string.h>

CAN_HandleTypeDef hcan1, hcan2;
volatile uint32_t dwt_cpu_freq_mhz = 480u;
static uint64_t now_us;
static unsigned clock_reads, registered, submitted;
static CANInstance can_instances[4];
static DaemonInstance daemon;
static uint8_t sent[8];

uint64_t DWT_GetTimeline_us(void) { ++clock_reads; return now_us; }
uint32_t HAL_GetTick(void) { return (uint32_t)(now_us / 1000u); }
float DWT_GetDeltaT(uint32_t *count) { ++*count; return 0.001f; }
uint32_t DWT_ProbeStart(void) { return 0u; }
void DWT_ProbeDone(volatile DWT_Probe_t *probe, uint32_t start) { (void)probe; (void)start; }
DaemonInstance *DaemonRegister(const Daemon_Init_Config_s *config) { (void)config; return &daemon; }
uint8_t DaemonUnregister(DaemonInstance *instance) { (void)instance; return 1u; }
void DaemonReload(DaemonInstance *instance) { (void)instance; }
DaemonStatus_t DaemonReportAt(DaemonInstance *instance, uint32_t received_ms, uint8_t flags)
{ (void)instance; (void)received_ms; (void)flags; return DAEMON_OK; }
uint8_t DaemonIsOnline(DaemonInstance *instance) { (void)instance; return 1u; }
CANInstance *CANRegister(CAN_Init_Config_s *config)
{
    TEST_CHECK(registered < 4u);
    CANInstance *instance = &can_instances[registered++];
    instance->can_handle = config->can_handle;
    instance->rx_id = config->rx_id;
    instance->id = config->id;
    instance->can_module_callback = config->can_module_callback;
    return instance;
}
CAN_TxSubmitStatus_t CANSubmitPeriodic(FDCAN_HandleTypeDef *hcan, const void *source,
                                       uint32_t key, const CAN_TxFrame_t *frame,
                                       CAN_TxPriority_t priority, uint32_t deadline_ms,
                                       uint32_t period_us, uint32_t *request_id)
{
    (void)hcan; (void)source; (void)key; (void)priority; (void)deadline_ms; (void)period_us;
    TEST_CHECK(frame != NULL && request_id != NULL && frame->length <= sizeof(sent));
    ++submitted;
    memset(sent, 0, sizeof(sent));
    memcpy(sent, frame->data, frame->length);
    *request_id = submitted;
    return CAN_TX_SUBMIT_ACCEPTED;
}
// 本测试没有发送前许可回调，复用原有的周期帧捕获。
CAN_TxSubmitStatus_t CANSubmitPeriodicPrepared(FDCAN_HandleTypeDef *hcan, const void *source,
    uint32_t key, const CAN_TxFrame_t *frame, CAN_TxPriority_t priority, uint32_t deadline_ms,
    uint32_t period_us, CAN_TxPeriodicPrepareFn_t prepare, uint32_t *request_id)
{
    TEST_CHECK(prepare == NULL); // 该目标不启用底盘板许可。
    return CANSubmitPeriodic(hcan, source, key, frame, priority, deadline_ms, period_us, request_id);
}

static void tick(uint64_t at)
{
    now_us = at;
    clock_reads = 0u;
    DJIMotorControl();
    TEST_CHECK(clock_reads == 1u);
}

static void feed_zero_feedback(DJIMotorInstance *motor)
{
    CANInstance *can = (CANInstance *)motor->motor_can_port.backend; // 测试模型直接注入底层接收帧。
    memset(can->rx_buff, 0, sizeof(can->rx_buff));
    can->rx_len = 8u;
    can->rx_timestamp_cycles = 480000u;
    can->rx_timestamp_ms = 1u;
    can->can_module_callback(can);
}

/** @brief 仅使用有效驱动接口和可观察的发送字节验证计算结果。 */
int main(void)
{
    PID_Init_Config_s pid = {.Kp = 1.0f, .Ki = 1.0f, .MaxOut = 100.0f,
        .DtMin_s = MOTOR_PID_DT_MIN_S, .DtMax_s = MOTOR_PID_DT_MAX_S};
    Motor_Init_Config_s config = {
        .can_init_config = {.can_bus = PLATFORM_CAN_BUS_1, .tx_id = 1u},
        .motor_type = M3508,
        .output_axis = {1.0f, MOTOR_AXIS_DIRECTION_NORMAL},
        .controller_setting_init_config = {.outer_loop_type = ANGLE_LOOP,
            .close_loop_type = ALL_THREE_LOOP},
        .controller_param_init_config = {.angle_PID = pid, .speed_PID = pid, .current_PID = pid},
    };
    config.can_init_config.can_bus = PLATFORM_CAN_INVALID; // 无绑定不能注册电机。
    TEST_CHECK(DJIMotorInit(&config) == NULL && registered == 0u && submitted == 0u);
    config.can_init_config.can_bus = PLATFORM_CAN_BUS_1; // 恢复合法总线后继续检查 PID。
    config.controller_param_init_config.speed_PID.DtMin_s = 0.0f;
    DJIMotorInstance *motor = DJIMotorInit(&config);
    TEST_CHECK(motor == NULL);
    TEST_CHECK(registered == 0u && submitted == 0u);
    config.controller_param_init_config.speed_PID = pid;
    motor = DJIMotorInit(&config);
    TEST_CHECK(motor != NULL && registered == 1u);
    feed_zero_feedback(motor);
    DJIMotorSetRef(motor, 2.0f);
    DJIMotorEnable(motor);
    tick(1000u);
    TEST_CHECK(sent[0] == 0u && sent[1] == 0u);
    tick(2000u);
    TEST_CHECK(sent[0] == 0u && sent[1] == 2u);
    TEST_CHECK(motor->motor_controller.angle_PID.state.dt == motor->motor_controller.speed_PID.state.dt);
    TEST_CHECK(motor->motor_controller.angle_PID.state.dt == motor->motor_controller.current_PID.state.dt);
    motor->measure.speed_aps = NAN;
    tick(3000u);
    TEST_CHECK(sent[0] == 0u && sent[1] == 0u);
    TEST_CHECK(motor->pid_clock.last_status == PID_NUMERIC_ERROR);
    TEST_CHECK(motor->motor_controller.angle_PID.state.Iout == 0.0f);
    motor->measure.speed_aps = 0.0f;
    tick(4000u);
    TEST_CHECK(sent[1] == 2u);
    tick(100000u);
    TEST_CHECK(sent[0] == 0u && sent[1] == 0u);
    tick(101000u);
    TEST_CHECK(sent[1] == 2u);
    tick(101000u);
    TEST_CHECK(sent[0] == 0u && sent[1] == 0u);
    return EXIT_SUCCESS;
}
