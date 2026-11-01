/**
 * @file motor_feedback_filter_test.c
 * @brief 通过 DJI 生产初始化、CAN 回调和摘要接口验证反馈滤波配置。
 * @note 每个 CTest 进程用一个参数选择独立场景；CAN、Daemon 和 DWT 由本地替身提供，
 *       不连接或驱动真实电机。
 * @author MC-02 project contributors
 * @copyright Copyright (c) HNU YueLu EC 2022 all rights reserved
 */
#include "bsp_dwt.h"
#include "dji_motor.h"
#include "motor_feedback_test_support.h"
#include "test_check.h"
#if defined(CHASSIS_BOARD)
#include "FreeRTOS.h"
#include "robot_safety.h"
#endif

#include <math.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

CAN_HandleTypeDef hcan1 = {1u}, hcan2 = {2u};
volatile uint32_t dwt_cpu_freq_mhz = 480u;

static uint64_t now_us;
static unsigned registered;
static unsigned daemon_reloads;
static uint8_t daemon_online = 1u;
static CANInstance can_instances[12];
static DaemonInstance daemon;
static uint8_t sent[8];
#if defined(CHASSIS_BOARD)
unsigned motor_safety_test_critical_depth;
#endif

uint64_t DWT_GetTimeline_us(void) { return now_us; }
uint32_t HAL_GetTick(void) { return (uint32_t)(now_us / 1000u); }
float DWT_GetDeltaT(uint32_t *count) { ++*count; return 0.001f; }
uint32_t DWT_ProbeStart(void) { return 0u; }
void DWT_ProbeDone(volatile DWT_Probe_t *probe, uint32_t start) { (void)probe; (void)start; }
DaemonInstance *DaemonRegister(const Daemon_Init_Config_s *config) { (void)config; return &daemon; }
uint8_t DaemonUnregister(DaemonInstance *instance) { (void)instance; return 1u; }
void DaemonReload(DaemonInstance *instance) { (void)instance; ++daemon_reloads; daemon_online = 1u; }
DaemonStatus_t DaemonReportAt(DaemonInstance *instance, uint32_t received_ms, uint8_t flags)
{ (void)instance; (void)received_ms; if ((flags & DAEMON_REPORT_TRANSPORT_VALID) != 0u) { ++daemon_reloads; daemon_online = 1u; } return DAEMON_OK; }
uint8_t DaemonIsOnline(DaemonInstance *instance) { (void)instance; return daemon_online; }

CANInstance *CANRegister(CAN_Init_Config_s *config)
{
    TEST_CHECK(config != NULL && registered < 12u);
    CANInstance *instance = &can_instances[registered++];
    memset(instance, 0, sizeof(*instance));
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
    return CANSubmitPeriodicPrepared(hcan, source, key, frame, priority, deadline_ms,
                                     period_us, NULL, request_id);
}

CAN_TxSubmitStatus_t CANSubmitPeriodicPrepared(FDCAN_HandleTypeDef *hcan, const void *source,
                                       uint32_t key, const CAN_TxFrame_t *frame,
                                       CAN_TxPriority_t priority, uint32_t deadline_ms,
                                       uint32_t period_us, CAN_TxPeriodicPrepareFn_t prepare,
                                       uint32_t *request_id)
{
    (void)hcan; (void)source; (void)key; (void)priority; (void)deadline_ms; (void)period_us;
    TEST_CHECK(frame != NULL && request_id != NULL && frame->length <= sizeof(sent));
#if defined(CHASSIS_BOARD)
    TEST_CHECK(motor_safety_test_critical_depth != 0u);
#endif
    uint8_t payload[sizeof(sent)] = {0};
    memcpy(payload, frame->data, frame->length);
    if (prepare != NULL)
        prepare(source, HAL_GetTick(), payload, frame->length);
    memset(sent, 0, sizeof(sent));
    memcpy(sent, payload, frame->length);
    *request_id = 1u;
    return CAN_TX_SUBMIT_ACCEPTED;
}

static Motor_Init_Config_s motor_config(uint8_t id)
{
    PID_Init_Config_s pid = {.Kp = 1.0f, .MaxOut = 100.0f,
        .DtMin_s = MOTOR_PID_DT_MIN_S, .DtMax_s = MOTOR_PID_DT_MAX_S};
    Motor_Init_Config_s config = {
        .can_init_config = {.can_handle = &hcan1, .tx_id = id},
        .motor_type = M3508,
        .output_axis = {1.0f, MOTOR_AXIS_DIRECTION_NORMAL},
        .controller_setting_init_config = {
            .outer_loop_type = SPEED_LOOP,
            .close_loop_type = SPEED_LOOP | CURRENT_LOOP,
        },
        .controller_param_init_config = {.speed_PID = pid, .current_PID = pid},
    };
    return config;
}

static DJI_Motor_Filter_Config_s filter_config(DJI_Motor_Filter_Mode_e mode)
{
    return (DJI_Motor_Filter_Config_s){
        .speed_mode = mode,
        .current_mode = mode,
        .speed_tau_s = mode == DJI_MOTOR_FILTER_TIME_CONSTANT ? 0.010f : 0.0f,
        .current_tau_s = mode == DJI_MOTOR_FILTER_TIME_CONSTANT ? 0.010f : 0.0f,
        .dt_min_s = MOTOR_PID_DT_MIN_S,
        .dt_max_s = MOTOR_PID_DT_MAX_S,
    };
}

static void receive(DJIMotorInstance *motor, uint32_t cycles, uint32_t tick_ms,
                    int16_t rpm, int16_t current)
{
    CANInstance *can = motor->motor_can_instance;
    memset(can->rx_buff, 0, sizeof(can->rx_buff));
    can->rx_buff[2] = (uint8_t)((uint16_t)rpm >> 8u);
    can->rx_buff[3] = (uint8_t)rpm;
    can->rx_buff[4] = (uint8_t)((uint16_t)current >> 8u);
    can->rx_buff[5] = (uint8_t)current;
    can->rx_len = 8u;
    can->rx_timestamp_cycles = cycles;
    can->rx_timestamp_ms = tick_ms;
    TEST_CHECK(can->can_module_callback != NULL);
    can->can_module_callback(can);
}

static DJIMotorSummary_t summary(DJIMotorInstance *motor)
{
    DJIMotorSummary_t result = {0};
    DJIMotorGetSummary(motor, &result);
    return result;
}

static void control_at(uint64_t at_us)
{
    now_us = at_us;
    DJIMotorControl();
#if defined(CHASSIS_BOARD)
    TEST_CHECK(motor_safety_test_critical_depth == 0u);
#endif
}

#if defined(CHASSIS_BOARD)
static int chassis_safety_output_gate(void)
{
    RobotSafetyInit();
    RobotSafetySetImuValid(1u);
    RobotSafetySetCalibValid(1u);
    RobotSafetySetTaskHealthy(1u);
    RobotSafetySnapshot_t safety_snapshot = {0};
    TEST_CHECK(RobotSafetyGetSnapshot(&safety_snapshot));
    TEST_CHECK(RobotSafetyTryGrantRemoteChassisPermit(safety_snapshot.chassis_inhibit_generation,
        HAL_GetTick(), 50u, HAL_GetTick()));
    RobotSafetyUpdate();
    TEST_CHECK(RobotSafetyIsReady());

    DJIMotorInstance *motors[4] = {0};
    for (uint8_t i = 0u; i < 4u; ++i)
    {
        Motor_Init_Config_s config = motor_config((uint8_t)(i + 1u));
        motors[i] = DJIMotorInit(&config);
        TEST_CHECK(motors[i] != NULL);
        receive(motors[i], 0u, 0u, 0, 0);
        DJIMotorSetRef(motors[i], 10.0f);
        DJIMotorEnable(motors[i]);
    }

    control_at(1000u);
    control_at(2000u);
    uint8_t nonzero = 0u;
    for (size_t i = 0u; i < sizeof(sent); ++i)
        if (sent[i] != 0u) nonzero = 1u;
    TEST_CHECK(nonzero != 0u);

    DJIMotorSummary_t motor_summary = summary(motors[2]);
    TEST_CHECK(motor_summary.online != 0u);
    motors[2]->speed_filter_valid = 0u;
    control_at(3000u);
    TEST_CHECK(!RobotSafetyIsReady());
    for (size_t i = 0u; i < sizeof(sent); ++i)
        TEST_CHECK(sent[i] == 0u);

    motors[2]->speed_filter_valid = 1u;
    RobotFaultClear(ROBOT_FAULT_MOTOR_OFFLINE);
    RobotSafetyUpdate();
    TEST_CHECK(!RobotSafetyIsReady());
    TEST_CHECK(RobotSafetyGetSnapshot(&safety_snapshot));
    TEST_CHECK(RobotSafetyTryGrantRemoteChassisPermit(safety_snapshot.chassis_inhibit_generation,
        HAL_GetTick(), 50u, HAL_GetTick()));
    RobotSafetyUpdate();
    TEST_CHECK(RobotSafetyIsReady());
    RobotSafetyRequestDisable();
    TEST_CHECK(!RobotSafetyIsReady());
    control_at(4000u);
    for (size_t i = 0u; i < sizeof(sent); ++i)
        TEST_CHECK(sent[i] == 0u);
    return EXIT_SUCCESS;
}
#endif

static int legacy_compatibility(void)
{
    Motor_Init_Config_s config = motor_config(1u);
    DJIMotorInstance *motor = DJIMotorInit(&config);
    TEST_CHECK(motor != NULL);
    motor->motor_can_instance->rx_len = 7u;
    motor->motor_can_instance->can_module_callback(motor->motor_can_instance);
    TEST_CHECK(daemon_reloads == 0u && motor->feedback_sample_count == 0u);
    receive(motor, 100u, 0u, 10, 100);
    DJIMotorSummary_t first = summary(motor);
    TEST_CHECK(fabsf(first.speed_aps - 51.0f) < 1e-5f);
    TEST_CHECK(first.current == 90);
    TEST_CHECK(first.raw_speed_aps == 60.0f && first.raw_current == 100);
    receive(motor, 480100u, 1u, 20, 200);
    DJIMotorSummary_t second = summary(motor);
    TEST_CHECK(fabsf(second.speed_aps - 109.65f) < 1e-4f);
    TEST_CHECK(second.current == 189 && second.raw_current == 200);
    TEST_CHECK(fabsf(second.feedback_dt_s - 0.001f) < 1e-6f);
    TEST_CHECK(second.speed_filter_status == DJI_FILTER_STATUS_VALID);
    TEST_CHECK(second.current_filter_status == DJI_FILTER_STATUS_VALID);
    TEST_CHECK(daemon_reloads == 2u);
    return EXIT_SUCCESS;
}

static int time_constant_filter(void)
{
    Motor_Init_Config_s config = motor_config(1u);
    DJI_Motor_Filter_Config_s filter = filter_config(DJI_MOTOR_FILTER_TIME_CONSTANT);
    DJIMotorInstance *motor = DJIMotorInitWithFilter(&config, &filter);
    TEST_CHECK(motor != NULL);
    receive(motor, 0xFFFFFF00u, 5u, 10, 100);
    DJIMotorSummary_t first = summary(motor);
    TEST_CHECK(first.speed_filter_status == DJI_FILTER_STATUS_REBUILT);
    TEST_CHECK(first.current_filter_status == DJI_FILTER_STATUS_REBUILT);
    TEST_CHECK(first.speed_aps == 60.0f && first.current_feedback == 100.0f);
    receive(motor, 0x00075200u, 5u, 20, 200);
    DJIMotorSummary_t second = summary(motor);
    TEST_CHECK(fabsf(second.feedback_dt_s - 0.001f) < 1e-6f);
    TEST_CHECK(fabsf(second.speed_aps - 65.454544f) < 1e-4f);
    TEST_CHECK(fabsf(second.current_feedback - 109.09091f) < 1e-4f);
    TEST_CHECK(second.raw_speed_aps == 120.0f && second.raw_current == 200);
    receive(motor, 0x0015F800u, 7u, 0, 0);
    DJIMotorSummary_t third = summary(motor);
    TEST_CHECK(fabsf(third.feedback_dt_s - 0.002f) < 1e-6f);
    TEST_CHECK(fabsf(third.speed_aps - 54.54545f) < 1e-4f);
    TEST_CHECK(fabsf(third.current_feedback - 90.90909f) < 1e-4f);
    /* 查询不能修改最近接收的来源间隔，也不能刷新 Daemon。 */
    for (unsigned i = 0; i < 8u; ++i)
        third = summary(motor);
    TEST_CHECK(fabsf(third.feedback_dt_s - 0.002f) < 1e-6f);
    TEST_CHECK(daemon_reloads == 3u);
    return EXIT_SUCCESS;
}

static int bypass_filter(void)
{
    Motor_Init_Config_s config = motor_config(1u);
    DJI_Motor_Filter_Config_s filter = filter_config(DJI_MOTOR_FILTER_DISABLED);
    DJIMotorInstance *motor = DJIMotorInitWithFilter(&config, &filter);
    TEST_CHECK(motor != NULL);
    receive(motor, 1000u, 8u, -25, -321);
    DJIMotorSummary_t result = summary(motor);
    TEST_CHECK(result.speed_aps == -150.0f && result.current_feedback == -321.0f);
    TEST_CHECK(result.speed_filter_status == DJI_FILTER_STATUS_BYPASSED);
    TEST_CHECK(result.current_filter_status == DJI_FILTER_STATUS_BYPASSED);
    TEST_CHECK(result.raw_speed_aps == -150.0f && result.raw_current == -321);
    return EXIT_SUCCESS;
}

static int gap_and_rebuild(void)
{
    Motor_Init_Config_s config = motor_config(1u);
    DJI_Motor_Filter_Config_s filter = filter_config(DJI_MOTOR_FILTER_TIME_CONSTANT);
    DJIMotorInstance *motor = DJIMotorInitWithFilter(&config, &filter);
    TEST_CHECK(motor != NULL);
    receive(motor, 1000u, 1u, 10, 100);
    receive(motor, 51u * 480000u + 1000u, 52u, 20, 200);
    DJIMotorSummary_t gap = summary(motor);
    TEST_CHECK(!gap.speed_filter_valid && !gap.current_filter_valid);
    TEST_CHECK(gap.speed_filter_status == DJI_FILTER_STATUS_GAP);
    TEST_CHECK(gap.current_filter_status == DJI_FILTER_STATUS_GAP);
    TEST_CHECK(gap.speed_aps == 60.0f && gap.current_feedback == 100.0f);

    DJIMotorEnable(motor);
    DJIMotorSetRef(motor, 10.0f);
    control_at(1000u);
    control_at(2000u);
    TEST_CHECK(sent[0] == 0u && sent[1] == 0u);

    receive(motor, 52u * 480000u + 1000u, 53u, 30, 300);
    DJIMotorSummary_t rebuilt = summary(motor);
    TEST_CHECK(rebuilt.speed_filter_valid && rebuilt.current_filter_valid);
    TEST_CHECK(rebuilt.speed_filter_status == DJI_FILTER_STATUS_REBUILT);
    TEST_CHECK(rebuilt.current_filter_status == DJI_FILTER_STATUS_REBUILT);
    TEST_CHECK(rebuilt.speed_aps == 180.0f && rebuilt.current_feedback == 300.0f);
    control_at(3000u);
    TEST_CHECK(sent[1] != 0u);
    return EXIT_SUCCESS;
}

static int independent_instances(void)
{
    Motor_Init_Config_s legacy_config = motor_config(1u);
    DJI_Motor_Filter_Config_s legacy_filter = filter_config(DJI_MOTOR_FILTER_LEGACY_FRAME);
    DJIMotorInstance *legacy = DJIMotorInitWithFilter(&legacy_config, &legacy_filter);
    TEST_CHECK(legacy != NULL);

    Motor_Init_Config_s bypass_config = motor_config(2u);
    DJI_Motor_Filter_Config_s bypass_filter = filter_config(DJI_MOTOR_FILTER_DISABLED);
    DJIMotorInstance *bypass = DJIMotorInitWithFilter(&bypass_config, &bypass_filter);
    TEST_CHECK(bypass != NULL);

    receive(legacy, 1000u, 0u, 10, 100);
    receive(bypass, 1000u, 0u, 10, 100);
    receive(legacy, 481000u, 1u, 20, 200);
    receive(bypass, 481000u, 1u, 20, 200);
    DJIMotorSummary_t legacy_result = summary(legacy);
    DJIMotorSummary_t bypass_result = summary(bypass);
    TEST_CHECK(fabsf(legacy_result.speed_aps - 109.65f) < 1e-4f);
    TEST_CHECK(bypass_result.speed_aps == 120.0f);
    TEST_CHECK(legacy_result.raw_current == 200 && bypass_result.raw_current == 200);
    return EXIT_SUCCESS;
}

static int interval_bounds(void)
{
    Motor_Init_Config_s config = motor_config(1u);
    DJI_Motor_Filter_Config_s filter = filter_config(DJI_MOTOR_FILTER_TIME_CONSTANT);
    DJIMotorInstance *motor = DJIMotorInitWithFilter(&config, &filter);
    TEST_CHECK(motor != NULL);

    receive(motor, 0u, 0u, 10, 100);
    receive(motor, 4800u, 0u, 20, 200);
    TEST_CHECK(summary(motor).speed_filter_status == DJI_FILTER_STATUS_VALID);
    receive(motor, 4800u + 24000000u, 50u, 30, 300);
    DJIMotorSummary_t at_max = summary(motor);
    TEST_CHECK(fabsf(at_max.feedback_dt_s - 0.05f) < 1e-7f);
    TEST_CHECK(at_max.speed_filter_status == DJI_FILTER_STATUS_VALID);

    /* 超过上界时保留最后数值并撤销有效标记；下一份样本重建。 */
    receive(motor, 4800u + 24000000u + 24004800u, 100u, 40, 400);
    DJIMotorSummary_t above_max = summary(motor);
    TEST_CHECK(!above_max.speed_filter_valid && above_max.speed_filter_status == DJI_FILTER_STATUS_GAP);
    TEST_CHECK(above_max.speed_aps == at_max.speed_aps);
    receive(motor, 4800u + 24000000u + 24004800u + 4800u, 100u, 50, 500);
    TEST_CHECK(summary(motor).speed_filter_status == DJI_FILTER_STATUS_REBUILT);

    /* 低于下界的间隔也作废，不能把异常样本混入历史。 */
    receive(motor, 4800u + 24000000u + 24004800u + 4800u + 4790u, 100u, 60, 600);
    DJIMotorSummary_t below_min = summary(motor);
    TEST_CHECK(!below_min.speed_filter_valid && below_min.speed_filter_status == DJI_FILTER_STATUS_GAP);
    receive(motor, 4800u + 24000000u + 24004800u + 4800u + 4790u + 4800u, 100u, 70, 700);
    TEST_CHECK(summary(motor).speed_filter_status == DJI_FILTER_STATUS_REBUILT);
    return EXIT_SUCCESS;
}

static int channel_independence(void)
{
    Motor_Init_Config_s config = motor_config(1u);
    DJI_Motor_Filter_Config_s filter = filter_config(DJI_MOTOR_FILTER_TIME_CONSTANT);
    filter.current_mode = DJI_MOTOR_FILTER_DISABLED;
    filter.current_tau_s = 0.0f;
    DJIMotorInstance *motor = DJIMotorInitWithFilter(&config, &filter);
    TEST_CHECK(motor != NULL);
    receive(motor, 1000u, 1u, 10, 100);
    receive(motor, 481000u, 2u, 20, 200);
    DJIMotorSummary_t mixed = summary(motor);
    TEST_CHECK(mixed.speed_filter_status == DJI_FILTER_STATUS_VALID);
    TEST_CHECK(mixed.current_filter_status == DJI_FILTER_STATUS_BYPASSED);
    TEST_CHECK(mixed.current_feedback == 200.0f);
    TEST_CHECK(fabsf(mixed.speed_aps - 65.454544f) < 1e-4f);

    /* 外部速度环不消费内部速度滤波值，内部通道失效不能挡住该环。 */
    float external_speed = 25.0f;
    config = motor_config(2u);
    config.controller_setting_init_config.speed_feedback_source = OTHER_FEED;
    config.controller_setting_init_config.close_loop_type = SPEED_LOOP;
    config.controller_param_init_config.other_speed_feedback = MotorFeedbackTestMakeSource(
        &external_speed, MotorFeedbackTestReadRadSExternal, MOTOR_UNIT_RAD_S,
        MOTOR_REFERENCE_EXTERNAL);
    filter = filter_config(DJI_MOTOR_FILTER_TIME_CONSTANT);
    motor = DJIMotorInitWithFilter(&config, &filter);
    TEST_CHECK(motor != NULL);
    receive(motor, 1000000u, 3u, 0, 0);
    TEST_CHECK(summary(motor).speed_filter_valid);
    DJIMotorEnable(motor);
    DJIMotorSetRef(motor, 30.0f);
    control_at(1000u);
    daemon_online = 0u;
    control_at(2000u);
    TEST_CHECK(daemon_reloads == 3u);
    TEST_CHECK(!summary(motor).speed_filter_valid);
    TEST_CHECK(sent[2] != 0u || sent[3] != 0u);
    return EXIT_SUCCESS;
}

static int missing_sample_recovery(void)
{
    Motor_Init_Config_s config = motor_config(1u);
    DJI_Motor_Filter_Config_s filter = filter_config(DJI_MOTOR_FILTER_TIME_CONSTANT);
    DJIMotorInstance *motor = DJIMotorInitWithFilter(&config, &filter);
    TEST_CHECK(motor != NULL);
    receive(motor, 1000u, 1u, 10, 100);
    DJIMotorEnable(motor);
    DJIMotorSetRef(motor, 10.0f);
    control_at(1000u);
    daemon_online = 0u;
    control_at(2000u);
    DJIMotorSummary_t summary_after_gap = summary(motor);
    TEST_CHECK(!summary_after_gap.speed_filter_valid && !summary_after_gap.current_filter_valid);
    TEST_CHECK(summary_after_gap.speed_filter_status == DJI_FILTER_STATUS_GAP);
    TEST_CHECK(sent[0] == 0u && sent[1] == 0u);
    TEST_CHECK(daemon_reloads == 1u);

    receive(motor, 481000u, 2u, 20, 200);
    DJIMotorSummary_t rebuilt = summary(motor);
    TEST_CHECK(rebuilt.speed_filter_valid && rebuilt.current_filter_valid);
    TEST_CHECK(rebuilt.speed_filter_status == DJI_FILTER_STATUS_REBUILT);
    TEST_CHECK(rebuilt.current_filter_status == DJI_FILTER_STATUS_REBUILT);
    TEST_CHECK(daemon_reloads == 2u);
    control_at(3000u);
    TEST_CHECK(sent[0] != 0u || sent[1] != 0u);
    return EXIT_SUCCESS;
}

static int reject_invalid_config(void)
{
    Motor_Init_Config_s config = motor_config(1u);
    DJI_Motor_Filter_Config_s filter = filter_config(DJI_MOTOR_FILTER_TIME_CONSTANT);
    filter.speed_tau_s = NAN;
    TEST_CHECK(DJIMotorInitWithFilter(&config, &filter) == NULL);
    TEST_CHECK(registered == 0u);
    filter.speed_tau_s = 0.01f;
    filter.dt_max_s = filter.dt_min_s / 2.0f;
    TEST_CHECK(DJIMotorInitWithFilter(&config, &filter) == NULL);
    TEST_CHECK(registered == 0u);
    filter.dt_max_s = MOTOR_PID_DT_MAX_S;
    filter.current_mode = (DJI_Motor_Filter_Mode_e)99;
    TEST_CHECK(DJIMotorInitWithFilter(&config, &filter) == NULL);
    TEST_CHECK(registered == 0u);
    return EXIT_SUCCESS;
}

int main(int argc, char **argv)
{
    TEST_CHECK(argc == 2);
#if defined(CHASSIS_BOARD)
    if (strcmp(argv[1], "chassis-safety-output") == 0)
        return chassis_safety_output_gate();
#endif
    if (strcmp(argv[1], "legacy") == 0) return legacy_compatibility();
    if (strcmp(argv[1], "time-constant") == 0) return time_constant_filter();
    if (strcmp(argv[1], "bypass") == 0) return bypass_filter();
    if (strcmp(argv[1], "gap") == 0) return gap_and_rebuild();
    if (strcmp(argv[1], "multi-instance") == 0) return independent_instances();
    if (strcmp(argv[1], "bounds") == 0) return interval_bounds();
    if (strcmp(argv[1], "channels") == 0) return channel_independence();
    if (strcmp(argv[1], "missing") == 0) return missing_sample_recovery();
    if (strcmp(argv[1], "config") == 0) return reject_invalid_config();
    return EXIT_FAILURE;
}
