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
#include "bsp_can.h" // 测试替身直接构造底层 CAN 接收实例。
#include "motor_feedback_test_support.h"
#include "test_check.h"
#if defined(CHASSIS_BOARD) || defined(ONE_BOARD)
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
#if defined(CHASSIS_BOARD) || defined(ONE_BOARD)
unsigned motor_safety_test_critical_depth;
#endif
#if defined(ONE_BOARD)
static CAN_TxPeriodicPrepareFn_t deferred_prepare; // 模拟目标已排队但尚未交给硬件。
static const void *deferred_source; // 保存与排队目标关联的发送来源。
static uint8_t deferred_payload[8]; // 测试副本，延后执行发送前检查。
static uint8_t reject_periodic; // 模拟队列拒绝替换旧目标。
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
#if defined(ONE_BOARD)
    if (reject_periodic != 0u) return CAN_TX_SUBMIT_FULL; // 失败时旧目标和回调保持原状。
#endif
#if defined(CHASSIS_BOARD) || defined(ONE_BOARD)
    TEST_CHECK(motor_safety_test_critical_depth != 0u);
#endif
    uint8_t payload[sizeof(sent)] = {0};
    memcpy(payload, frame->data, frame->length);
#if defined(ONE_BOARD)
    deferred_prepare = prepare; // 保留真实回调，用于模拟队列等待期间失能。
    deferred_source = source;
    memcpy(deferred_payload, payload, sizeof(deferred_payload));
#endif
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
        .can_init_config = {.can_bus = PLATFORM_CAN_BUS_1, .tx_id = id},
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
    CANInstance *can = (CANInstance *)motor->motor_can_port.backend; // 从测试端点注入底层帧。
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
#if defined(CHASSIS_BOARD) || defined(ONE_BOARD)
    TEST_CHECK(motor_safety_test_critical_depth == 0u);
#endif
}

#if defined(ONE_BOARD)
static int single_safety_output_gate(void)
{
    RobotSafetyInit(); // 初始状态禁止电机输出。
    Motor_Init_Config_s config = motor_config(1u); // 使用隔离的主机测试电机参数。
    DJIMotorInstance *motor = DJIMotorInit(&config);
    TEST_CHECK(motor != NULL);
    receive(motor, 0u, 0u, 0, 0); // 提供合法反馈供测试 PID 计算。
    DJIMotorSetRef(motor, 10.0f);
    DJIMotorEnable(motor);
    control_at(1000u);
    for (size_t i = 0u; i < sizeof(sent); ++i) TEST_CHECK(sent[i] == 0u); // 未使能时只发送零目标。
    RobotSafetySetRcOnline(1u);
    RobotSafetySetImuValid(1u);
    RobotSafetySetCalibValid(1u);
    RobotSafetySetTaskHealthy(1u);
    RobotSafetyRequestEnable();
    RobotSafetyUpdate();
    TEST_CHECK(RobotSafetyIsReady());
    RobotSafetySnapshot_t before_fault = {0};
    TEST_CHECK(RobotSafetyGetSnapshot(&before_fault));
    RobotFaultReport(ROBOT_FAULT_CAN_BUSOFF); // 两次发送之间短暂出现并恢复故障。
    RobotFaultClear(ROBOT_FAULT_CAN_BUSOFF);
    RobotSafetyUpdate();
    TEST_CHECK(RobotSafetyIsReady());
    TEST_CHECK(!RobotSafetyIsReadyForGeneration(HAL_GetTick(), before_fault.control_generation)); // 旧代际不能借恢复后的 READY 通过。
    control_at(2000u);
    uint8_t nonzero = 0u;
    for (size_t i = 0u; i < sizeof(sent); ++i) if (sent[i] != 0u) nonzero = 1u;
    TEST_CHECK(nonzero != 0u && deferred_prepare != NULL); // 有许可时队列取得实际目标和复核回调。
    RobotSafetyRequestDisable(); // 目标排队后、交给 HAL 前撤销许可。
    deferred_prepare(deferred_source, HAL_GetTick(), deferred_payload, sizeof(deferred_payload));
    for (size_t i = 0u; i < sizeof(deferred_payload); ++i) TEST_CHECK(deferred_payload[i] == 0u);
    RobotSafetyRequestEnable(); // 假设故障消失并重新使能。
    RobotSafetyUpdate();
    TEST_CHECK(RobotSafetyIsReady());
    reject_periodic = 1u; // 替换失败，队列仍持有失能前的旧目标。
    control_at(2500u);
    deferred_payload[0] = 0x5au; // 模拟旧队列仍保存非零电流目标。
    deferred_prepare(deferred_source, HAL_GetTick(), deferred_payload, sizeof(deferred_payload));
    TEST_CHECK(deferred_payload[0] == 0u); // 恢复许可不能给旧目标续用新的代际。
    reject_periodic = 0u;
    RobotSafetyRequestDisable();
    control_at(3000u);
    for (size_t i = 0u; i < sizeof(sent); ++i) TEST_CHECK(sent[i] == 0u); // 后续计算仍不能输出旧目标。
    return EXIT_SUCCESS;
}

static int single_feedback_roles(void)
{
    DJI_Feedback_Roles_s result = {0}; // 接收每次独立读取的机构状态。
    DJIMotorGetFeedbackRoles(0u, &result); // 没有注册电机时不能声称反馈有效。
    TEST_CHECK(result.present_roles == 0u && result.valid_roles == 0u);
    Motor_Init_Config_s config = motor_config(1u); // 注册两台同属底盘的电机。
    config.output_role = MOTOR_OUTPUT_CHASSIS;
    DJIMotorInstance *first = DJIMotorInit(&config);
    config.can_init_config.tx_id = 2u;
    DJIMotorInstance *second = DJIMotorInit(&config);
    TEST_CHECK(first != NULL && second != NULL);
    now_us = 1000u;
    receive(first, 1000u, 1u, 0, 0); // 只有第一台有反馈，不足以放行整个机构。
    DJIMotorGetFeedbackRoles(1u, &result);
    const uint32_t chassis = 1u << MOTOR_OUTPUT_CHASSIS;
    TEST_CHECK(result.present_roles == chassis && result.valid_roles == 0u);
    TEST_CHECK(result.motor_count[MOTOR_OUTPUT_CHASSIS] == 2u); // 即使尚未全部反馈，也能检查注册数量。
    now_us = 2000u; receive(second, 2000u, 2u, 0, 0); // 同机构两台均有效后才标记有效。
    DJIMotorGetFeedbackRoles(2u, &result);
    TEST_CHECK(result.valid_roles == chassis && result.oldest_source_ms[MOTOR_OUTPUT_CHASSIS] == 1u);
    DJIMotorGetFeedbackRoles(2u, &result); // 重复读取不改变最早采样时刻。
    TEST_CHECK(result.oldest_source_ms[MOTOR_OUTPUT_CHASSIS] == 1u);
    now_us = 23000u; // 两个样本均已超过现有反馈期限。
    DJIMotorGetFeedbackRoles(23u, &result);
    TEST_CHECK(result.valid_roles == 0u);
    receive(first, 23000u, 23u, 0, 0); // 只恢复一台不能恢复整个机构。
    DJIMotorGetFeedbackRoles(23u, &result);
    TEST_CHECK(result.valid_roles == 0u);
    receive(second, 23000u, 23u, 0, 0);
    DJIMotorGetFeedbackRoles(23u, &result);
    TEST_CHECK(result.valid_roles == chassis && result.oldest_source_ms[MOTOR_OUTPUT_CHASSIS] == 23u);
    config.can_init_config.tx_id = 3u; // 云台反馈缺失不影响已经有效的底盘。
    config.output_role = MOTOR_OUTPUT_GIMBAL;
    DJIMotorInstance *gimbal = DJIMotorInit(&config);
    TEST_CHECK(gimbal != NULL);
    DJIMotorGetFeedbackRoles(23u, &result);
    TEST_CHECK(result.present_roles == (chassis | (1u << MOTOR_OUTPUT_GIMBAL)));
    TEST_CHECK(result.valid_roles == chassis && result.motor_count[MOTOR_OUTPUT_GIMBAL] == 1u);
    config.can_init_config.tx_id = 4u; // 未声明用途应单独报告，不能算作已通过的机构。
    config.output_role = MOTOR_OUTPUT_UNASSIGNED;
    TEST_CHECK(DJIMotorInit(&config) != NULL);
    DJIMotorGetFeedbackRoles(23u, &result);
    TEST_CHECK(result.unassigned_count == 1u && result.valid_roles == chassis);
    daemon_online = 0u; // 即使时间尚未过期，离线来源也不能标记有效。
    DJIMotorGetFeedbackRoles(23u, &result);
    TEST_CHECK(result.valid_roles == 0u);
    return EXIT_SUCCESS;
}

static int single_feedback_roles_wrap(void)
{
    Motor_Init_Config_s config = motor_config(1u); // 两台底盘电机分别在回绕前后接收反馈。
    config.output_role = MOTOR_OUTPUT_CHASSIS;
    DJIMotorInstance *first = DJIMotorInit(&config);
    config.can_init_config.tx_id = 2u;
    DJIMotorInstance *second = DJIMotorInit(&config);
    TEST_CHECK(first != NULL && second != NULL);
    now_us = (uint64_t)(UINT32_MAX - 3u) * 1000u;
    receive(first, 1000u, UINT32_MAX - 3u, 0, 0);
    now_us = ((uint64_t)UINT32_MAX + 2u) * 1000u; // 本机毫秒回绕到 1。
    receive(second, 2000u, 1u, 0, 0);
    DJI_Feedback_Roles_s result = {0};
    DJIMotorGetFeedbackRoles(1u, &result);
    TEST_CHECK((result.valid_roles & (1u << MOTOR_OUTPUT_CHASSIS)) != 0u);
    TEST_CHECK(result.oldest_source_ms[MOTOR_OUTPUT_CHASSIS] == UINT32_MAX - 3u);
    return EXIT_SUCCESS;
}

static int single_feedback_roles_to_safety(void)
{
    now_us = 1000u; RobotSafetyInit(); // 使用实际安全管理与 DJI 反馈汇总。
    RobotSafetySetRcOnline(1u); RobotSafetySetImuValid(1u);
    RobotSafetySetCalibValid(1u); RobotSafetySetTaskHealthy(1u);
    const RobotControlPolicy_t policy = {ROBOT_CONTROL_CHASSIS, 1u, 0u, 0u};
    TEST_CHECK(RobotSafetyConfigureControl(&policy));
    RobotSafetyRequestEnable(); RobotSafetyUpdate();
    Motor_Init_Config_s config = motor_config(1u); // 登记两台底盘 DJI 电机。
    config.output_role = MOTOR_OUTPUT_CHASSIS;
    DJIMotorInstance *first = DJIMotorInit(&config);
    config.can_init_config.tx_id = 2u;
    DJIMotorInstance *second = DJIMotorInit(&config);
    TEST_CHECK(first != NULL && second != NULL);
    now_us = 2000u; receive(first, 2000u, 2u, 0, 0);
    now_us = 4000u; receive(second, 4000u, 4u, 0, 0);
    DJI_Feedback_Roles_s roles = {0};
    DJIMotorGetFeedbackRoles(4u, &roles); // 核对所有预期电机均有效后，才向安全管理提交。
    TEST_CHECK(roles.motor_count[MOTOR_OUTPUT_CHASSIS] == 2u &&
               (roles.valid_roles & (1u << MOTOR_OUTPUT_CHASSIS)) != 0u);
    RobotSafetyControlInputs_t input = {0};
    input.mode = ROBOT_CONTROL_MANUAL;
    input.manual = (RobotControlLease_t){4u, 100u, 1u};
    input.feedback_valid_functions = ROBOT_CONTROL_CHASSIS;
    input.feedback[0] = (RobotControlLease_t){roles.oldest_source_ms[MOTOR_OUTPUT_CHASSIS], DJI_FEEDBACK_MAX_AGE_MS, 1u};
    TEST_CHECK(RobotSafetyPublishControlInputs(&input)); RobotSafetyUpdate();
    TEST_CHECK(RobotSafetyIsReady());
    now_us = 10000u; receive(second, 10000u, 10u, 0, 0); // 更新第二台不能延长第一台的期限。
    DJIMotorGetFeedbackRoles(10u, &roles);
    TEST_CHECK(roles.oldest_source_ms[MOTOR_OUTPUT_CHASSIS] == 2u);
    input.manual.source_time_ms = 10u;
    input.feedback[0].source_time_ms = roles.oldest_source_ms[MOTOR_OUTPUT_CHASSIS];
    TEST_CHECK(RobotSafetyPublishControlInputs(&input)); RobotSafetyUpdate();
    now_us = 23000u; TEST_CHECK(!RobotSafetyIsReady()); // 第一台的实际反馈已超过 20ms。
    now_us = 24000u; receive(first, 24000u, 24u, 0, 0);
    DJIMotorGetFeedbackRoles(24u, &roles);
    TEST_CHECK(roles.oldest_source_ms[MOTOR_OUTPUT_CHASSIS] == 10u);
    input.feedback[0].source_time_ms = roles.oldest_source_ms[MOTOR_OUTPUT_CHASSIS];
    input.manual.source_time_ms = 23u; // 恢复前的旧命令不得接管。
    TEST_CHECK(RobotSafetyPublishControlInputs(&input)); RobotSafetyUpdate();
    TEST_CHECK(!RobotSafetyIsReady());
    now_us = 25000u; input.manual.source_time_ms = 25u;
    TEST_CHECK(RobotSafetyPublishControlInputs(&input)); RobotSafetyUpdate();
    TEST_CHECK(RobotSafetyIsReady()); // 反馈和恢复后的新命令均有效。
    return EXIT_SUCCESS;
}

static int single_feedback_expired_frame(void)
{
    now_us = 1000u; RobotSafetyInit(); // 建立允许人工运行的核心条件。
    RobotSafetySetRcOnline(1u); RobotSafetySetImuValid(1u);
    RobotSafetySetCalibValid(1u); RobotSafetySetTaskHealthy(1u);
    const RobotControlPolicy_t policy = {ROBOT_CONTROL_ALL, 1u, 0u, 0u};
    TEST_CHECK(RobotSafetyConfigureControl(&policy));
    RobotSafetyRequestEnable(); RobotSafetyUpdate();
    RobotSafetyControlInputs_t input = {0};
    input.mode = ROBOT_CONTROL_MANUAL;
    input.manual = (RobotControlLease_t){2u, 100u, 1u}; // 运动命令仍在有效期限内。
    for (unsigned i = 0u; i < 3u; ++i) input.feedback[i] = (RobotControlLease_t){2u, 100u, 1u};
    input.feedback[0].max_age_ms = 3u; // 底盘反馈在本测试中先过期。
    input.feedback_valid_functions = ROBOT_CONTROL_ALL;
    now_us = 2000u; TEST_CHECK(RobotSafetyPublishControlInputs(&input)); RobotSafetyUpdate();
    for (uint8_t i = 0u; i < 2u; ++i)
    {
        Motor_Init_Config_s config = motor_config((uint8_t)(i + 1u));
        config.output_role = i == 0u ? MOTOR_OUTPUT_CHASSIS : MOTOR_OUTPUT_GIMBAL;
        DJIMotorInstance *motor = DJIMotorInit(&config);
        TEST_CHECK(motor != NULL);
        receive(motor, 2000u, 2u, 0, 0); // 两台电机都收到设备反馈。
        DJIMotorSetRef(motor, 10.0f); DJIMotorEnable(motor);
    }
    control_at(3000u); control_at(4000u); // 计算并排队两个非零目标。
    TEST_CHECK((sent[0] || sent[1]) && (sent[2] || sent[3]));
    now_us = 6000u; // 底盘期限已过，排队帧还没有被硬件接受。
    deferred_prepare(deferred_source, HAL_GetTick(), deferred_payload, sizeof(deferred_payload));
    for (unsigned i = 0u; i < 8u; ++i) TEST_CHECK(deferred_payload[i] == 0u); // 发现过期时旧代际整帧撤销。
    control_at(7000u); // 下一帧可以保留其他仍有效的机构。
    TEST_CHECK(sent[0] == 0u && sent[1] == 0u && (sent[2] || sent[3]));
    now_us = 8000u; input.feedback[0] = (RobotControlLease_t){8u, 3u, 1u};
    input.manual.source_time_ms = 7u; // 故障恢复前的命令不得接管。
    TEST_CHECK(RobotSafetyPublishControlInputs(&input)); RobotSafetyUpdate();
    control_at(9000u); TEST_CHECK(sent[0] == 0u && sent[1] == 0u);
    now_us = 10000u; input.manual.source_time_ms = 10u; // 恢复后新的人工命令才可以接管。
    TEST_CHECK(RobotSafetyPublishControlInputs(&input)); RobotSafetyUpdate();
    control_at(11000u); TEST_CHECK(sent[0] || sent[1]);
    return EXIT_SUCCESS;
}

static int single_shared_frame_permissions(uint8_t include_unassigned)
{
    now_us = 1000u; // 先建立本机时刻和核心健康状态。
    RobotSafetyInit();
    RobotSafetySetRcOnline(1u);
    RobotSafetySetImuValid(1u);
    RobotSafetySetCalibValid(1u);
    RobotSafetySetTaskHealthy(1u);
    const RobotControlPolicy_t policy = {ROBOT_CONTROL_ALL, 1u, 0u, 0u};
    TEST_CHECK(RobotSafetyConfigureControl(&policy)); // 启用真实分功能策略。
    RobotSafetyRequestEnable();
    RobotSafetyUpdate(); // 建立恢复起点，不能使用之前的来源。
    RobotSafetyControlInputs_t input = {0};
    input.mode = ROBOT_CONTROL_MANUAL;
    input.manual = (RobotControlLease_t){2u, 100u, 1u}; // 恢复后新鲜的人工来源。
    for (unsigned i = 0u; i < 3u; ++i) input.feedback[i] = (RobotControlLease_t){2u, 100u, 1u}; // 模拟独立有效的三组反馈。
    input.feedback_valid_functions = ROBOT_CONTROL_ALL;
    input.manual_fire_requested = 1u;
    now_us = 2000u;
    TEST_CHECK(RobotSafetyPublishControlInputs(&input));
    RobotSafetyUpdate();
    TEST_CHECK(RobotSafetyIsReady());
    const MotorOutputRole_t roles[4] = {MOTOR_OUTPUT_CHASSIS, MOTOR_OUTPUT_GIMBAL,
                                        MOTOR_OUTPUT_FRICTION, MOTOR_OUTPUT_LOADER};
    for (uint8_t i = 0u; i < 4u; ++i)
    {
        Motor_Init_Config_s config = motor_config((uint8_t)(i + 1u));
        config.output_role = (include_unassigned != 0u && i == 3u) ? MOTOR_OUTPUT_UNASSIGNED : roles[i]; // 可注入未声明的电机。
        DJIMotorInstance *motor = DJIMotorInit(&config);
        TEST_CHECK(motor != NULL);
        receive(motor, 2000u, 2u, 0, 0);
        DJIMotorSetRef(motor, 10.0f);
        DJIMotorEnable(motor);
    }
    control_at(3000u);
    control_at(4000u); // 首个控制周期建立 PID 计时，下一周期才有可检查输出。
    for (uint8_t i = 0u; i < 3u; ++i)
        TEST_CHECK(sent[2u * i] != 0u || sent[2u * i + 1u] != 0u); // 四个机构均有许可。
    if (include_unassigned != 0u)
        TEST_CHECK(sent[6] == 0u && sent[7] == 0u); // 新策略启用时未声明的电机不能运行。
    else
        TEST_CHECK(sent[6] != 0u || sent[7] != 0u); // 已声明的拨弹机构取得机构许可。
    input.feedback_valid_functions &= ~(uint32_t)ROBOT_CONTROL_SHOOTER; // 发射局部故障。
    now_us = 5000u;
    TEST_CHECK(RobotSafetyPublishControlInputs(&input));
    RobotSafetyUpdate();
    deferred_prepare(deferred_source, HAL_GetTick(), deferred_payload, sizeof(deferred_payload));
    for (uint8_t i = 0u; i < 8u; ++i) TEST_CHECK(deferred_payload[i] == 0u); // 旧代际的整帧不能重放。
    control_at(6000u);
    TEST_CHECK(sent[0] != 0u || sent[1] != 0u); // 底盘继续工作。
    TEST_CHECK(sent[2] != 0u || sent[3] != 0u); // 云台继续工作。
    for (uint8_t i = 4u; i < 8u; ++i) TEST_CHECK(sent[i] == 0u); // 摩擦轮和拨弹被单独禁止。
    return EXIT_SUCCESS;
}
#endif

#if defined(CHASSIS_BOARD)
static int feedback_sample_updated_after_caller_tick(void)
{
    Motor_Init_Config_s config = motor_config(1u);
    DJIMotorInstance *motor = DJIMotorInit(&config);
    TEST_CHECK(motor != NULL);

    // 其他任务取时后，电机任务在下一毫秒发布新反馈。
    now_us = 101000u;
    receive(motor, 101000u, 101u, 20, 200);
    Motor_Feedback_Snapshot_s feedback = {0};
    TEST_CHECK(DJIMotorGetFeedbackSnapshot(motor, 100u, &feedback) == MOTOR_FEEDBACK_OK);
    TEST_CHECK(feedback.speed.valid != 0u && feedback.current.valid != 0u);
    return EXIT_SUCCESS;
}

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
    CANInstance *can = (CANInstance *)motor->motor_can_port.backend;
    can->rx_len = 7u;
    can->can_module_callback(can);
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
#if defined(ONE_BOARD)
    if (strcmp(argv[1], "single-safety-output") == 0)
        return single_safety_output_gate();
    if (strcmp(argv[1], "single-feedback-roles") == 0)
        return single_feedback_roles();
    if (strcmp(argv[1], "single-feedback-roles-wrap") == 0)
        return single_feedback_roles_wrap();
    if (strcmp(argv[1], "single-feedback-roles-to-safety") == 0)
        return single_feedback_roles_to_safety();
    if (strcmp(argv[1], "single-feedback-expired-frame") == 0)
        return single_feedback_expired_frame();
    if (strcmp(argv[1], "single-shared-frame") == 0)
        return single_shared_frame_permissions(0u);
    if (strcmp(argv[1], "single-unassigned") == 0)
        return single_shared_frame_permissions(1u);
#endif
#if defined(CHASSIS_BOARD)
    if (strcmp(argv[1], "feedback-after-observed-tick") == 0)
        return feedback_sample_updated_after_caller_tick();
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
