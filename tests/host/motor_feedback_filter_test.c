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
#include "can_recovery.h" // 验证真实反馈能否用于恢复顺序检查。
#if defined(ONE_BOARD)
#include "robot_can_motor_recovery.h" // 使用车型身份检查真实电机反馈。
#include "robot_can_runtime.h" // 联合核心 CAN 故障与安全恢复检查。
#include "robot_can_recovery_coordinator.h" // 联合实际协调器验证恢复顺序。
#include "robot_control_feedback_runtime.h" // 使用固件实际采集入口提交分功能反馈。
#include "message_center.h" // 姿态来源时间使用消息中心的本机时钟标识。
#endif
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
static PlatformCanFaultObserver_t recovery_fault_observer; // 保存测试使用的总线故障回调。
static void *recovery_fault_context; // 测试回调关联上下文。
static uint8_t recovery_bus_ok; // 注入核心总线是否已经重启。

uint8_t PlatformCanInstallFaultObserver(PlatformCanFaultObserver_t observer, void *context)
{
    if (observer == NULL || recovery_fault_observer != NULL) return 0u; // 一个测试进程只安装一次。
    recovery_fault_observer = observer; // 后续使用真实整车观察者处理故障。
    recovery_fault_context = context; // 保留原上下文。
    return 1u;
}
PlatformCanStatus_t PlatformCanGetStatus(PlatformCanBus_t bus)
{
    if (bus != PLATFORM_CAN_BUS_1 && bus != PLATFORM_CAN_BUS_2) return PLATFORM_CAN_STATUS_UNKNOWN; // 仅这两条是测试核心总线。
    return recovery_bus_ok ? PLATFORM_CAN_STATUS_OK : PLATFORM_CAN_STATUS_BUSOFF; // 控制器状态由测试注入。
}
uint8_t PlatformCanRecoveryStillReady(PlatformCanBus_t bus)
{
    return (uint8_t)(recovery_bus_ok && (bus == PLATFORM_CAN_BUS_1 || bus == PLATFORM_CAN_BUS_2)); // 最终总线状态由测试注入。
}
uint8_t RemoteControlRecoveryStillValid(const RemoteControlSnapshot_s *remote, uint32_t now)
{
    return (uint8_t)(remote != NULL && remote->remote_online && remote->radio_sequence != 0u &&
        (uint32_t)(now - remote->radio_source_time_ms) <= REMOTE_RADIO_LEASE_MS); // 独立遥控驱动测试覆盖真实最终复核。
}
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

static void receive_with_encoder(DJIMotorInstance *motor, uint32_t cycles, uint32_t tick_ms,
                                 int16_t rpm, int16_t current, uint16_t encoder)
{
    CANInstance *can = (CANInstance *)motor->motor_can_port.backend; // 从测试端点注入底层帧。
    memset(can->rx_buff, 0, sizeof(can->rx_buff));
    can->rx_buff[0] = (uint8_t)(encoder >> 8u); // 单圈编码器高字节。
    can->rx_buff[1] = (uint8_t)encoder; // 单圈编码器低字节。
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

static void receive(DJIMotorInstance *motor, uint32_t cycles, uint32_t tick_ms,
                    int16_t rpm, int16_t current)
{
    receive_with_encoder(motor, cycles, tick_ms, rpm, current, 0u); // 其余用例保留零编码器输入。
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

// 在线监测尚未超时的情况下，控制计算也必须遵守反馈自身的有效期。
static int control_rejects_expired_feedback(void)
{
    Motor_Init_Config_s config = motor_config(1u); // 只在主机使用这组测试电机参数。
    DJIMotorInstance *motor = DJIMotorInit(&config); // 使用生产初始化和控制代码。
    TEST_CHECK(motor != NULL); // 确认后续测试有可用实例。
    receive(motor, 0u, 0u, 0, 0); // 首帧的实际接收时间是 0 ms。
    DJIMotorSetRef(motor, 10.0f); // 提供能产生非零输出的速度目标。
    DJIMotorEnable(motor); // 允许测试实例参加控制计算。
    control_at(1000u); // 首周期建立计算时钟。
    control_at(2000u); // 第二周期开始计算。
    TEST_CHECK(sent[0] != 0u || sent[1] != 0u); // 新鲜反馈允许闭环输出。
    control_at((uint64_t)DJI_FEEDBACK_MAX_AGE_MS * 1000u); // 恰好到达有效期边界。
    TEST_CHECK(sent[0] != 0u || sent[1] != 0u); // 边界仍属于有效范围。
    TEST_CHECK(daemon_online != 0u); // 模拟离线监测尚未发现缺帧。
    control_at(((uint64_t)DJI_FEEDBACK_MAX_AGE_MS + 1u) * 1000u); // 超过反馈期限。
    TEST_CHECK(sent[0] == 0u && sent[1] == 0u); // 不能继续用旧反馈计算输出。
    return EXIT_SUCCESS;
}

#if defined(ONE_BOARD)
// 两台生产电机、整车安全管理和协调器一起运行；只替换总线及遥控硬件。
static int recovery_coordinator_real_motors(void)
{
    DJIMotorInstance *motors[2] = {0}; // 两台测试电机都有独立身份。
    RobotCanRecoveryCoordinator_t coordinator = {0}; // 模拟已确认的隔离测试配置。
    coordinator.enabled = 1u;
    coordinator.motors.count = 2u;
    for (uint8_t i = 0u; i < 2u; ++i)
    {
        Motor_Init_Config_s config = motor_config((uint8_t)(i + 1u)); // 每台采用不同的测试编号。
        config.output_role = MOTOR_OUTPUT_CHASSIS;
        config.can_init_config.can_bus = PLATFORM_CAN_BUS_1;
        motors[i] = DJIMotorInit(&config); // 使用真实电机注册和接收处理。
        TEST_CHECK(motors[i] != NULL);
        coordinator.motors.motors[i] = (RobotCanMotorRequirement_t){
            .bus = PLATFORM_CAN_BUS_1, .role = MOTOR_OUTPUT_CHASSIS, .type = M3508, .id = i + 1u};
        receive(motors[i], 480000u, 1u, 0, 0); // 故障前的旧反馈不能用于恢复。
    }
    const CanRecoveryConfig_t timing = {3u, 100u}; // 仅在主机使用的候选时间。
    TEST_CHECK(CanRecoveryTimedInit(&coordinator.timing, &timing));
    now_us = 1000u;
    RobotSafetyInit(); RobotSafetySetRcOnline(1u); RobotSafetySetImuValid(1u);
    RobotSafetySetCalibValid(1u); RobotSafetySetTaskHealthy(1u);
    RobotSafetyRequestEnable(); RobotSafetyUpdate(); // 建立人工有效使能。
    RobotModulePlan_t plan = {0};
    plan.enabled = plan.required = ROBOT_MODULE_BIT(ROBOT_MODULE_CHASSIS);
    const RobotCanFilterDemand_t demand = {ROBOT_MODULE_CHASSIS, PLATFORM_CAN_BUS_1, 2u, 0u};
    TEST_CHECK(RobotCanRuntimeInstall(&plan, &demand, 1u)); // 使用生产故障观察者。
    recovery_fault_observer(PLATFORM_CAN_BUS_1, recovery_fault_context); // 故障立即撤销整车许可。
    RemoteControlSnapshot_s remote = {.remote_online = 1u, .radio_sequence = 1u,
        .radio_source_time_ms = 1u, .source_selection_generation = 1u};
    remote.input.rc.switch_left = RC_SW_DOWN;
    MessageFreshness_t freshness = {.valid = 1u, .timestamp_ms = 1u};
    TEST_CHECK(!RobotCanRecoveryCoordinatorStep(&coordinator, &remote, 0u, &freshness)); // 总线未恢复先记录基准。
    recovery_bus_ok = 1u; // 测试模拟控制器重新可用。
    for (uint32_t ms = 2u; ms <= 102u; ms += 10u)
    {
        now_us = (uint64_t)ms * 1000u;
        for (unsigned i = 0u; i < 2u; ++i) receive(motors[i], ms * 480000u, ms, 0, 0);
        remote.radio_source_time_ms = ms; ++remote.radio_sequence; // 每轮交付真实意义上的新遥控帧。
        freshness.timestamp_ms = ms; // 云台反馈同步保持有效。
        TEST_CHECK(!RobotCanRecoveryCoordinatorStep(&coordinator, &remote, 0u, &freshness)); // 稳定结束时仍不得使用旧命令。
    }
    now_us = 103000u; remote.radio_source_time_ms = 103u; ++remote.radio_sequence;
    freshness.timestamp_ms = 103u;
    TEST_CHECK(RobotCanRecoveryCoordinatorStep(&coordinator, &remote, 0u, &freshness)); // 稳定后的新命令允许尝试解除。
    TEST_CHECK((RobotSafetyGetFaults() & ROBOT_FAULT_CAN_BUSOFF) == 0u); // 真正的安全管理已清除故障。
    return EXIT_SUCCESS;
}

// 将测试命令的原始时间交给生产恢复检查，序号和反馈由调用方分别维护。
static uint8_t recovery_timed_step(CanRecoveryTimed_t *state, const CanRecoveryInput_t *input,
                                   uint32_t source_ms)
{
    const CanRecoveryTimedInput_t timed = {.core = *input, .command_source_id = 1u,
        .command_source_ms = source_ms, .command_max_age_ms = 200u}; // 测试来源期限与电机反馈期限分开。
    return CanRecoveryTimedStep(state, &timed); // 原始时间和序号都要通过。
}

// 用两条真实驱动反馈推进恢复；不使用机构汇总时间冒充每台电机的更新。
static int recovery_real_feedback(void)
{
    DJIMotorInstance *motors[2] = {0}; // 两台电机使用固定登记顺序。
    for (uint8_t i = 0u; i < 2u; ++i)
    {
        Motor_Init_Config_s config = motor_config((uint8_t)(i + 1u)); // 分配不同设备编号。
        config.output_role = MOTOR_OUTPUT_CHASSIS; // 本场景检查两台底盘电机。
        config.can_init_config.can_bus = i == 0u ? PLATFORM_CAN_BUS_1 : PLATFORM_CAN_BUS_2; // 分别记录实际总线。
        motors[i] = DJIMotorInit(&config); // 使用生产驱动登记。
        TEST_CHECK(motors[i] != NULL); // 登记失败时不能继续。
        receive(motors[i], 480000u, 1u, 0, 0); // 两台均在故障前有反馈，周期计数与 480 MHz 时钟对应。
    }
    Motor_Init_Config_s optional = motor_config(3u); // 增加一台尚无反馈的非核心测试设备。
    optional.output_role = MOTOR_OUTPUT_FRICTION; // 明确区分它的用途。
    optional.can_init_config.can_bus = PLATFORM_CAN_BUS_2; // 与第二台共享测试总线但使用不同设备编号。
    TEST_CHECK(DJIMotorInit(&optional) != NULL); // 该设备虽未收到帧，也必须出现在登记列表。
    now_us = 1000u; // 与最早反馈使用相同时间域。
    DJI_RecoveryFeedback_s feedback = {0}; // 收集每台电机的原始信息。
    TEST_CHECK(DJIMotorGetRecoveryFeedback(1u, &feedback) && feedback.count == 3u); // 全部已登记设备都必须出现。
    TEST_CHECK(feedback.motors[0].bus == PLATFORM_CAN_BUS_1 && feedback.motors[1].bus == PLATFORM_CAN_BUS_2); // 总线绑定不能混淆。
    TEST_CHECK(feedback.motors[0].sequence == 1u && feedback.motors[1].sequence == 1u); // 初始反馈各自保存序号。
    const RobotCanMotorPlan_t identity = {.count = 2u, .motors = {
        {.bus = PLATFORM_CAN_BUS_1, .role = MOTOR_OUTPUT_CHASSIS, .type = M3508, .id = 1u},
        {.bus = PLATFORM_CAN_BUS_2, .role = MOTOR_OUTPUT_CHASSIS, .type = M3508, .id = 2u}}}; // 两台必需电机按车型声明身份。
    CanRecoveryInput_t mapped = {0}; // 非核心第三台不会代替必需电机。
    uint32_t registered_mask = 0u; // 最终复核使用实际登记下标。
    TEST_CHECK(RobotCanMotorRecoveryMapWithMask(&identity, &feedback, 1u, &mapped, &registered_mask) &&
               mapped.valid_feedback_mask == 3u && mapped.feedback_sequence[0] == 1u &&
               mapped.feedback_sequence[1] == 1u); // 用生产驱动数据核对型号、编号、总线和来源序号。
    TEST_CHECK(registered_mask == 3u && DJIMotorRecoveryStillValid(&feedback, registered_mask, 1u));
    TEST_CHECK(!DJIMotorRecoveryStillValid(&feedback, registered_mask, 22u)); // 后台尚未巡检时也按原始时间拒绝过期样本。
    TEST_CHECK(feedback.motors[2].bus == PLATFORM_CAN_BUS_2 && feedback.motors[2].sequence == 0u &&
               !feedback.motors[2].speed_valid && !feedback.motors[2].position_valid); // 无首帧的设备不能被标记为有效。
    CanRecoveryTimed_t recovery = {0}; // 测试实例独立持有反馈与命令时间。
    const CanRecoveryConfig_t config = {3u, 100u}; // 测试用户选择的 100 ms 候选稳定时间，不代表实车确认。
    TEST_CHECK(CanRecoveryTimedInit(&recovery, &config)); // 验证测试配置有效。
    RobotSafetyInit(); // 由真实安全管理维护使能和故障。
    RobotSafetySetRcOnline(1u); RobotSafetySetImuValid(1u);
    RobotSafetySetCalibValid(1u); RobotSafetySetTaskHealthy(1u);
    RobotSafetyRequestEnable(); RobotSafetyUpdate();
    RobotModulePlan_t core = {0}; // 两条总线都属于测试车型的核心底盘。
    core.enabled = core.required = ROBOT_MODULE_BIT(ROBOT_MODULE_CHASSIS);
    const RobotCanFilterDemand_t demands[] = {
        {ROBOT_MODULE_CHASSIS, PLATFORM_CAN_BUS_1, 1u, 0u},
        {ROBOT_MODULE_CHASSIS, PLATFORM_CAN_BUS_2, 1u, 0u}};
    TEST_CHECK(RobotCanRuntimeInstall(&core, demands, 2u)); // 安装生产核心总线观察者。
    recovery_fault_observer(PLATFORM_CAN_BUS_1, recovery_fault_context); // 真实观察者记录 CAN 故障。
    RobotCanReadiness_t buses = {0};
    TEST_CHECK(RobotCanRuntimeGetReadiness(&buses) && buses.ready_bus_mask == 0u); // 总线未恢复不能放行。
    RobotSafetySnapshot_t safety = {0};
    TEST_CHECK(RobotSafetyGetSnapshot(&safety) && RobotSafetyCanRecover(safety.control_generation));
    CanRecoveryInput_t input = {.now_ms = 1u, .inhibit_generation = buses.fault_generation,
        .permit_ok = 1u, .command_sequence = 10u}; // 故障中先记录两台的基准。
    TEST_CHECK(RobotCanMotorRecoveryMap(&identity, &feedback, 1u, &input)); // 原序号和有效性经过生产身份映射。
    TEST_CHECK(!recovery_timed_step(&recovery, &input, 1u)); // 总线故障期间必须禁止。
    now_us = 2000u; receive(motors[0], 960000u, 2u, 0, 0); // 只有第一台收到新反馈。
    TEST_CHECK(DJIMotorRecoveryStillValid(&feedback, registered_mask, 2u)); // 连续更新且没有间断时可以检查更新后的反馈。
    TEST_CHECK(DJIMotorGetRecoveryFeedback(2u, &feedback)); // 重新读取两台实际反馈。
    recovery_bus_ok = 1u; // 两条测试总线均已恢复。
    TEST_CHECK(RobotCanRuntimeGetReadiness(&buses) && buses.ready_bus_mask == buses.core_bus_mask);
    input.now_ms = 2u; input.bus_ok = 1u; // 测试总线恢复。
    TEST_CHECK(RobotCanMotorRecoveryMap(&identity, &feedback, 2u, &input)); // 原序号不会因汇总而递增。
    TEST_CHECK(!recovery_timed_step(&recovery, &input, 1u)); // 第二台仍是旧样本，不能开始稳定计时。
    now_us = 3000u; receive(motors[1], 1440000u, 3u, 0, 0); // 第二台也收到新反馈。
    TEST_CHECK(DJIMotorGetRecoveryFeedback(3u, &feedback)); // 读取各自新序号。
    input.now_ms = 3u; TEST_CHECK(RobotCanMotorRecoveryMap(&identity, &feedback, 3u, &input)); // 更新各路原始序号及有效性。
    TEST_CHECK(!recovery_timed_step(&recovery, &input, 1u)); // 全部新反馈到齐后开始稳定计时。
    for (uint32_t time_ms = 13u; time_ms <= 103u; time_ms += 10u)
    {
        now_us = (uint64_t)time_ms * 1000u; // 每隔 10 ms 更新测试时钟。
        for (unsigned i = 0u; i < 2u; ++i)
            receive(motors[i], time_ms * 480000u, time_ms, 0, 0); // 两台电机必须持续产生新反馈。
        TEST_CHECK(DJIMotorGetRecoveryFeedback(time_ms, &feedback)); // 取得当前真实驱动数据。
        input.now_ms = time_ms; TEST_CHECK(RobotCanMotorRecoveryMap(&identity, &feedback, time_ms, &input)); // 按各自原始时间检查有效性。
        TEST_CHECK(!recovery_timed_step(&recovery, &input, 1u)); // 包括 100 ms 边界在内，仍等待之后的新命令。
    }
    input.now_ms = 104u; input.command_sequence = 11u; TEST_CHECK(recovery_timed_step(&recovery, &input, 104u)); // 稳定后的新命令才使恢复条件成立。
    now_us = 104000u; // 最终安全检查使用同一真实测试时钟。
    TEST_CHECK(RobotCanRuntimeGetReadiness(&buses) && buses.ready_bus_mask == buses.core_bus_mask);
    TEST_CHECK(RobotSafetyGetSnapshot(&safety) && RobotSafetyCanRecover(safety.control_generation));
    recovery_fault_observer(PLATFORM_CAN_BUS_1, recovery_fault_context); // 最终清除前又出现短故障。
    TEST_CHECK(!RobotCanRuntimeTryClearFault(buses.fault_generation, safety.control_generation, 104u)); // 旧总线代次不能清除新故障。
    TEST_CHECK(RobotCanRuntimeGetReadiness(&buses)); // 新一轮必须使用新的故障代次。
    input.inhibit_generation = buses.fault_generation; // 恢复状态机观察到这次短故障。
    TEST_CHECK(!recovery_timed_step(&recovery, &input, 104u)); // 新故障要求重新验证所有反馈。
    for (uint32_t time_ms = 105u; time_ms <= 205u; time_ms += 10u)
    {
        now_us = (uint64_t)time_ms * 1000u; // 使用真实测试时钟推进第二次等待。
        for (unsigned i = 0u; i < 2u; ++i)
            receive(motors[i], time_ms * 480000u, time_ms, 0, 0); // 两台电机重新提供连续反馈。
        TEST_CHECK(DJIMotorGetRecoveryFeedback(time_ms, &feedback));
        input.now_ms = time_ms;
        TEST_CHECK(RobotCanMotorRecoveryMap(&identity, &feedback, time_ms, &input));
        TEST_CHECK(!recovery_timed_step(&recovery, &input, 104u)); // 稳定期结束后仍等待新命令。
    }
    now_us = 206000u;
    input.now_ms = 206u; input.command_sequence = 12u; // 第二次恢复后使用更新的命令。
    TEST_CHECK(recovery_timed_step(&recovery, &input, 206u));
    TEST_CHECK(RobotCanRuntimeGetReadiness(&buses) && RobotSafetyGetSnapshot(&safety));
    TEST_CHECK(RobotCanRuntimeTryClearFault(buses.fault_generation, safety.control_generation, 206u)); // 真实故障代次与资格通过后才清除。
    TEST_CHECK((RobotSafetyGetFaults() & ROBOT_FAULT_CAN_BUSOFF) == 0u); // 故障位已解除，但仍需状态机放行。
    now_us = 227000u; TEST_CHECK(DJIMotorGetRecoveryFeedback(227u, &feedback)); // 原始反馈超过 20 ms。
    TEST_CHECK(!feedback.motors[0].speed_valid && !feedback.motors[1].speed_valid); // 重复汇总不能刷新反馈期限。
    input.now_ms = 227u; TEST_CHECK(RobotCanMotorRecoveryMap(&identity, &feedback, 227u, &input)); // 从电机结果读取过期状态。
    TEST_CHECK(!recovery_timed_step(&recovery, &input, 206u)); // 反馈过期立即撤销恢复结果。
    const uint32_t prior_gap = feedback.motors[0].gap_generation; // 记录断流前的间断代次。
    now_us = 228000u; receive(motors[0], 228u * 480000u, 228u, 0, 0); // 反馈超过 20 ms 后重新到达。
    TEST_CHECK(DJIMotorGetRecoveryFeedback(228u, &feedback)); // 通过生产驱动再次读取。
    TEST_CHECK(feedback.motors[0].gap_generation != prior_gap); // 两次检查之间断流也会留下记录。
    return EXIT_SUCCESS;
}

// 发射机构可运行但击发已撤销时，拨弹电机不能借摩擦轮许可继续输出。
static int single_motion_reference_generation(void)
{
    now_us = 1000u; RobotSafetyInit(); // 设置可运行的单板整车条件。
    RobotSafetySetRcOnline(1u); RobotSafetySetImuValid(1u);
    RobotSafetySetCalibValid(1u); RobotSafetySetTaskHealthy(1u);
    RobotSafetyRequestEnable(); RobotSafetyUpdate(); // 旧整车路径也必须保护运动参考。
    RobotSafetySnapshot_t before = {0}; // 故障前的命令携带此代际。
    TEST_CHECK(RobotSafetyGetSnapshot(&before) && RobotSafetyIsReady());
    DJIMotorInstance *motors[2] = {0}; // 底盘和云台共用一个发送帧。
    for (uint8_t i = 0u; i < 2u; ++i)
    {
        Motor_Init_Config_s config = motor_config((uint8_t)(i + 1u));
        config.output_role = i == 0u ? MOTOR_OUTPUT_CHASSIS : MOTOR_OUTPUT_GIMBAL;
        motors[i] = DJIMotorInit(&config);
        TEST_CHECK(motors[i] != NULL);
        receive(motors[i], 1000u, 1u, 0, 0); // 两台电机都具备新鲜反馈。
        TEST_CHECK(DJIMotorSetRefForGeneration(motors[i], 10.0f, before.control_generation));
        DJIMotorEnable(motors[i]);
    }
    control_at(2000u); control_at(3000u); // 初始化计算间隔后生成非零输出。
    TEST_CHECK((sent[0] || sent[1]) && (sent[2] || sent[3]));
    RobotFaultReport(ROBOT_FAULT_CAN_BUSOFF); // 在两个电机周期之间发生并恢复故障。
    RobotFaultClear(ROBOT_FAULT_CAN_BUSOFF); RobotSafetyUpdate();
    TEST_CHECK(RobotSafetyIsReady());
    TEST_CHECK(!DJIMotorSetRefForGeneration(motors[0], 20.0f, before.control_generation));
    control_at(4000u); // 底盘和云台任务尚未刷新参考。
    TEST_CHECK(sent[0] == 0u && sent[1] == 0u && sent[2] == 0u && sent[3] == 0u);
    RobotSafetySnapshot_t after = {0}; // 新命令必须绑定恢复后的代际。
    TEST_CHECK(RobotSafetyGetSnapshot(&after));
    TEST_CHECK(DJIMotorSetRefForGeneration(motors[0], 10.0f, after.control_generation));
    TEST_CHECK(DJIMotorSetRefForGeneration(motors[1], 10.0f, after.control_generation));
    control_at(5000u); // 新参考才可继续计算。
    TEST_CHECK((sent[0] || sent[1]) && (sent[2] || sent[3]));
    return EXIT_SUCCESS;
}

static int single_loader_fire_permit(void)
{
    now_us = 1000u; RobotSafetyInit(); // 从初始安全状态建立测试条件。
    RobotSafetySetRcOnline(1u); RobotSafetySetImuValid(1u); // 人工输入和姿态有效。
    RobotSafetySetCalibValid(1u); RobotSafetySetTaskHealthy(1u); // 标定和任务有效。
    const RobotControlPolicy_t policy = {ROBOT_CONTROL_ALL, 1u, 0u, 0u}; // 测试分功能人工策略。
    TEST_CHECK(RobotSafetyConfigureControl(&policy)); // 在初始状态配置许可策略。
    RobotSafetyRequestEnable(); RobotSafetyUpdate(); // 请求使能并建立来源起点。
    now_us = 2000u; // 新输入晚于使能起点。
    RobotSafetyControlInputs_t input = {0}; // 用明确的测试事实填充策略输入。
    input.mode = ROBOT_CONTROL_MANUAL; // 人工决定击发。
    input.manual = (RobotControlLease_t){2u, 100u, 1u}; // 人工来源有独立期限。
    for (unsigned i = 0u; i < 3u; ++i) input.feedback[i] = (RobotControlLease_t){2u, 100u, 1u}; // 三组反馈均有效。
    input.feedback_valid_functions = ROBOT_CONTROL_ALL; // 声明所有测试机构具备反馈。
    input.manual_fire_requested = 1u; // 初始允许人工击发。
    TEST_CHECK(RobotSafetyPublishControlInputs(&input)); RobotSafetyUpdate(); // 发布本轮许可。
    RobotSafetySnapshot_t initial_safety = {0}; // 保存故障前的代际供后续拒绝测试。
    TEST_CHECK(RobotSafetyGetSnapshot(&initial_safety)); // 初始安全状态必须可读。
    DJIMotorInstance *motors[2] = {0}; // 两台电机共享一个 CAN 帧。
    for (uint8_t i = 0u; i < 2u; ++i)
    {
        Motor_Init_Config_s config = motor_config((uint8_t)(i + 1u)); // 使用独立测试设备编号。
        config.output_role = i == 0u ? MOTOR_OUTPUT_FRICTION : MOTOR_OUTPUT_LOADER; // 分别声明摩擦轮和拨弹。
        motors[i] = DJIMotorInit(&config); // 走实际电机注册路径。
        TEST_CHECK(motors[i] != NULL); // 注册失败不能继续测试。
        receive(motors[i], 2000u, 2u, 0, 0); // 提供新鲜电机反馈。
        if (i == 0u) DJIMotorSetRef(motors[i], 10.0f); // 摩擦轮使用普通参考。
        else
        {
            RobotSafetySnapshot_t safety = {0}; // 拨弹参考必须绑定当前许可代际。
            TEST_CHECK(RobotSafetyGetSnapshot(&safety)); // 保存当前安全代际。
            TEST_CHECK(DJIMotorSetRefForGeneration(motors[i], 10.0f, safety.control_generation)); // 模拟发射任务提交授权参考。
        }
        DJIMotorEnable(motors[i]); // 允许电机参加计算。
    }
    control_at(3000u); control_at(4000u); // 建立 PID 计时并产生非零输出。
    TEST_CHECK((sent[0] || sent[1]) && (sent[2] || sent[3])); // 初始许可允许两个机构运行。
    now_us = 5000u; input.manual_fire_requested = 0u; // 人工撤销击发，摩擦轮仍获机构许可。
    TEST_CHECK(RobotSafetyPublishControlInputs(&input)); RobotSafetyUpdate(); // 发射任务尚未运行。
    TEST_CHECK(!DJIMotorSetRefForGeneration(motors[1], 20.0f, initial_safety.control_generation)); // 旧代际不能趁撤销时写入参考。
    control_at(6000u); // 电机任务先于发射任务运行。
    TEST_CHECK(sent[0] || sent[1]); // 摩擦轮可继续运转。
    TEST_CHECK(sent[2] == 0u && sent[3] == 0u); // 旧拨弹参考不能借机构许可继续输出。
    now_us = 7000u; input.manual_fire_requested = 1u; input.manual.source_time_ms = 7u; // 恢复后的新人工来源允许再次击发。
    TEST_CHECK(RobotSafetyPublishControlInputs(&input)); RobotSafetyUpdate(); // 安全管理接受新许可。
    TEST_CHECK(!DJIMotorSetRefForGeneration(motors[1], 20.0f, initial_safety.control_generation)); // 许可恢复也不能使旧代际重新有效。
    control_at(8000u); // 发射任务仍未刷新拨弹参考。
    TEST_CHECK(sent[0] || sent[1]); // 摩擦轮继续运行。
    TEST_CHECK(sent[2] == 0u && sent[3] == 0u); // 新许可不能激活故障前留下的旧参考。
    RobotSafetySnapshot_t renewed = {0}; // 重新读取恢复后的安全代际。
    TEST_CHECK(RobotSafetyGetSnapshot(&renewed)); // 只有当前代际可用于新参考。
    TEST_CHECK(!DJIMotorSetRefForGeneration(motors[0], 10.0f, renewed.control_generation)); // 摩擦轮不能误用拨弹接口。
    TEST_CHECK(!DJIMotorSetRefForGeneration(motors[1], NAN, renewed.control_generation)); // 非有限参考必须拒绝。
    TEST_CHECK(DJIMotorSetRefForGeneration(motors[1], 10.0f, renewed.control_generation)); // 模拟发射任务提交新的拨弹参考。
    control_at(9000u); // 电机任务计算新代际参考。
    TEST_CHECK(sent[2] || sent[3]); // 新许可与新参考同时存在时才输出。
    return EXIT_SUCCESS;
}

// 发送前检查必须使用计算旧目标时的反馈时间，不能被后来的反馈续期。
static int single_queued_feedback_expiry(void)
{
    now_us = 1000u; // 建立单板核心条件时使用同一本机时钟。
    RobotSafetyInit(); // 初始状态禁止输出。
    RobotSafetySetRcOnline(1u); // 测试遥控输入在线。
    RobotSafetySetImuValid(1u); // 测试姿态有效。
    RobotSafetySetCalibValid(1u); // 测试标定有效。
    RobotSafetySetTaskHealthy(1u); // 测试核心任务正常。
    RobotSafetyRequestEnable(); // 申请整车使能。
    RobotSafetyUpdate(); // 发布本次测试的整车许可。
    Motor_Init_Config_s config = motor_config(1u); // 使用隔离的主机电机配置。
    DJIMotorInstance *motor = DJIMotorInit(&config); // 注册一台实际驱动实例。
    TEST_CHECK(motor != NULL); // 注册失败不能继续。
    receive(motor, 2000u, 2u, 0, 0); // 电机反馈在 2 ms 接收。
    DJIMotorSetRef(motor, 10.0f); // 请求非零速度。
    DJIMotorEnable(motor); // 允许测试电机参加计算。
    control_at(3000u); // 建立 PID 计算时间。
    control_at(4000u); // 根据 2 ms 反馈计算并入队。
    TEST_CHECK(deferred_prepare != NULL && (deferred_payload[0] || deferred_payload[1])); // 队列保存了非零目标。
    uint8_t original[8]; // 保存队列副本，供两次发送前检查使用。
    memcpy(original, deferred_payload, sizeof(original)); // 记录计算时生成的旧目标。
    now_us = 22000u; // 原始反馈恰好达到 20 ms。
    deferred_prepare(deferred_source, HAL_GetTick(), deferred_payload, sizeof(deferred_payload)); // 模拟硬件提交前检查。
    TEST_CHECK(deferred_payload[0] || deferred_payload[1]); // 有效期边界仍可使用。
    now_us = 23000u; // 原始反馈已经超过期限。
    receive(motor, 23000u, 23u, 0, 0); // 新反馈到达也不能改变旧目标的计算依据。
    memcpy(deferred_payload, original, sizeof(original)); // 再次模拟检查同一个旧队列目标。
    deferred_prepare(deferred_source, HAL_GetTick(), deferred_payload, sizeof(deferred_payload)); // 按旧目标的来源时刻检查。
    TEST_CHECK(deferred_payload[0] == 0u && deferred_payload[1] == 0u); // 旧目标不能借新反馈续期。
    control_at(24000u); // 根据 23 ms 反馈生成新的队列目标。
    TEST_CHECK(deferred_payload[0] || deferred_payload[1]); // 确认新目标已经进入队列。
    now_us = 26000u; // 模拟后续反馈和一次排队替换。
    receive(motor, 26000u, 26u, 0, 0); // 提供比队列目标更新的反馈。
    reject_periodic = 1u; // 队列拒绝替换，保留 23 ms 的旧目标。
    control_at(26000u); // 驱动尝试提交新的控制值。
    reject_periodic = 0u; // 恢复测试队列行为。
    now_us = 44000u; // 23 ms 的反馈已过期，26 ms 反馈仍在有效期内。
    deferred_prepare(deferred_source, HAL_GetTick(), deferred_payload, sizeof(deferred_payload)); // 检查实际保留的旧目标。
    TEST_CHECK(deferred_payload[0] == 0u && deferred_payload[1] == 0u); // 替换失败不能用较新的反馈给旧目标续期。
    return EXIT_SUCCESS;
}

// 单板底盘命令即使在新反馈不断到来时也只能使用到自己的截止点。
static int single_chassis_reference_expiry(void)
{
    const uint64_t wrap_us = (UINT64_C(1) << 32u) * UINT64_C(1000); // 转为 32 位毫秒时在这里回到零。
    now_us = wrap_us - 6000u; // 在回绕前准备许可和电机。
    RobotSafetyInit();
    RobotSafetySetRcOnline(1u);
    RobotSafetySetImuValid(1u);
    RobotSafetySetCalibValid(1u);
    RobotSafetySetTaskHealthy(1u);
    RobotSafetyRequestEnable();
    RobotSafetyUpdate();
    RobotSafetySnapshot_t safety;
    TEST_CHECK(RobotSafetyGetSnapshot(&safety));
    Motor_Init_Config_s config = motor_config(1u); // 编号 1 对应测试发送帧中的前两字节。
    config.output_role = MOTOR_OUTPUT_CHASSIS; // 只测试底盘轮电机。
    DJIMotorInstance *motor = DJIMotorInit(&config);
    TEST_CHECK(motor != NULL && DJIMotorSetChassisRefUntil(motor, 10.0f, safety.control_generation, 0u));
    DJIMotorEnable(motor);
    now_us = wrap_us - 4000u;
    receive(motor, 2000u, HAL_GetTick(), 0, 0); // 当前反馈比原命令更晚到期。
    control_at(wrap_us - 3000u); // 首轮建立 PID 计时。
    control_at(wrap_us - 2000u); // 原命令仍有效，生成非零 CAN 目标。
    TEST_CHECK(deferred_prepare != NULL && (deferred_payload[0] || deferred_payload[1]));
    uint8_t original[8];
    memcpy(original, deferred_payload, sizeof(original)); // 模拟发送队列保留的旧副本。

    now_us = wrap_us;
    deferred_prepare(deferred_source, HAL_GetTick(), deferred_payload, sizeof(deferred_payload));
    TEST_CHECK(deferred_payload[0] || deferred_payload[1]); // 截止时刻零仍可使用。
    now_us = wrap_us + 1000u;
    receive(motor, 2000u + 5u * 480000u, HAL_GetTick(), 0, 0); // 新帧比上一帧晚 5 ms，按 480 MHz 注入周期数。
    Motor_Feedback_Snapshot_s feedback;
    TEST_CHECK(DJIMotorGetFeedbackSnapshot(motor, HAL_GetTick(), &feedback) == MOTOR_FEEDBACK_OK &&
        feedback.speed.valid && feedback.current.valid); // 排除因反馈失效而清零的情况。
    memcpy(deferred_payload, original, sizeof(original));
    deferred_prepare(deferred_source, HAL_GetTick(), deferred_payload, sizeof(deferred_payload));
    TEST_CHECK(deferred_payload[0] == 0u && deferred_payload[1] == 0u); // 队列发送也必须按原命令到期。
    TEST_CHECK(!DJIMotorSetChassisRefUntil(motor, 10.0f, safety.control_generation, 0u)); // 重复提交过期命令也会失败。
    control_at(wrap_us + 2000u);
    TEST_CHECK(sent[0] == 0u && sent[1] == 0u); // 电机计算不能继续用旧参考。

    TEST_CHECK(DJIMotorSetChassisRefUntil(motor, 10.0f, safety.control_generation, 5u)); // 活跃许可不变，接受新的命令期限。
    control_at(wrap_us + 3000u);
    TEST_CHECK(sent[0] || sent[1]);
    return EXIT_SUCCESS;
}

// 即使同组电机仍在线，一台电机的短暂测量异常也必须清掉它的排队目标。
// 验证底盘上限由配置传到最终 CAN 字节，和 PID 请求的大小独立。
static int single_chassis_output_limit(void)
{
    now_us = 1000u;
    RobotSafetyInit();
    RobotSafetySetRcOnline(1u);
    RobotSafetySetImuValid(1u);
    RobotSafetySetCalibValid(1u);
    RobotSafetySetTaskHealthy(1u);
    RobotSafetyRequestEnable();
    RobotSafetyUpdate();
    RobotSafetySnapshot_t safety;
    TEST_CHECK(RobotSafetyGetSnapshot(&safety));

    Motor_Init_Config_s config = motor_config(1u); // 使用测试速度环和电流环，原始输出范围大于测试上限。
    config.output_role = MOTOR_OUTPUT_CHASSIS;
    config.dji_chassis_output_limit_raw = DJI_OUTPUT_MAX_RAW + 1u;
    TEST_CHECK(DJIMotorInit(&config) == NULL && registered == 0u); // 范围非法时不得登记 CAN 端点。
    config.output_role = MOTOR_OUTPUT_UNASSIGNED;
    config.dji_chassis_output_limit_raw = 5u; // 只用于测试报文的原始整数上限。
    TEST_CHECK(DJIMotorInit(&config) == NULL && registered == 0u); // 其他角色不能误接底盘配置字段。
    config.output_role = MOTOR_OUTPUT_CHASSIS;
    DJIMotorInstance *first = DJIMotorInit(&config);
    config.can_init_config.tx_id = 2u; // 第二个轮位与第一轮共用发送帧。
    config.output_axis.direction = MOTOR_AXIS_DIRECTION_REVERSE; // 正轮速在反装轮位应编码为负值。
    DJIMotorInstance *second = DJIMotorInit(&config);
    TEST_CHECK(first != NULL && second != NULL && registered == 2u);
    TEST_CHECK(DJIMotorSetChassisRefUntil(first, 40.0f, safety.control_generation, 20u));
    TEST_CHECK(DJIMotorSetChassisRefUntil(second, 40.0f, safety.control_generation, 20u));
    DJIMotorEnable(first);
    DJIMotorEnable(second);
    receive(first, 960000u, 2u, 0, 0);
    receive(second, 960000u, 2u, 0, 0);
    control_at(3000u); // 首轮建立 PID 时间基准。
    control_at(4000u); // 速度和电流闭环完成后，最终报文在两方向应用同一上限。
    TEST_CHECK(sent[0] == 0x00u && sent[1] == 0x05u);
    TEST_CHECK(sent[2] == 0xffu && sent[3] == 0xfbu);

    TEST_CHECK(DJIMotorSetChassisRefUntil(first, 3.0f, safety.control_generation, 20u));
    control_at(5000u);
    TEST_CHECK(sent[0] == 0x00u && sent[1] == 0x03u); // 在上限内保持原输出。
    return EXIT_SUCCESS;
}

// 在同一个整车安全代际内复现底盘单轮停止并重新启用。
static int single_chassis_queued_stop(void)
{
    now_us = 1000u;
    RobotSafetyInit();
    RobotSafetySetRcOnline(1u);
    RobotSafetySetImuValid(1u);
    RobotSafetySetCalibValid(1u);
    RobotSafetySetTaskHealthy(1u);
    RobotSafetyRequestEnable();
    RobotSafetyUpdate();
    RobotSafetySnapshot_t safety;
    TEST_CHECK(RobotSafetyGetSnapshot(&safety));

    Motor_Init_Config_s config = motor_config(1u); // 两台底盘电机共享一个发送帧。
    config.output_role = MOTOR_OUTPUT_CHASSIS;
    DJIMotorInstance *first = DJIMotorInit(&config);
    config.can_init_config.tx_id = 2u; // 第二台占用另外两个字节。
    DJIMotorInstance *second = DJIMotorInit(&config);
    TEST_CHECK(first != NULL && second != NULL);
    TEST_CHECK(DJIMotorSetChassisRefUntil(first, 10.0f, safety.control_generation, 15u));
    TEST_CHECK(DJIMotorSetChassisRefUntil(second, 9.0f, safety.control_generation, 15u));
    DJIMotorEnable(first);
    DJIMotorEnable(second);
    receive(first, 960000u, 2u, 0, 0);
    receive(second, 960000u, 2u, 0, 0);
    control_at(3000u); // 建立 PID 时间基准。
    control_at(4000u); // 生成两个仍有效的非零目标。
    TEST_CHECK(deferred_prepare != NULL && (deferred_payload[0] || deferred_payload[1]) &&
        (deferred_payload[2] || deferred_payload[3]));

    DJIMotorStop(first); // 队列尚未发送，也没有经过新的控制周期。
    DJIMotorEnable(first); // 仅改回运行标记，旧目标应保持撤销。
    now_us = 5000u;
    uint8_t queued[8]; // 保留原帧，以便再次模拟同一队列项。
    memcpy(queued, deferred_payload, sizeof(queued));
    deferred_prepare(deferred_source, HAL_GetTick(), queued, sizeof(queued));
    TEST_CHECK(!(queued[0] || queued[1]) && (queued[2] || queued[3])); // 只有被停止的轮位清零。

    reject_periodic = 1u; // 新目标算完后仍可能不能替换队列。
    TEST_CHECK(DJIMotorSetChassisRefUntil(first, 20.0f, safety.control_generation, 15u));
    control_at(6000u);
    memcpy(queued, deferred_payload, sizeof(queued));
    deferred_prepare(deferred_source, HAL_GetTick(), queued, sizeof(queued));
    TEST_CHECK(!(queued[0] || queued[1]) && (queued[2] || queued[3])); // 新参考不能重新授权旧帧。

    reject_periodic = 0u;
    control_at(7000u);
    TEST_CHECK((sent[0] || sent[1]) && (sent[2] || sent[3])); // 成功入队的新目标可以通过检查。
    DJIMotorStop(first);
    DJIMotorEnable(first);
    control_at(8000u);
    TEST_CHECK(!(sent[0] || sent[1]) && (sent[2] || sent[3])); // 光重新启用也不能再次计算旧参考。
    return EXIT_SUCCESS;
}

static int single_queued_feedback_gap(void)
{
    now_us = 1000u; // 所有安全条件使用测试本机时间。
    RobotSafetyInit();
    RobotSafetySetRcOnline(1u); // 维持整车许可，单独观察电机测量异常。
    RobotSafetySetImuValid(1u);
    RobotSafetySetCalibValid(1u);
    RobotSafetySetTaskHealthy(1u);
    RobotSafetyRequestEnable();
    RobotSafetyUpdate();
    Motor_Init_Config_s config = motor_config(1u); // 两台测试电机共享同一帧。
    DJI_Motor_Filter_Config_s filter = filter_config(DJI_MOTOR_FILTER_LEGACY_FRAME);
    filter.dt_min_s = 0.004f; // 为第一台设置 4 ms 的测试下限，便于注入短间隔错误。
    DJIMotorInstance *first = DJIMotorInitWithFilter(&config, &filter);
    config.can_init_config.tx_id = 2u; // 使用组内另一个槽位。
    DJIMotorInstance *second = DJIMotorInit(&config);
    TEST_CHECK(first != NULL && second != NULL);
    receive(first, 960000u, 2u, 0, 0); // 同时收到两台电机的首帧。
    receive(second, 960000u, 2u, 0, 0);
    DJIMotorSetRef(first, 10.0f);
    DJIMotorSetRef(second, 10.0f);
    DJIMotorEnable(first);
    DJIMotorEnable(second);
    control_at(3000u); // 首次调用只建立 PID 时间基准。
    control_at(4000u); // 用有效反馈生成两个非零目标。
    TEST_CHECK(deferred_prepare != NULL);
    uint8_t original[8];
    memcpy(original, deferred_payload, sizeof(original)); // 留存间断出现前的同一队列副本。
    TEST_CHECK((original[0] || original[1]) && (original[2] || original[3]));

    now_us = 5000u;
    receive(first, 2400000u, 5u, 0, 0); // 3 ms 间隔小于滤波下限，只有第一台测量失效。
    TEST_CHECK(!first->speed_filter_valid && !first->current_filter_valid);
    deferred_prepare(deferred_source, HAL_GetTick(), deferred_payload, sizeof(deferred_payload));
    TEST_CHECK(!(deferred_payload[0] || deferred_payload[1]) && (deferred_payload[2] || deferred_payload[3]));

    now_us = 10000u;
    receive(first, 4800000u, 10u, 0, 0); // 5 ms 间隔有效，旧目标的原始时刻也尚未过期。
    TEST_CHECK(first->speed_filter_valid && first->current_filter_valid);
    memcpy(deferred_payload, original, sizeof(original)); // 模拟失效期间完全没有发送检查。
    deferred_prepare(deferred_source, HAL_GetTick(), deferred_payload, sizeof(deferred_payload));
    TEST_CHECK(!(deferred_payload[0] || deferred_payload[1]) && (deferred_payload[2] || deferred_payload[3]));

    DJIMotorSetRef(first, 12.0f); // 发送新的测试参考和计算结果，再验证两台都能输出。
    control_at(11000u);
    TEST_CHECK((sent[0] || sent[1]) && (sent[2] || sent[3]));
    DJIMotorStop(second); // 停止操作发生在入队后，到期前仍要将该槽位置零。
    deferred_prepare(deferred_source, HAL_GetTick(), deferred_payload, sizeof(deferred_payload));
    TEST_CHECK((deferred_payload[0] || deferred_payload[1]) && !(deferred_payload[2] || deferred_payload[3]));
    TEST_CHECK(DJIMotorRebaseReference(first, HAL_GetTick()) == MOTOR_FEEDBACK_OK); // 新零点不能继承旧目标。
    deferred_prepare(deferred_source, HAL_GetTick(), deferred_payload, sizeof(deferred_payload));
    TEST_CHECK(!(deferred_payload[0] || deferred_payload[1])); // 尚未过期的旧代次目标也须撤销。
    return EXIT_SUCCESS;
}

// 测试源独立控制采样时刻，新读数不能续期已经排队的目标。
static Motor_Feedback_Status_e read_timed_test_signal(const void *context, Motor_Signal_Snapshot_s *output)
{
    *output = *(const Motor_Signal_Snapshot_s *)context;
    return MOTOR_FEEDBACK_OK;
}

static int single_queued_external_expiry(void)
{
    const uint32_t start_ms = UINT32_MAX - 3u; // 最先到期的信号跨过计数回绕。
    const uint64_t start_us = (uint64_t)start_ms * 1000u;
    now_us = start_us;
    RobotSafetyInit();
    RobotSafetySetRcOnline(1u); // 整车许可保持有效，单独检验每个计算依据的期限。
    RobotSafetySetImuValid(1u);
    RobotSafetySetCalibValid(1u);
    RobotSafetySetTaskHealthy(1u);
    RobotSafetyRequestEnable();
    RobotSafetyUpdate();

    Motor_Signal_Snapshot_s speed = {
        .value = 1.0f, // 外部速度反馈，单位 rad/s。
        .unit = MOTOR_UNIT_RAD_S,
        .reference = MOTOR_REFERENCE_EXTERNAL,
        .source_time_ms = start_ms, // 记录计算时真正使用的样本。
        .sequence = 1u,
        .valid = 1u,
    };
    Motor_Signal_Snapshot_s ff = {
        .value = 2.0f, // 电流前馈为设备侧目标量。
        .unit = MOTOR_UNIT_DEVICE_CURRENT,
        .reference = MOTOR_REFERENCE_OUTPUT_AXIS,
        .source_time_ms = start_ms, // 与速度同一时刻，期限更短。
        .sequence = 1u,
        .valid = 1u,
    };
    Motor_Init_Config_s config = motor_config(1u);
    config.controller_setting_init_config.speed_feedback_source = OTHER_FEED; // 速度环使用外部读数。
    config.controller_setting_init_config.feedforward_flag = CURRENT_FEEDFORWARD; // 电流环还依赖前馈。
    config.controller_param_init_config.other_speed_feedback = (Motor_Signal_Source_s){
        .read = read_timed_test_signal, // 测试替身交付完整样本。
        .context = &speed, // 绑定外部速度读数。
        .max_age_ms = 10u, // 速度到期晚于前馈。
        .unit = MOTOR_UNIT_RAD_S, // 速度为 rad/s。
        .reference = MOTOR_REFERENCE_EXTERNAL, // 与速度环反馈接口一致。
    };
    config.controller_param_init_config.current_feedforward = (Motor_Signal_Source_s){
        .read = read_timed_test_signal, // 复用同一只读接口。
        .context = &ff, // 绑定独立电流前馈样本。
        .max_age_ms = 5u, // 最早到期点由它决定。
        .unit = MOTOR_UNIT_DEVICE_CURRENT, // 使用电调设备量。
        .reference = MOTOR_REFERENCE_OUTPUT_AXIS, // 按统一输出轴方向给前馈。
    };
    DJIMotorInstance *motor = DJIMotorInit(&config);
    TEST_CHECK(motor != NULL);
    receive(motor, 100u, start_ms, 0, 0); // 内部电流反馈 20 ms 内仍有效。
    DJIMotorSetRef(motor, 10.0f);
    DJIMotorEnable(motor);
    control_at(start_us + 1000u); // 建立 PID 时间基准。
    control_at(start_us + 2000u); // 使用同一组有效反馈和前馈计算目标。
    TEST_CHECK(deferred_prepare != NULL && (deferred_payload[0] || deferred_payload[1]));

    now_us = start_us + 5000u; // 刚好到期，HAL 时刻已经回绕。
    deferred_prepare(deferred_source, HAL_GetTick(), deferred_payload, sizeof(deferred_payload));
    TEST_CHECK(deferred_payload[0] || deferred_payload[1]);
    ff.source_time_ms = HAL_GetTick(); // 新前馈到达，但还没有重新计算目标。
    ++ff.sequence;
    now_us = start_us + 6000u;
    deferred_prepare(deferred_source, HAL_GetTick(), deferred_payload, sizeof(deferred_payload));
    TEST_CHECK(!(deferred_payload[0] || deferred_payload[1])); // 外部速度和内部电流还新鲜，也不能续期。
    control_at(start_us + 7000u); // 新计算可使用新的前馈。
    TEST_CHECK(sent[0] || sent[1]);
    return EXIT_SUCCESS;
}

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

static int single_control_feedback_to_safety(void)
{
    now_us = 1000u; RobotSafetyInit(); // 使用真实安全管理和生产电机驱动。
    RobotSafetySetRcOnline(1u); RobotSafetySetImuValid(1u);
    RobotSafetySetCalibValid(1u); RobotSafetySetTaskHealthy(1u);
    Motor_Init_Config_s config = motor_config(1u); config.output_role = MOTOR_OUTPUT_CHASSIS;
    DJIMotorInstance *chassis = DJIMotorInit(&config);
    config.can_init_config.tx_id = 2u; config.output_role = MOTOR_OUTPUT_GIMBAL;
    DJIMotorInstance *gimbal = DJIMotorInit(&config);
    TEST_CHECK(chassis != NULL && gimbal != NULL);
    const RobotCanMotorPlan_t plan = {.count = 2u, .motors = {
        {.bus = PLATFORM_CAN_BUS_1, .role = MOTOR_OUTPUT_CHASSIS, .type = M3508, .id = 1u},
        {.bus = PLATFORM_CAN_BUS_1, .role = MOTOR_OUTPUT_GIMBAL, .type = M3508, .id = 2u}}};
    TEST_CHECK(RobotControlFeedbackStart(&plan)); // 使用固件启动绑定入口。
    TEST_CHECK(!RobotControlFeedbackStart(&plan)); // 绑定后不能再次改写设备清单。
    RobotSafetyRequestEnable(); RobotSafetyUpdate();
    TEST_CHECK(!RobotSafetyIsReady()); // 源策略未就绪时旧许可不能驱动机器。
    RobotSafetyRequestDisable(); // 恢复启动测试的初始 SAFE 条件。
    const RobotControlPolicy_t policy = {ROBOT_CONTROL_CHASSIS | ROBOT_CONTROL_GIMBAL, 1u, 0u, 0u};
    TEST_CHECK(RobotSafetyConfigureControl(&policy)); RobotSafetyRequestEnable(); RobotSafetyUpdate();
    MessageFreshness_t pose = {.valid = 1u, .source_clock = MESSAGE_CLOCK_LOCAL, .timestamp_ms = 1u};
    now_us = 2000u; receive(chassis, 960000u, 2u, 0, 0); receive(gimbal, 960000u, 2u, 0, 0);
    TEST_CHECK(RobotControlFeedbackPoll(&pose)); // 由实际采集器读取、复核并提交电机反馈。
    RobotSafetyControlInputs_t input = {.mode = ROBOT_CONTROL_MANUAL, .manual = {2u, 100u, 1u}};
    TEST_CHECK(RobotSafetyPublishControlInputs(&input)); RobotSafetyUpdate(); // 核心恢复建立新命令起点。
    now_us = 3000u; input.manual.source_time_ms = 3u;
    TEST_CHECK(RobotSafetyPublishControlInputs(&input)); RobotSafetyUpdate();
    RobotControlPermits_t permits = {0}; TEST_CHECK(RobotSafetyGetFunctionPermitsAt(3u, &permits));
    TEST_CHECK(permits.functions == (ROBOT_CONTROL_CHASSIS | ROBOT_CONTROL_GIMBAL));
    now_us = 15000u; receive(gimbal, 7200000u, 15u, 0, 0); // 云台持续有反馈，底盘暂时未更新。
    TEST_CHECK(RobotControlFeedbackPoll(&pose));
    input.manual.source_time_ms = 15u;
    TEST_CHECK(RobotSafetyPublishControlInputs(&input)); RobotSafetyUpdate();
    now_us = 24000u; receive(chassis, 11520000u, 24u, 0, 0); receive(gimbal, 11520000u, 24u, 0, 0);
    TEST_CHECK(RobotControlFeedbackPoll(&pose)); // 底盘帧虽已恢复，采集者仍报告经过的间断。
    input.feedback_valid_functions = ROBOT_CONTROL_CHASSIS; // 模拟命令侧误填可用反馈。
    input.feedback[0] = (RobotControlLease_t){24u, 100u, 1u}; // 命令数据不能替换真实底盘反馈。
    input.manual.source_time_ms = 24u;
    TEST_CHECK(RobotSafetyPublishControlInputs(&input)); RobotSafetyUpdate();
    TEST_CHECK(RobotSafetyGetFunctionPermitsAt(24u, &permits) && permits.functions == ROBOT_CONTROL_GIMBAL);
    now_us = 25000u; TEST_CHECK(RobotControlFeedbackPoll(&pose));
    TEST_CHECK(RobotSafetyPublishControlInputs(&input)); RobotSafetyUpdate();
    TEST_CHECK(RobotSafetyGetFunctionPermitsAt(25u, &permits) && permits.functions == ROBOT_CONTROL_GIMBAL); // 恢复前来源不能让底盘接管。
    now_us = 26000u; input.manual.source_time_ms = 26u;
    TEST_CHECK(RobotSafetyPublishControlInputs(&input)); RobotSafetyUpdate();
    TEST_CHECK(RobotSafetyGetFunctionPermitsAt(26u, &permits) &&
        permits.functions == (ROBOT_CONTROL_CHASSIS | ROBOT_CONTROL_GIMBAL)); // 新来源恢复对应功能。
    now_us = 27000u;
    TEST_CHECK(RobotControlFeedbackPoll(NULL)); // 云台姿态消失不影响仍有效的底盘反馈。
    TEST_CHECK(RobotSafetyGetFunctionPermitsAt(27u, &permits) && permits.functions == ROBOT_CONTROL_CHASSIS);
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
    DJIMotorInstance *motors[2] = {0}; // 保留电机实例，模拟应用后续刷新参考。
    for (uint8_t i = 0u; i < 2u; ++i)
    {
        Motor_Init_Config_s config = motor_config((uint8_t)(i + 1u));
        config.output_role = i == 0u ? MOTOR_OUTPUT_CHASSIS : MOTOR_OUTPUT_GIMBAL;
        DJIMotorInstance *motor = DJIMotorInit(&config);
        motors[i] = motor;
        TEST_CHECK(motor != NULL);
        receive(motor, 2000u, 2u, 0, 0); // 两台电机都收到设备反馈。
        RobotSafetySnapshot_t safety = {0};
        TEST_CHECK(RobotSafetyGetSnapshot(&safety));
        TEST_CHECK(DJIMotorSetRefForGeneration(motor, 10.0f, safety.control_generation));
        DJIMotorEnable(motor);
    }
    control_at(3000u); control_at(4000u); // 计算并排队两个非零目标。
    TEST_CHECK((sent[0] || sent[1]) && (sent[2] || sent[3]));
    now_us = 6000u; // 底盘期限已过，排队帧还没有被硬件接受。
    deferred_prepare(deferred_source, HAL_GetTick(), deferred_payload, sizeof(deferred_payload));
    for (unsigned i = 0u; i < 8u; ++i) TEST_CHECK(deferred_payload[i] == 0u); // 发现过期时旧代际整帧撤销。
    RobotSafetySnapshot_t renewed = {0}; // 下一轮云台应用先提交新代际参考。
    TEST_CHECK(RobotSafetyGetSnapshot(&renewed));
    now_us = 7000u;
    TEST_CHECK(DJIMotorSetRefForGeneration(motors[1], 10.0f, renewed.control_generation));
    control_at(7000u); // 下一帧可以保留其他仍有效的机构。
    TEST_CHECK(sent[0] == 0u && sent[1] == 0u && (sent[2] || sent[3]));
    now_us = 8000u; input.feedback[0] = (RobotControlLease_t){8u, 3u, 1u};
    input.manual.source_time_ms = 7u; // 故障恢复前的命令不得接管。
    TEST_CHECK(RobotSafetyPublishControlInputs(&input)); RobotSafetyUpdate();
    control_at(9000u); TEST_CHECK(sent[0] == 0u && sent[1] == 0u);
    now_us = 10000u; input.manual.source_time_ms = 10u; // 恢复后新的人工命令才可以接管。
    TEST_CHECK(RobotSafetyPublishControlInputs(&input)); RobotSafetyUpdate();
    TEST_CHECK(RobotSafetyGetSnapshot(&renewed)); // 以新的来源和代际刷新底盘参考。
    TEST_CHECK(DJIMotorSetRefForGeneration(motors[0], 10.0f, renewed.control_generation));
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
    DJIMotorInstance *motors[4] = {0}; // 继续使用相同实例检查共享帧。
    for (uint8_t i = 0u; i < 4u; ++i)
    {
        Motor_Init_Config_s config = motor_config((uint8_t)(i + 1u));
        config.output_role = (include_unassigned != 0u && i == 3u) ? MOTOR_OUTPUT_UNASSIGNED : roles[i]; // 可注入未声明的电机。
        DJIMotorInstance *motor = DJIMotorInit(&config);
        motors[i] = motor;
        TEST_CHECK(motor != NULL);
        receive(motor, 2000u, 2u, 0, 0);
        if (config.output_role == MOTOR_OUTPUT_LOADER || config.output_role == MOTOR_OUTPUT_CHASSIS ||
            config.output_role == MOTOR_OUTPUT_GIMBAL)
        {
            RobotSafetySnapshot_t safety = {0}; // 拨弹参考绑定本轮安全代际。
            TEST_CHECK(RobotSafetyGetSnapshot(&safety));
            TEST_CHECK(DJIMotorSetRefForGeneration(motor, 10.0f, safety.control_generation));
        }
        else DJIMotorSetRef(motor, 10.0f);
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
    RobotSafetySnapshot_t current = {0}; // 未受影响的应用在下一轮刷新自己的运动参考。
    TEST_CHECK(RobotSafetyGetSnapshot(&current));
    TEST_CHECK(DJIMotorSetRefForGeneration(motors[0], 10.0f, current.control_generation));
    TEST_CHECK(DJIMotorSetRefForGeneration(motors[1], 10.0f, current.control_generation));
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

static int single_turn_angle(void)
{
    Motor_Init_Config_s config = motor_config(1u);
    DJIMotorInstance *motor = DJIMotorInit(&config);
    TEST_CHECK(motor != NULL);
    float angle_deg = -1.0f;
    uint32_t source_ms = 42u;
    TEST_CHECK(!DJIMotorGetSingleTurnAngle(motor, &angle_deg, &source_ms)); // 启动后没有帧时不交付。
    TEST_CHECK(angle_deg == -1.0f && source_ms == 42u);
    now_us = 1000u;
    receive_with_encoder(motor, 1000u, 1u, 0, 0, 2048u); // 四分之一圈，原始接收时刻为 1 ms。
    TEST_CHECK(DJIMotorGetSingleTurnAngle(motor, &angle_deg, &source_ms));
    TEST_CHECK(fabsf(angle_deg - 90.0f) < 0.1f && source_ms == 1u);
    motor->output_position_trusted = 0u; // 多圈位置失去参考时仍可读取单圈角度。
    now_us = (1u + DJI_FEEDBACK_MAX_AGE_MS) * 1000u;
    TEST_CHECK(DJIMotorGetSingleTurnAngle(motor, &angle_deg, &source_ms));
    now_us += 1000u;
    TEST_CHECK(!DJIMotorGetSingleTurnAngle(motor, &angle_deg, &source_ms)); // 期限过去一毫秒后立即拒绝。
    TEST_CHECK(fabsf(angle_deg - 90.0f) < 0.1f && source_ms == 1u);
    return EXIT_SUCCESS;
}

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
    control_at(52000u); // 控制时刻不早于刚接收的 52 ms 数据。
    control_at(52500u); // 在滤波器重建之前仍输出零值。
    TEST_CHECK(sent[0] == 0u && sent[1] == 0u);

    receive(motor, 52u * 480000u + 1000u, 53u, 30, 300);
    DJIMotorSummary_t rebuilt = summary(motor);
    TEST_CHECK(rebuilt.speed_filter_valid && rebuilt.current_filter_valid);
    TEST_CHECK(rebuilt.speed_filter_status == DJI_FILTER_STATUS_REBUILT);
    TEST_CHECK(rebuilt.current_filter_status == DJI_FILTER_STATUS_REBUILT);
    TEST_CHECK(rebuilt.speed_aps == 180.0f && rebuilt.current_feedback == 300.0f);
    control_at(53000u); // 新反馈到达后在同一时间基准上计算。
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
    if (strcmp(argv[1], "single-motion-reference-generation") == 0)
        return single_motion_reference_generation();
    if (strcmp(argv[1], "recovery-real-feedback") == 0)
        return recovery_real_feedback();
    if (strcmp(argv[1], "recovery-coordinator-real-motors") == 0)
        return recovery_coordinator_real_motors();
    if (strcmp(argv[1], "control-feedback-safety") == 0)
        return single_control_feedback_to_safety();
    if (strcmp(argv[1], "single-loader-fire-permit") == 0)
        return single_loader_fire_permit();
    if (strcmp(argv[1], "single-queued-feedback-expiry") == 0)
        return single_queued_feedback_expiry();
    if (strcmp(argv[1], "single-chassis-reference-expiry") == 0)
        return single_chassis_reference_expiry();
    if (strcmp(argv[1], "single-chassis-output-limit") == 0)
        return single_chassis_output_limit();
    if (strcmp(argv[1], "single-chassis-queued-stop") == 0)
        return single_chassis_queued_stop();
    if (strcmp(argv[1], "single-queued-feedback-gap") == 0)
        return single_queued_feedback_gap();
    if (strcmp(argv[1], "single-queued-external-expiry") == 0)
        return single_queued_external_expiry();
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
    if (strcmp(argv[1], "single-turn-angle") == 0) return single_turn_angle();
    if (strcmp(argv[1], "control-expired") == 0) return control_rejects_expired_feedback();
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
