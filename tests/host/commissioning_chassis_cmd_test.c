/** @file commissioning_chassis_cmd_test.c
 * @brief 联测受限底盘命令、人工手势、安全许可及消息有效期。
 */
#include "commissioning_chassis_cmd.h"
#include "daemon.h"
#include "message_center.h"
#include "platform_irq.h"
#include "platform_sync.h"
#include "robot.h"
#include "robot_can_recovery_coordinator.h"
#include "robot_control_feedback_runtime.h"
#include "robot_def.h"
#include "robot_safety.h"
#include "sentinel_config.h"
#include "std_cmd_remote.h"
#include "test_check.h"
#include "topics/robot_topics.h"
#include <float.h>
#include <math.h>
#include <string.h>

static uint32_t tick;
static uint32_t time_offset; // 复用同一场景验证毫秒回绕。
static RemoteControlSnapshot_s radio;
static uint8_t wheel_feedback_available = 1u;
static uint8_t reject_remote_final_check;
static Robot_Config_s vehicle; // 主机提供隔离的只读车型候选，预检仍走真实入口。

const Robot_Config_s *SentinelConfigInit(void) { return &vehicle; }

uint32_t HAL_GetTick(void) { return tick; }

static uint8_t IrqRestored(void)
{
    const PlatformIrqState_t previous = PlatformIrqSaveAndDisable(); // 通过平台替身读取之前的屏蔽状态。
    PlatformIrqRestore(previous);
    return previous == 0u;
}

void BSP_ResetRecoverySetManualEstop(uint8_t latched) { (void)latched; }

void RemoteControlGetSnapshot(RemoteControlSnapshot_s *snapshot) { *snapshot = radio; }

uint8_t RemoteControlRecoveryStillValid(const RemoteControlSnapshot_s *snapshot, uint32_t now_ms)
{
    return !reject_remote_final_check && snapshot->radio_sequence == radio.radio_sequence &&
        (uint32_t)(now_ms - snapshot->radio_source_time_ms) <= REMOTE_RADIO_LEASE_MS; // 最终读取失败由独立场景注入。
}

DaemonStatus_t DaemonProcessTaskEvents(uint8_t consumer, uint8_t budget, uint8_t *processed)
{
    (void)consumer;
    (void)budget;
    (void)processed;
    return DAEMON_OK; // 该联测直接提供遥控和反馈变化。
}

uint8_t RobotControlFeedbackPoll(const MessageFreshness_t *pose)
{
    TEST_CHECK(pose == NULL); // 纯底盘无云台话题。
    const RobotControlLease_t feedback[3] = {
        {tick, 80u, 1u}, // 让长间隔场景与轮组反馈过期相互独立。
        {0}, // 本构建没有云台反馈。
        {0}, // 本构建没有发射反馈。
    };
    return RobotSafetyPublishVehicleFeedback(wheel_feedback_available ? ROBOT_CONTROL_CHASSIS : 0u, feedback);
}

uint8_t RobotCanRecoveryPoll(const RemoteControlSnapshot_s *remote, uint8_t keyboard_allowed,
                             const MessageFreshness_t *pose)
{
    (void)remote;
    (void)keyboard_allowed;
    TEST_CHECK(pose == NULL); // CAN 恢复本身由协调器的专项回归验证。
    return 0u;
}

static void NewRadio(uint32_t time_ms)
{
    tick = time_ms + time_offset;
    ++radio.radio_sequence;
    radio.radio_source_time_ms = tick;
}

static void NewKeyboard(uint32_t time_ms)
{
    NewRadio(time_ms); // 键盘使用独立序号，同时需要在线无线输入。
    ++radio.pc_sample_sequence;
    radio.pc_source_time_ms = tick;
}

// 先配置一组合法的主机轮组，再单独验证人工参数错误和重复选择。
static void CheckVehicleSelection(const RobotManualInputConfig_s *valid_input)
{
    Chassis_Config_s *chassis = &vehicle.chassis_param;
    chassis->wheel_type = OMNI_WHEEL;
    chassis->motion_limits = (ChassisMotionLimits_s){
        .configured = 1u, // 主机使用已知有效的运动范围。
        .max_wz_deg_s = 180.0f, // 本体最大绕上轴速度，deg/s。
        .max_motor_speed_deg_s = 720.0f, // 测试轮轴速度上限，deg/s。
    };
    chassis->chassis_motor_config.can_init_config.can_bus = PLATFORM_CAN_BUS_1;
    chassis->chassis_motor_config.output_axis.gear_ratio = 19.0f;
    chassis->chassis_motor_config.motor_type = M3508;
    chassis->chassis_motor_config.dji_chassis_output_limit_raw = 60u; // 主机专用电调原始命令上限。
    chassis->chassis_motor_config.controller_setting_init_config.outer_loop_type = SPEED_LOOP; // 受限底盘也交付轮轴速度。
    chassis->chassis_motor_config.controller_setting_init_config.close_loop_type = SPEED_LOOP; // 只使用速度环。
    chassis->chassis_motor_config.controller_param_init_config.speed_PID = (PID_Init_Config_s){
        .DtMin_s = MOTOR_PID_DT_MIN_S, // 主机测试沿用电机计算间隔下限。
        .DtMax_s = MOTOR_PID_DT_MAX_S, // 主机测试沿用电机计算间隔上限。
    };
    const float diagonal = 0.70710678118f; // 测试轮的 45 度滚动方向分量。
    chassis->omni_geometry.wheels[0] = (ChassisWheelGeometry_t){0.2f, 0.2f, diagonal, -diagonal, 0.08f}; // 左前轮位置、方向和半径。
    chassis->omni_geometry.wheels[1] = (ChassisWheelGeometry_t){0.2f, -0.2f, diagonal, diagonal, 0.08f}; // 右前轮。
    chassis->omni_geometry.wheels[2] = (ChassisWheelGeometry_t){-0.2f, 0.2f, diagonal, diagonal, 0.08f}; // 左后轮。
    chassis->omni_geometry.wheels[3] = (ChassisWheelGeometry_t){-0.2f, -0.2f, diagonal, -diagonal, 0.08f}; // 右后轮。
    for (unsigned i = 0u; i < CHASSIS_WHEEL_COUNT; ++i)
        chassis->wheel_motors[i] = (ChassisWheelMotorConfig_t){(uint8_t)(i + 1u), MOTOR_AXIS_DIRECTION_NORMAL}; // 四个编号互异。

    const Robot_Config_s *selected = NULL;
    vehicle.motor_periods.compute_us = 2000u; // 主机受限轮组的计算周期为 2 ms。
    vehicle.manual_input = *valid_input;
    TEST_CHECK(RobotConfigSelect(SENTINEL_ROBOT, &selected) == ROBOT_CONFIG_OK && selected == &vehicle);
    chassis->chassis_motor_config.controller_param_init_config.speed_PID.DtMax_s = 0.001f; // 名义 2 ms 超过速度环允许的 1 ms。
    TEST_CHECK(RobotConfigSelect(SENTINEL_ROBOT, &selected) == ROBOT_CONFIG_BAD_PARAMETERS && selected == NULL);
    chassis->chassis_motor_config.controller_param_init_config.speed_PID.DtMax_s = 0.002f; // 恢复至允许边界，后续继续核对人工输入。
    chassis->chassis_motor_config.controller_setting_init_config.feedforward_flag = CURRENT_FEEDFORWARD; // 受限底盘也必须拒绝缺少回调的启用来源。
    TEST_CHECK(RobotConfigSelect(SENTINEL_ROBOT, &selected) == ROBOT_CONFIG_BAD_PARAMETERS && selected == NULL);
    chassis->chassis_motor_config.controller_setting_init_config.feedforward_flag = FEEDFORWARD_NONE; // 测试轮组不使用前馈。
    chassis->chassis_motor_config.controller_setting_init_config.outer_loop_type = ANGLE_LOOP; // 受限调试也不能用速度命令驱动位置环。
    TEST_CHECK(RobotConfigSelect(SENTINEL_ROBOT, &selected) == ROBOT_CONFIG_BAD_PARAMETERS && selected == NULL);
    chassis->chassis_motor_config.controller_setting_init_config.outer_loop_type = SPEED_LOOP; // 恢复后继续独立核对人工输入。
    vehicle.manual_input.map = NULL; // 未提供人工映射时清除上一次成功选择。
    TEST_CHECK(RobotConfigSelect(SENTINEL_ROBOT, &selected) == ROBOT_CONFIG_BAD_MANUAL_INPUT);
    TEST_CHECK(selected == NULL && RobotConfigFeed() == NULL);
    vehicle.manual_input = *valid_input;
    vehicle.manual_input.max_dt_ms = 0u; // 缺少最大允许间隔，不能启动轮组。
    TEST_CHECK(RobotConfigSelect(SENTINEL_ROBOT, &selected) == ROBOT_CONFIG_BAD_MANUAL_INPUT);
    vehicle.manual_input.max_dt_ms = 2000u; // 用 2 秒重现有限速率换算后溢出的情况。
    vehicle.manual_input.commissioning_linear_accel_m_s2 = FLT_MAX;
    TEST_CHECK(RobotConfigSelect(SENTINEL_ROBOT, &selected) == ROBOT_CONFIG_BAD_MANUAL_INPUT);
    TEST_CHECK(selected == NULL && RobotConfigFeed() == NULL);
    vehicle.manual_input = *valid_input; // 只允许重新选中已修正的配置。
    TEST_CHECK(RobotConfigSelect(SENTINEL_ROBOT, &selected) == ROBOT_CONFIG_OK && selected == &vehicle);

    StdCmdRemoteMap_t map = *valid_input->map; // 用隔离副本逐项检查活动输入的非零边界。
    vehicle.manual_input.map = &map;
    map.remote_map.axis[STD_CMD_VX].gain = FLT_MAX; // 零输入通过，但拨满时先于限幅发生溢出。
    TEST_CHECK(StdCmdRemoteMapIsValid(&map));
    TEST_CHECK(RobotConfigSelect(SENTINEL_ROBOT, &selected) == ROBOT_CONFIG_BAD_MANUAL_INPUT);
    TEST_CHECK(selected == NULL && RobotConfigFeed() == NULL);

    map = *valid_input->map;
    map.remote_map.axis[STD_CMD_VX].gain = FLT_MAX / 1024.0f; // 原始值到 m/s 有限，换到 mm/s 会溢出。
    map.remote_map.axis[STD_CMD_VX].output_limit = FLT_MAX;
    TEST_CHECK(RobotConfigSelect(SENTINEL_ROBOT, &selected) == ROBOT_CONFIG_BAD_MANUAL_INPUT);

    map = *valid_input->map;
    map.keyboard_map.axis[STD_CMD_VX].input_limit = 0.5f; // 实际按住方向键时产生 1 或 -1。
    TEST_CHECK(RobotConfigSelect(SENTINEL_ROBOT, &selected) == ROBOT_CONFIG_BAD_MANUAL_INPUT);

    map = *valid_input->map;
    map.keyboard_positive[STD_CMD_WZ] = (uint16_t)(1u << Key_D); // 只在此用例启用键盘转动通道。
    map.keyboard_map.axis[STD_CMD_WZ].gain = -FLT_MAX;
    map.keyboard_map.axis[STD_CMD_WZ].output_limit = FLT_MAX;
    TEST_CHECK(RobotConfigSelect(SENTINEL_ROBOT, &selected) == ROBOT_CONFIG_BAD_MANUAL_INPUT); // 有限 rad/s 换到 deg/s 时溢出。

    map = *valid_input->map;
    map.remote_map.axis[STD_CMD_VX].input_limit = FLT_MAX; // 协议不会交付超出拨杆物理上限的原始值。
    map.remote_map.axis[STD_CMD_WZ].output_limit = FLT_MAX; // 实际可到达的速度仍受正常比例限制。
    TEST_CHECK(RobotConfigSelect(SENTINEL_ROBOT, &selected) == ROBOT_CONFIG_OK && selected == &vehicle);
    vehicle.manual_input = *valid_input; // 离开函数前恢复有效的长期车型映射。
}

// 用独立副本检查名义周期边界，保留后续运行场景原有的时限。
static void CheckPublisherTiming(const RobotManualInputConfig_s *valid_input)
{
    StdCmdRemoteMap_t map = *valid_input->map; // 测试可分别缩短摇杆和键鼠期限。
    RobotManualInputConfig_s input = *valid_input; // 使用已经通过车型检查的数值。
    input.map = &map;
    input.max_dt_ms = 5u;
    TEST_CHECK(RobotCommissioningChassisInputFitsPeriod(&input, 5000u)); // 等于运行期允许的时差仍可启动。
    TEST_CHECK(!RobotCommissioningChassisInputFitsPeriod(&input, 5001u)); // 不能把不足的 1 μs 向下舍去。
    TEST_CHECK(!RobotCommissioningChassisInputFitsPeriod(&input, 0u)); // 没有拍点时不授予启动条件。
    input.max_dt_ms = valid_input->max_dt_ms;
    map.remote_map.max_age_ms = 4u; // 无线映射可以单独低于 5 ms 发布周期。
    TEST_CHECK(!RobotCommissioningChassisInputFitsPeriod(&input, 5000u));
    map.remote_map.max_age_ms = 5u;
    map.keyboard_map.max_age_ms = 4u; // 键鼠配置也按同一发布周期复核。
    TEST_CHECK(!RobotCommissioningChassisInputFitsPeriod(&input, 5000u));
    map.keyboard_map.max_age_ms = 5u;
    TEST_CHECK(RobotCommissioningChassisInputFitsPeriod(&input, 5000u));

    const uint32_t radio_period_us = REMOTE_RADIO_LEASE_MS * 1000u; // 把两种车型期限放宽，仅验证协议上限。
    map.remote_map.max_age_ms = map.keyboard_map.max_age_ms = input.max_dt_ms = UINT32_MAX / 2u;
    TEST_CHECK(RobotCommissioningChassisInputFitsPeriod(&input, radio_period_us));
    TEST_CHECK(!RobotCommissioningChassisInputFitsPeriod(&input, radio_period_us + 1u)); // 配置不能延长无线期限。
}

// 使用主机时间参数验证各段都合法但合计太长的情况。
static void CheckChassisPipelineTiming(const RobotManualInputConfig_s *valid_input)
{
    StdCmdRemoteMap_t map = *valid_input->map; // 独立调整摇杆和键鼠期限。
    RobotManualInputConfig_s input = *valid_input;
    input.map = &map;
    PlatformPeriodicConfig_t command = {.period_us = 5000u, .budget_us = 5000u}; // 发布任务每 5 ms 执行，预算同周期。
    PlatformPeriodicConfig_t chassis = {.period_us = 1000u, .budget_us = 1000u}; // 测试用底盘拍点和预算各 1 ms。
    const MotorTaskPeriods_t motor = {.compute_us = 1000u, .transmit_us = 1000u}; // 计算和发送名义等待各 1 ms。
    const PlatformPeriodicConfig_t dispatch = {.period_us = 1000u, .budget_us = 1000u}; // 发送轮次另留 1 ms 处理。
    map.remote_map.max_age_ms = 15u; // 五个阶段的名义合计恰好 15 ms。
    map.keyboard_map.max_age_ms = 15u;
    input.max_dt_ms = 10u; // 上一拍一开始就发布，下一拍用完预算才发布，间隔恰好 10 ms。
    TEST_CHECK(RobotCommissioningChassisTimingFits(&input, &command, &chassis, &motor, &dispatch));
    input.max_dt_ms = 9u; // 来源与话题有时间送达，但变化率会因两次发布间隔超限而撤销旧目标。
    TEST_CHECK(RobotCommissioningChassisInputFitsPeriod(&input, command.period_us));
    TEST_CHECK(!RobotCommissioningChassisTimingFits(&input, &command, &chassis, &motor, &dispatch));
    input.max_dt_ms = 10u; // 恢复合法的发布间隔，以下继续检查端到端路径。
    ++chassis.period_us;
    TEST_CHECK(!RobotCommissioningChassisTimingFits(&input, &command, &chassis, &motor, &dispatch)); // 多 1 μs 也不能向下舍入。
    --chassis.period_us;
    map.keyboard_map.max_age_ms = 14u; // 旧单任务检查可以通过，但键鼠不能满足整条路径。
    TEST_CHECK(RobotCommissioningChassisInputFitsPeriod(&input, command.period_us));
    TEST_CHECK(!RobotCommissioningChassisTimingFits(&input, &command, &chassis, &motor, &dispatch));
    map.keyboard_map.max_age_ms = map.remote_map.max_age_ms = 150u; // 放宽车型期限，分别验证话题和协议上限。
    chassis.period_us = 46000u;
    TEST_CHECK(RobotCommissioningChassisTimingFits(&input, &command, &chassis, &motor, &dispatch)); // 发布后恰好 50 ms。
    ++chassis.period_us;
    TEST_CHECK(!RobotCommissioningChassisTimingFits(&input, &command, &chassis, &motor, &dispatch));
    --chassis.period_us;
    command = (PlatformPeriodicConfig_t){.period_us = 25000u, .budget_us = 25000u}; // 输入可读取后合计恰好达到无线期限。
    input.max_dt_ms = 80u; // 放宽变化率的间隔，允许单独检查话题与协议。
    TEST_CHECK(RobotCommissioningChassisTimingFits(&input, &command, &chassis, &motor, &dispatch));
    ++command.period_us;
    TEST_CHECK(!RobotCommissioningChassisTimingFits(&input, &command, &chassis, &motor, &dispatch));
    chassis.period_us = 1000u; // 下游缩短，整条路径仍在来源期限内。
    TEST_CHECK(!RobotCommissioningChassisTimingFits(&input, &command, &chassis, &motor, &dispatch)); // 发布间隔 50001 μs 会让话题过期。
    --command.period_us;
    TEST_CHECK(RobotCommissioningChassisTimingFits(&input, &command, &chassis, &motor, &dispatch)); // 恰好等于话题期限时保留旧目标。
    chassis.period_us = UINT32_MAX; // 用宽整数保留真实大小，不能绕回合法时限。
    TEST_CHECK(!RobotCommissioningChassisTimingFits(&input, &command, &chassis, &motor, &dispatch));
    chassis = (PlatformPeriodicConfig_t){.period_us = 1000u}; // 缺少执行预算，不能判断组合时限。
    TEST_CHECK(!RobotCommissioningChassisTimingFits(&input, &command, &chassis, &motor, &dispatch));
}

int main(int argc, char **argv)
{
    time_offset = argc == 2 && strcmp(argv[1], "wrap") == 0 ? UINT32_MAX - 1015u : 0u; // 让同一速度段跨过零。
    tick = 1u + time_offset;
    RobotSafetyInit();
    TEST_CHECK(RobotSafetyBindVehicleFeedback());
    const RobotControlPolicy_t policy = {
        .enabled_functions = ROBOT_CONTROL_CHASSIS, // 测试调用与启动配置使用同一功能。
        .configured = 1u, // 测试提供完整的人工输入。
        .formal_autonomous = 0u, // 无自主来源。
        .commissioning = 1u, // 只接受人工。
    };
    TEST_CHECK(RobotSafetyConfigureControl(&policy));
    RobotSafetySetImuValid(1u);
    RobotSafetySetCalibValid(1u);
    RobotSafetySetTaskHealthy(1u);

    TEST_CHECK(RobotTopicsInit());
    uint8_t topic_count = 0u;
    TEST_CHECK(MessageGetDiagnosticTopicCount(&topic_count) == MESSAGE_OK && topic_count == 2u);
    TEST_CHECK(MessageTopicGetDescriptor(ROBOT_TOPIC_GIMBAL_COMMAND, &(MessageTopicDescriptor_t){0}) != MESSAGE_OK);
    MessageSubscriber_t *subscriber = MessageSubscriberRegister(ROBOT_TOPIC_CHASSIS_COMMAND, ROBOT_TOPIC_ENDPOINT_CHASSIS);
    TEST_CHECK(subscriber != NULL);

    StdCmdRemoteMap_t map = {0};
    map.remote_map.max_age_ms = 15u; // 主机测试的无线输入短期限。
    map.keyboard_map.max_age_ms = 15u; // 键盘来源也有自己的短期限。
    map.remote_map.allowed_frames = 1u << STD_CMD_FRAME_CHASSIS; // 测试只接受本体坐标。
    map.keyboard_map.allowed_frames = 1u << STD_CMD_FRAME_CHASSIS; // 键盘与遥控采用同一参照。
    map.remote_frame = STD_CMD_FRAME_BIG_YAW; // 先验证缺少云台参照时拒绝启动。
    map.keyboard_frame = STD_CMD_FRAME_CHASSIS;
    for (unsigned axis = 0u; axis < STD_CMD_AXIS_COUNT; ++axis)
    {
        map.remote_map.axis[axis] = (StdCmdAxisMap_t){660.0f, 0.001f, 1.0f}; // 摇杆方向和范围仅供测试。
        map.keyboard_map.axis[axis] = (StdCmdAxisMap_t){1.0f, 0.1f, 1.0f}; // 测试键盘比例与范围。
    }
    map.remote_axis[STD_CMD_VX] = STD_CMD_RC_RIGHT_Y; // 测试右摇杆控制前向。
    map.remote_axis[STD_CMD_VY] = STD_CMD_RC_RIGHT_X; // 同时给出左移以检查平移总步长。
    map.remote_axis[STD_CMD_WZ] = STD_CMD_RC_LEFT_X; // 给出独立的绕上轴请求。
    map.keyboard_positive[STD_CMD_VX] = (uint16_t)(1u << Key_W); // 测试 W 键控制前向。
    RobotManualInputConfig_s input = {
        .map = &map, // 只使用本场景的映射。
        .max_dt_ms = 20u, // 配置超过 20 ms 时撤销。
        .commissioning_linear_accel_m_s2 = 2.0f, // 主机用平移步长。
        .commissioning_angular_accel_rad_s2 = 3.0f, // 主机用绕上轴步长。
    };
    TEST_CHECK(!CommissioningChassisCmdInit(&input));
    map.remote_frame = STD_CMD_FRAME_CHASSIS;
    input.commissioning_linear_accel_m_s2 = 0.0f; // 缺少平移上限时不能登记。
    TEST_CHECK(!CommissioningChassisCmdInit(&input));
    input.commissioning_linear_accel_m_s2 = 2.0f;
    input.commissioning_angular_accel_rad_s2 = NAN; // 拒绝非有限的转动上限。
    TEST_CHECK(!CommissioningChassisCmdInit(&input));
    input.commissioning_angular_accel_rad_s2 = 3.0f;
    CheckVehicleSelection(&input);
    CheckPublisherTiming(&input);
    CheckChassisPipelineTiming(&input);
    TEST_CHECK(CommissioningChassisCmdInit(&input));

    radio.remote_online = 1u;
    radio.input.rc.switch_left = RC_SW_DOWN;
    radio.input.rc.switch_right = RC_SW_MID;
    radio.input.rc.rocker_r1 = 100;
    radio.input.rc.rocker_r_ = 100; // 与前向请求相同，形成斜向移动。
    radio.input.rc.rocker_l_ = 100; // 独立请求正向转动。
    NewRadio(2u);
    CommissioningChassisCmdTask(); // 新鲜输入本身不得使能。
    Chassis_Ctrl_Cmd_s command;
    MessageFreshness_t freshness;
    TEST_CHECK(MessageReadSnapshot(subscriber, &command, sizeof(command), tick, NULL) != MESSAGE_OK);
    radio.input.rc.switch_right = RC_SW_DOWN; // 用生产手势进入人工使能。
    NewRadio(10u);
    CommissioningChassisCmdTask();
    NewRadio(1010u);
    CommissioningChassisCmdTask();
    TEST_CHECK(MessageReadSnapshot(subscriber, &command, sizeof(command), tick, NULL) != MESSAGE_OK); // 使能同刻的输入仍旧。
    NewRadio(1011u);
    CommissioningChassisCmdTask();
    TEST_CHECK(MessageReadSnapshot(subscriber, &command, sizeof(command), tick, &freshness) == MESSAGE_OK);
    TEST_CHECK(command.chassis_mode == CHASSIS_NO_FOLLOW && command.vx == 0.0f &&
        command.vy == 0.0f && command.wz == 0.0f); // 首份获准目标只建立零基准。
    TEST_CHECK(freshness.source_time_ms == 1011u + time_offset && freshness.source_max_age_ms == 15u);

    tick = 1020u + time_offset;
    CommissioningChassisCmdTask(); // 重发也保留原始来源期限。
    TEST_CHECK(MessageReadSnapshot(subscriber, &command, sizeof(command), tick, &freshness) == MESSAGE_OK &&
        freshness.source_time_ms == 1011u + time_offset && freshness.source_max_age_ms == 15u);
    TEST_CHECK(fabsf(hypotf(command.vx, command.vy) - 18.0f) < 0.01f && command.vx > 0.0f &&
        fabsf(command.vx + command.vy) < 0.01f); // 9 ms 的总步长为 18 mm/s，左移在旧消息里为负。
    TEST_CHECK(fabsf(command.wz - 1.546986f) < 0.001f); // 9 ms 的独立角速度步长换成 deg/s。
    const float unchanged_vx = command.vx;
    CommissioningChassisCmdTask(); // 同一毫秒重复运行没有额外变化预算。
    TEST_CHECK(MessageReadSnapshot(subscriber, &command, sizeof(command), tick, NULL) == MESSAGE_OK &&
        command.vx == unchanged_vx);

    radio.input.rc.rocker_r1 = -100;
    radio.input.rc.rocker_r_ = -100;
    radio.input.rc.rocker_l_ = -100;
    NewRadio(1022u);
    CommissioningChassisCmdTask(); // 反向请求先减速，不能一步跳成反方向。
    TEST_CHECK(MessageReadSnapshot(subscriber, &command, sizeof(command), tick, NULL) == MESSAGE_OK);
    TEST_CHECK(fabsf(hypotf(command.vx, command.vy) - 14.0f) < 0.01f && command.vx > 0.0f &&
        fabsf(command.wz - 1.203211f) < 0.001f); // 2 ms 只允许分别减少 4 mm/s 和 0.006 rad/s。

    NewRadio(1038u); // 新输入到达时旧目标已超过 15 ms，期间任务没有观察到失效。
    CommissioningChassisCmdTask();
    TEST_CHECK(MessageReadSnapshot(subscriber, &command, sizeof(command), tick, NULL) != MESSAGE_OK); // 新输入不能继承过期的速度基准。

    NewRadio(1039u);
    CommissioningChassisCmdTask(); // 新无线数据允许恢复。
    TEST_CHECK(MessageReadSnapshot(subscriber, &command, sizeof(command), tick, NULL) == MESSAGE_OK &&
        command.vx == 0.0f && command.wz == 0.0f);
    ++radio.source_selection_generation;
    radio.input.rc.switch_left = RC_SW_UP;
    radio.pc_valid = 1u;
    radio.input.key[KEY_PRESS].keys = (uint16_t)(1u << Key_W);
    NewKeyboard(1040u);
    CommissioningChassisCmdTask(); // 切档这一轮先撤销，不沿用旧目标。
    TEST_CHECK(MessageReadSnapshot(subscriber, &command, sizeof(command), tick, NULL) != MESSAGE_OK);
    NewKeyboard(1041u);
    CommissioningChassisCmdTask();
    TEST_CHECK(MessageReadSnapshot(subscriber, &command, sizeof(command), tick, NULL) == MESSAGE_OK);
    const uint32_t old_generation = command.safety_generation; // 发送方不能重用检查失败前的代际。
    reject_remote_final_check = 1u;
    NewKeyboard(1042u);
    CommissioningChassisCmdTask();
    TEST_CHECK(MessageReadSnapshot(subscriber, &command, sizeof(command), tick, NULL) != MESSAGE_OK);
    TEST_CHECK(!RobotSafetyIsReadyForGeneration(tick, old_generation) && IrqRestored());
    reject_remote_final_check = 0u;

    NewKeyboard(1043u);
    CommissioningChassisCmdTask();
    TEST_CHECK(MessageReadSnapshot(subscriber, &command, sizeof(command), tick, NULL) == MESSAGE_OK && command.vx == 0.0f);
    NewKeyboard(1048u);
    CommissioningChassisCmdTask();
    TEST_CHECK(MessageReadSnapshot(subscriber, &command, sizeof(command), tick, NULL) == MESSAGE_OK &&
        fabsf(command.vx - 10.0f) < 0.01f);
    NewKeyboard(1070u); // 任务间隔超过配置，仍有新鲜键盘和轮组反馈。
    CommissioningChassisCmdTask();
    TEST_CHECK(MessageReadSnapshot(subscriber, &command, sizeof(command), tick, NULL) != MESSAGE_OK);
    NewKeyboard(1071u);
    CommissioningChassisCmdTask();
    TEST_CHECK(MessageReadSnapshot(subscriber, &command, sizeof(command), tick, NULL) == MESSAGE_OK && command.vx == 0.0f);
    NewKeyboard(1073u);
    CommissioningChassisCmdTask();
    TEST_CHECK(MessageReadSnapshot(subscriber, &command, sizeof(command), tick, NULL) == MESSAGE_OK && command.vx > 0.0f);
    const uint32_t before_stop = command.safety_generation; // 保存故障发生前已交给消费者的旧代次。
    RobotStopCommissioningCommand(); // 模拟命令任务收到时钟故障，此时原目标仍在有效期内。
    TEST_CHECK(MessageReadSnapshot(subscriber, &command, sizeof(command), tick, NULL) != MESSAGE_OK);
    TEST_CHECK(!RobotSafetyIsReadyForGeneration(tick, before_stop) && IrqRestored());

    NewKeyboard(1074u);
    CommissioningChassisCmdTask(); // 新输入不能替代重新使能。
    TEST_CHECK(MessageReadSnapshot(subscriber, &command, sizeof(command), tick, NULL) != MESSAGE_OK);
    RobotSafetyRequestEnable(); // 直接模拟一次新的人工使能，验证接管的时间边界。
    CommissioningChassisCmdTask();
    TEST_CHECK(MessageReadSnapshot(subscriber, &command, sizeof(command), tick, NULL) != MESSAGE_OK);
    NewKeyboard(1075u);
    CommissioningChassisCmdTask();
    TEST_CHECK(MessageReadSnapshot(subscriber, &command, sizeof(command), tick, NULL) == MESSAGE_OK && command.vx == 0.0f);
    NewKeyboard(1077u);
    CommissioningChassisCmdTask();
    TEST_CHECK(MessageReadSnapshot(subscriber, &command, sizeof(command), tick, NULL) == MESSAGE_OK && command.vx > 0.0f);
    wheel_feedback_available = 0u;
    NewKeyboard(1078u);
    CommissioningChassisCmdTask(); // 仍有人工输入，缺少轮组反馈也必须禁止。
    TEST_CHECK(MessageReadSnapshot(subscriber, &command, sizeof(command), tick, NULL) != MESSAGE_OK);
    radio.remote_online = 0u;
    NewRadio(1079u);
    CommissioningChassisCmdTask(); // 无线离线锁存急停。
    TEST_CHECK(EstopIsLatched() && EstopGetReason() == ESTOP_REASON_RC_OFFLINE && IrqRestored());
    return 0;
}
