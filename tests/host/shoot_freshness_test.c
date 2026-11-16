#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "shoot.h"
#include "robot_def.h"
#include "config.h"
#include "message_center.h"
#include "topics/robot_topics.h"
#include "robot_safety.h" // 发射任务通过真实安全管理验证代际和许可。

#define CHECK(condition) do { if (!(condition)) { \
    fprintf(stderr, "FAIL:%d %s\n", __LINE__, #condition); return 1; \
} } while (0)

CAN_HandleTypeDef hcan2;
DJIMotorInstance shoot_fake_motors[5];
uint32_t shoot_fake_motor_init_count;
static uint32_t fake_init_attempt; // 记录电机申请次数，供失败注入使用。
static uint32_t fake_fail_attempt; // 非零时在指定申请次数返回失败。
static uint32_t fake_rollback_count; // 记录已撤销的电机数量。
static uint8_t fake_rollback_failure; // 注入回退无法完成的情况。
static uint8_t fake_rolled_back[5]; // 记录各实例是否已撤销。
#if defined(MC02_TEST_WRAP_SUBSCRIBER)
static uint8_t fake_subscriber_failure; // 注入最后一步话题申请失败。
MessageSubscriber_t *__real_MessageSubscriberRegister(MessageTopicId_t topic_id, MessageEndpointId_t endpoint_id);
MessageSubscriber_t *__wrap_MessageSubscriberRegister(MessageTopicId_t topic_id, MessageEndpointId_t endpoint_id)
{
    if (fake_subscriber_failure) return NULL; // 测试中拒绝发射话题登记。
    return __real_MessageSubscriberRegister(topic_id, endpoint_id); // 其他场景调用生产实现。
}
#endif
static uint32_t now_ms;
static float timeline_ms;
static Shoot_Config_s shoot_config = {
    .shoot_type = DUAL_SHOOT,
    .friction_motor_config = {
        .can_init_config = {.can_bus = PLATFORM_CAN_BUS_2},
        .controller_setting_init_config = {.motor_reverse_flag = MOTOR_DIRECTION_NORMAL},
        .output_axis = {19.0f, MOTOR_AXIS_DIRECTION_NORMAL},
        .motor_type = M3508,
    },
    .loader_motor_config = {
        .can_init_config = {.can_bus = PLATFORM_CAN_BUS_2, .tx_id = 3u},
        .controller_setting_init_config = {.close_loop_type = SPEED_LOOP | CURRENT_LOOP | ANGLE_LOOP},
        .output_axis = {36.0f, MOTOR_AXIS_DIRECTION_NORMAL},
        .motor_type = M2006,
    },
    .friction_motor_id = {1u, 2u},
    .friction_motor_reverse_flag = {MOTOR_DIRECTION_NORMAL, MOTOR_DIRECTION_REVERSE},
    .loader_slot_num = 10u,
    .loader_gearbox_ratio = 36.0f,
    .loader_disk_ratio = 1.0f,
    .friction_default_speed_rad_s = 30000.0f * 0.01745329252f / 19.0f,
    .loader_reverse_speed_rad_s = 500.0f * 0.01745329252f / 36.0f,
};

uint32_t HAL_GetTick(void) { return now_ms; }
float PlatformProbeTimelineMs(void) { return timeline_ms; }
const Shoot_Config_s *ShootConfigFeed(void) { return &shoot_config; }
DJIMotorInstance *DJIMotorInit(const Motor_Init_Config_s *config)
{
    ++fake_init_attempt; // 每次申请均计数，包括失败的申请。
    if (fake_fail_attempt == fake_init_attempt) return NULL; // 按测试配置注入设备注册失败。
    if (config == NULL || shoot_fake_motor_init_count >= 5u)
        return NULL;
    DJIMotorInstance *motor = &shoot_fake_motors[shoot_fake_motor_init_count];
    memset(motor, 0, sizeof(*motor));
    motor->output_axis = config->output_axis;
    motor->close_loop_type = config->controller_setting_init_config.close_loop_type;
    motor->measure.total_angle = 100.0f + (float)shoot_fake_motor_init_count;
    shoot_fake_motor_init_count++;
    return motor;
}
uint8_t DJIMotorRollbackStartup(DJIMotorInstance *motor)
{
    if (fake_rollback_failure) return 0u; // 模拟无法安全释放资源。
    for (unsigned i = 0u; i < 5u; ++i)
        if (motor == &shoot_fake_motors[i] && !fake_rolled_back[i])
        {
            fake_rolled_back[i] = 1u; // 标记指定实例已释放。
            ++fake_rollback_count; // 验证没有漏回退或重复回退。
            return 1u;
        }
    return 0u; // 非测试实例或重复回退均视为失败。
}
void DJIMotorSetRef(DJIMotorInstance *motor, float ref) { if (motor != NULL) motor->ref = ref; }
uint8_t DJIMotorSetRefForGeneration(DJIMotorInstance *motor, float ref, uint32_t generation)
{
    RobotControlPermits_t permits = {0}; // 测试替身复用真实安全管理。
    uint8_t active = 0u; // 记录是否使用分功能策略。
    if (motor == NULL || !RobotSafetyGetOutputPermitsForGeneration(now_ms, generation, &permits, &active) ||
        !(permits.functions & ROBOT_CONTROL_SHOOTER) || (active && !permits.fire_allowed)) return 0u; // 拒绝错误代际和未许可的拨弹参考。
    motor->ref = ref; // 允许时模拟电机驱动写入参考。
    return 1u; // 测试可继续检查写入结果。
}
void DJIMotorStop(DJIMotorInstance *motor) { if (motor != NULL) motor->enabled = 0u; }
void DJIMotorEnable(DJIMotorInstance *motor) { if (motor != NULL) motor->enabled = 1u; }
void DJIMotorOuterLoop(DJIMotorInstance *motor, Closeloop_Type_e loop)
{ if (motor != NULL) motor->outer_loop = loop; }
void DJIMotorGetSummary(DJIMotorInstance *motor, DJIMotorSummary_t *summary)
{ if (motor != NULL && summary != NULL) summary->enabled = motor->enabled; }
Motor_Feedback_Status_e DJIMotorGetFeedbackSnapshot(const DJIMotorInstance *motor,
                                                     uint32_t timestamp,
                                                     Motor_Feedback_Snapshot_s *snapshot)
{
    (void)timestamp;
    if (motor == NULL || snapshot == NULL)
        return MOTOR_FEEDBACK_BAD_ARGUMENT;
    const float scale = 0.01745329252f * (float)motor->output_axis.direction /
                        motor->output_axis.gear_ratio;
    snapshot->position = (Motor_Signal_Snapshot_s){
        motor->measure.total_angle * scale, MOTOR_UNIT_RAD, MOTOR_REFERENCE_OUTPUT_AXIS,
        now_ms, 1u, 1u};
    return MOTOR_FEEDBACK_OK;
}

static MessageStatus_t publish(MessagePublisher_t *publisher,
                               const Shoot_Ctrl_Cmd_s *command, uint32_t timestamp)
{
    Shoot_Ctrl_Cmd_s stamped = *command; // 测试发布者模仿命令任务绑定安全代际。
    RobotSafetySnapshot_t safety = {0}; // 保存计算命令时读取的安全状态。
    if (!RobotSafetyGetSnapshot(&safety)) return MESSAGE_ERR_RANGE; // 查询失败时不能发布动作。
    stamped.safety_generation = safety.control_generation; // 后续故障不能给旧命令续用新代际。
    stamped.fire_source = ROBOT_CONTROL_SOURCE_MANUAL; // 本测试命令来自人工输入。
    return MessagePublishAt(publisher, &stamped, timestamp, timestamp, timestamp,
                            MESSAGE_CLOCK_LOCAL);
}

int main(int argc, char **argv)
{
    RobotSafetyInit(); // 测试默认使用真实安全管理。
    RobotSafetySetRcOnline(1u); // 提供人工模式的遥控在线条件。
    RobotSafetySetImuValid(1u); // 提供姿态有效条件。
    RobotSafetySetCalibValid(1u); // 提供标定有效条件。
    RobotSafetySetTaskHealthy(1u); // 提供任务健康条件。
    RobotSafetyRequestEnable(); // 测试操作者请求使能。
    RobotSafetyUpdate(); // 发布经检查的整车状态。
    if (argc == 2 && strcmp(argv[1], "safety-fire") == 0)
    {
        RobotSafetyInit(); // 新策略只能在初始 SAFE 阶段配置。
        RobotSafetySetRcOnline(1u); RobotSafetySetImuValid(1u); // 提供遥控和姿态条件。
        RobotSafetySetCalibValid(1u); RobotSafetySetTaskHealthy(1u); // 提供标定和任务条件。
        const RobotControlPolicy_t policy = {ROBOT_CONTROL_ALL, 1u, 0u, 0u}; // 本测试使用人工分功能许可。
        CHECK(RobotSafetyConfigureControl(&policy)); // 启动时配置策略。
        RobotSafetyRequestEnable(); RobotSafetyUpdate(); // 等待后续人工来源。
        now_ms = 1u; // 新来源晚于初次使能。
        RobotSafetyControlInputs_t inputs = {0}; // 本轮测试的来源与反馈。
        inputs.mode = ROBOT_CONTROL_MANUAL; // 人工决定击发。
        inputs.manual = (RobotControlLease_t){now_ms, 100u, 1u}; // 测试人工来源有效。
        for (unsigned i = 0u; i < 3u; ++i) inputs.feedback[i] = (RobotControlLease_t){now_ms, 100u, 1u}; // 测试三组反馈有效。
        inputs.feedback_valid_functions = ROBOT_CONTROL_ALL; // 全部测试机构可用。
        inputs.manual_fire_requested = 1u; // 当前人工请求连续供弹。
        CHECK(RobotSafetyPublishControlInputs(&inputs)); RobotSafetyUpdate(); // 发布初始许可。
        CHECK(ShootInit() == 1u); // 注册生产发射任务和测试电机。
        MessagePublisher_t *publisher = MessagePublisherRegister(ROBOT_TOPIC_SHOOT_COMMAND, ROBOT_TOPIC_ENDPOINT_COMMAND); // 通过实际话题提交。
        CHECK(publisher != NULL); // 话题注册必须成功。
        Shoot_Ctrl_Cmd_s command = {.shoot_mode = SHOOT_ON, .friction_mode = FRICTION_ON,
            .load_mode = LOAD_BURSTFIRE, .shoot_rate = 8.0f, .bullet_speed = (Bullet_Speed_e)99}; // 产生非零拨弹目标。
        CHECK(publish(publisher, &command, now_ms) == MESSAGE_OK); ShootTask(); // 发布并执行命令。
        CHECK(shoot_fake_motors[2].ref > 0.0f); // 初始许可允许供弹。
        ++now_ms; inputs.manual_fire_requested = 0u; // 人工撤销击发，机构仍可运行。
        CHECK(RobotSafetyPublishControlInputs(&inputs)); RobotSafetyUpdate(); ShootTask(); // 发射任务发现许可失效。
        CHECK(shoot_fake_motors[2].ref == 0.0f && !shoot_fake_motors[2].enabled); // 旧代际先撤销全部电机目标。
        command.load_mode = LOAD_STOP; // 命令任务重新发布允许摩擦轮的停止供弹命令。
        CHECK(publish(publisher, &command, now_ms) == MESSAGE_OK); ShootTask(); // 新代际可以保留机构运行。
        CHECK(shoot_fake_motors[0].enabled && shoot_fake_motors[2].ref == 0.0f); // 摩擦轮恢复而拨弹继续停止。
        command.load_mode = LOAD_BURSTFIRE; // 下一步模拟旧连续请求。
        ++now_ms; inputs.manual_fire_requested = 1u; // 恢复许可但不发布新的发射命令。
        CHECK(RobotSafetyPublishControlInputs(&inputs)); RobotSafetyUpdate();
        CHECK(publish(publisher, &command, now_ms) == MESSAGE_OK); ShootTask(); // 以新的发布时刻重复旧人工来源。
        CHECK(shoot_fake_motors[2].ref == 0.0f); // 旧来源不能借许可恢复继续供弹。
        ++now_ms; inputs.manual.source_time_ms = now_ms; // 操作者提供恢复后的新来源。
        CHECK(RobotSafetyPublishControlInputs(&inputs)); RobotSafetyUpdate(); // 更新安全事实。
        CHECK(publish(publisher, &command, now_ms) == MESSAGE_OK); ShootTask(); // 发布新的连续供弹命令。
        CHECK(shoot_fake_motors[2].ref > 0.0f); // 新来源允许重新接管。
        ++now_ms; inputs.manual_fire_requested = 0u; // 发射任务不运行时先撤销击发。
        CHECK(RobotSafetyPublishControlInputs(&inputs)); RobotSafetyUpdate(); // 安全管理记录撤销边沿。
        ++now_ms; inputs.manual_fire_requested = 1u; // 随后恢复请求，但来源仍是撤销前的值。
        CHECK(RobotSafetyPublishControlInputs(&inputs)); RobotSafetyUpdate(); ShootTask(); // 两次变化均发生在发射任务周期之间。
        CHECK(!shoot_fake_motors[2].enabled && shoot_fake_motors[2].ref == 0.0f); // 即使恢复得很快，旧代际也不能供弹。
        CHECK(publish(publisher, &command, now_ms) == MESSAGE_OK); ShootTask(); // 只重发旧来源不能绕过恢复屏障。
        CHECK(shoot_fake_motors[2].ref == 0.0f); // 没有恢复后的新人工输入仍禁止拨弹。
        ++now_ms; inputs.manual.source_time_ms = now_ms; // 最后提供真正的新人工来源。
        CHECK(RobotSafetyPublishControlInputs(&inputs)); RobotSafetyUpdate(); // 新来源通过恢复检查。
        CHECK(publish(publisher, &command, now_ms) == MESSAGE_OK); ShootTask(); // 发布新命令。
        CHECK(shoot_fake_motors[2].ref > 0.0f); // 新来源才能重新供弹。
        return 0;
    }
    if (argc == 2 && strcmp(argv[1], "safety-generation") == 0)
    {
        CHECK(ShootInit() == 1u); // 初始化生产发射任务与测试电机。
        MessagePublisher_t *publisher = MessagePublisherRegister(ROBOT_TOPIC_SHOOT_COMMAND, ROBOT_TOPIC_ENDPOINT_COMMAND); // 使用实际消息中心发布。
        CHECK(publisher != NULL); // 必须取得命令话题。
        Shoot_Ctrl_Cmd_s command = {.shoot_mode = SHOOT_ON, .friction_mode = FRICTION_ON,
            .load_mode = LOAD_BURSTFIRE, .shoot_rate = 8.0f, .bullet_speed = (Bullet_Speed_e)99}; // 使用人工连续供弹请求。
        now_ms = 1u; // 让命令具有明确的本机时间。
        CHECK(publish(publisher, &command, now_ms) == MESSAGE_OK); // 发布故障前命令。
        ShootTask(); // 执行生产发射任务。
        CHECK(shoot_fake_motors[2].enabled && shoot_fake_motors[2].ref > 0.0f); // 故障前允许供弹。
        RobotFaultReport(ROBOT_FAULT_CAN_BUSOFF); // 两次发射任务之间发生总线故障。
        RobotFaultClear(ROBOT_FAULT_CAN_BUSOFF); // 模拟故障立即恢复。
        RobotSafetyUpdate(); // 整车可能再次 READY，但旧命令代际已失效。
        ++now_ms; // 旧命令仍在消息期限内。
        ShootTask(); // 不发布新命令，直接消费旧值。
        CHECK(!shoot_fake_motors[2].enabled && shoot_fake_motors[2].ref == 0.0f); // 旧值不得在恢复后继续供弹。
        CHECK(publish(publisher, &command, now_ms) == MESSAGE_OK); // 恢复后的新命令使用当前代际。
        ShootTask(); // 再次运行发射任务。
        CHECK(shoot_fake_motors[2].enabled && shoot_fake_motors[2].ref > 0.0f); // 新连续命令才可重新接管。
        return 0;
    }
    if (argc == 2 && strncmp(argv[1], "release-", 8u) == 0)
    {
        CHECK(ShootInit() == 1u); // 清理对象必须是已经完成初始化的机构。
        fake_rollback_failure = (uint8_t)(strcmp(argv[1], "release-failure") == 0); // 随后模拟底层清理失败。
        CHECK(ShootReleaseStartupDevices() == (uint8_t)!fake_rollback_failure);
        CHECK(fake_rollback_count == (fake_rollback_failure ? 0u : 3u)); // 两个摩擦轮加一个拨弹盘。
        CHECK(ShootReleaseStartupDevices() == 1u); // 不再次释放旧实例。
        return 0;
    }
    if (argc == 2) // 独立进程执行启动失败场景，避免复用已经注册的资源。
    {
        const uint32_t scenario = (uint32_t)strtoul(argv[1], NULL, 10); // 参数选择失败场景。
        fake_fail_attempt = scenario; // 默认直接作为申请序号。
        if (scenario >= 5u && scenario <= 9u) // 验证四摩擦轮机构的每个电机申请阶段。
        {
            fake_fail_attempt -= 4u; // 场景 5～9 对应申请序号 1～5。
            shoot_config.shoot_type = FOUR_SHOOT; // 启用全部四个摩擦轮。
            shoot_config.friction_motor_id[2] = 5u; // 第三个电机使用独立编号。
            shoot_config.friction_motor_id[3] = 6u; // 第四个电机使用独立编号。
            shoot_config.friction_motor_reverse_flag[2] = MOTOR_DIRECTION_NORMAL; // 第三个电机按正向安装。
            shoot_config.friction_motor_reverse_flag[3] = MOTOR_DIRECTION_NORMAL; // 第四个电机按正向安装。
        }
        if (scenario == 4u) { fake_fail_attempt = 3u; fake_rollback_failure = 1u; } // 第四种场景让回退自身失败。
#if defined(MC02_TEST_WRAP_SUBSCRIBER)
        if (scenario == 10u) { fake_fail_attempt = 0u; fake_subscriber_failure = 1u; } // 所有电机注册后拒绝话题。
#endif
        CHECK(ShootInit() == 0u); // 申请失败不得发布完整发射模块。
        if (fake_rollback_failure)
        {
            CHECK(ShootInitRollbackFailed() != 0u); // 无法回退必须升级为启动故障。
        }
        else
        {
            CHECK(ShootInitRollbackFailed() == 0u); // 成功清理后允许按模块重要性降级。
            const uint32_t expected = scenario == 10u ? 3u : fake_fail_attempt - 1u; // 话题失败时需要撤销全部三台电机。
            CHECK(fake_rollback_count == expected); // 所有先前成功电机均被撤销。
            for (uint32_t i = 0u; i < fake_rollback_count; ++i) CHECK(fake_rolled_back[i]); // 逐个核对已申请实例。
            fake_fail_attempt = 0u; // 后续测试不再注入注册错误。
#if defined(MC02_TEST_WRAP_SUBSCRIBER)
            fake_subscriber_failure = 0u; // 恢复正常话题登记。
#endif
            fake_init_attempt = 0u; // 从第一次申请重新开始计数。
            shoot_fake_motor_init_count = 0u; // 替身允许复用先前撤销的实例槽。
            memset(fake_rolled_back, 0, sizeof(fake_rolled_back)); // 清除替身的旧释放记录。
            CHECK(ShootInit() == 1u); // 清理后相同配置可再次完成初始化。
        }
        return 0;
    }
    CHECK(ShootFrictionWheelCount(DUAL_SHOOT) == 2u);
    CHECK(ShootFrictionWheelCount(THREE_SHOOT) == 3u);
    CHECK(ShootFrictionWheelCount(FOUR_SHOOT) == 4u);
    CHECK(ShootFrictionWheelCount((Shoot_Type_e)99) == 0u);

    timeline_ms = 1000.0f;
    CHECK(ShootInit() == 1u);
    CHECK(shoot_fake_motor_init_count == 3u);
    CHECK((shoot_fake_motors[2].close_loop_type & ANGLE_LOOP) != 0);
    MessageSubscriber_t *observer = MessageSubscriberRegister(ROBOT_TOPIC_SHOOT_COMMAND, ROBOT_TOPIC_ENDPOINT_SHOOT);
    MessagePublisher_t *publisher = MessagePublisherRegister(ROBOT_TOPIC_SHOOT_COMMAND, ROBOT_TOPIC_ENDPOINT_COMMAND);
    CHECK(observer != NULL && publisher != NULL);

    Shoot_Ctrl_Cmd_s command = {
        .shoot_mode = SHOOT_ON,
        .load_mode = LOAD_STOP,
        .lid_mode = LID_CLOSE,
        .friction_mode = FRICTION_ON,
        .bullet_speed = (Bullet_Speed_e)99,
    };
    now_ms = 0u;
    CHECK(publish(publisher, &command, now_ms) == MESSAGE_OK);
    Shoot_Ctrl_Cmd_s published_command = {0};
    const MessageStatus_t read_status = MessageReadSnapshot(observer, &published_command,
        sizeof(published_command), now_ms, NULL);
    CHECK(read_status == MESSAGE_OK);
    CHECK(published_command.shoot_mode == command.shoot_mode &&
          published_command.friction_mode == command.friction_mode &&
          published_command.load_mode == command.load_mode); // 核对实际发布的发射目标。
    RobotSafetySnapshot_t published_safety = {0}; // 同时核对发布者附加的安全代际。
    CHECK(RobotSafetyGetSnapshot(&published_safety)); // 测试中安全管理应可读取。
    CHECK(published_command.safety_generation == published_safety.control_generation &&
          published_command.fire_source == ROBOT_CONTROL_SOURCE_MANUAL); // 代际和来源由发布者明确填写。
    ShootTask();
    CHECK(shoot_fake_motors[0].enabled && shoot_fake_motors[1].enabled && shoot_fake_motors[2].enabled);
    const float friction_speed = 30000.0f * 0.01745329252f / 19.0f;
    CHECK(fabsf(shoot_fake_motors[0].ref - friction_speed) < 1e-4f &&
          fabsf(shoot_fake_motors[1].ref - friction_speed) < 1e-4f);
    CHECK(shoot_fake_motors[0].output_axis.direction == MOTOR_AXIS_DIRECTION_NORMAL &&
          shoot_fake_motors[1].output_axis.direction == MOTOR_AXIS_DIRECTION_REVERSE);

    command.load_mode = LOAD_1_BULLET;
    now_ms = 1u;
    timeline_ms = 1001.0f;
    CHECK(publish(publisher, &command, now_ms) == MESSAGE_OK);
    ShootTask();
    const float expected_one_bullet_target =
        102.0f * 0.01745329252f / 36.0f + 0.62831853072f;
    if (fabsf(shoot_fake_motors[2].ref - expected_one_bullet_target) >= 1e-5f)
        fprintf(stderr, "loader target got %.9f, expected %.9f\n",
                shoot_fake_motors[2].ref, expected_one_bullet_target);
    CHECK(fabsf(shoot_fake_motors[2].ref - expected_one_bullet_target) < 1e-5f);

    /* No later publication: age 51ms must disable every motor and clear references. */
    now_ms = 52u;
    ShootTask();
    CHECK(!shoot_fake_motors[0].enabled && !shoot_fake_motors[1].enabled && !shoot_fake_motors[2].enabled);
    CHECK(shoot_fake_motors[0].ref == 0.0f && shoot_fake_motors[1].ref == 0.0f && shoot_fake_motors[2].ref == 0.0f);

    /* A shot cannot re-arm from an old trigger; a fresh LOAD_STOP is required first. */
    command.load_mode = LOAD_1_BULLET;
    now_ms = 53u;
    CHECK(publish(publisher, &command, now_ms) == MESSAGE_OK);
    ShootTask();
    CHECK(shoot_fake_motors[2].ref == 0.0f);

    command.load_mode = LOAD_STOP;
    now_ms = 54u;
    CHECK(publish(publisher, &command, now_ms) == MESSAGE_OK);
    ShootTask();
    CHECK(shoot_fake_motors[2].ref == 0.0f);

    command.load_mode = LOAD_3_BULLET;
    now_ms = 55u;
    timeline_ms = 1300.0f;
    CHECK(publish(publisher, &command, now_ms) == MESSAGE_OK);
    ShootTask();
    const float expected_three_bullet_target =
        102.0f * 0.01745329252f / 36.0f + 3.0f * 0.62831853072f;
    CHECK(fabsf(shoot_fake_motors[2].ref - expected_three_bullet_target) < 1e-5f);

    shoot_config.loader_motor_config.can_init_config.tx_id = 1u;
    shoot_fake_motor_init_count = 0u;
    CHECK(ShootInit() == 0u);
    CHECK(shoot_fake_motor_init_count == 0u);
    shoot_config.loader_motor_config.can_init_config.tx_id = 3u;

    shoot_config.shoot_type = THREE_SHOOT;
    shoot_config.friction_motor_id[2] = 5u;
    shoot_config.friction_motor_reverse_flag[2] = MOTOR_DIRECTION_NORMAL;
    shoot_fake_motor_init_count = 0u;
    CHECK(ShootInit() == 1u);
    CHECK(shoot_fake_motor_init_count == 4u);
    Shoot_Motor_Summary_t summary = {0};
    Shoot_GetMotorSummary(&summary);
    CHECK(summary.friction_count == 3u);

    shoot_config.shoot_type = FOUR_SHOOT;
    shoot_config.friction_motor_id[2] = 5u;
    shoot_config.friction_motor_id[3] = 6u;
    shoot_config.friction_motor_reverse_flag[2] = MOTOR_DIRECTION_NORMAL;
    shoot_config.friction_motor_reverse_flag[3] = MOTOR_DIRECTION_REVERSE;
    shoot_fake_motor_init_count = 0u;
    CHECK(ShootInit() == 1u);
    CHECK(shoot_fake_motor_init_count == 5u);
    Shoot_GetMotorSummary(&summary);
    CHECK(summary.friction_count == 4u);

    puts("PASS: production shooter initialization, expiry shutdown, and configured friction wheel count");
    return 0;
}
