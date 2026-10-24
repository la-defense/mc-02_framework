/** @file robot_cmd_v2_test.c
 * @brief 验证V2瞄准和导航的独立期限、运行许可及撤销行为。
 * @note 使用生产RobotCMD和消息中心；外部输入由本地主机替身提供。
 */
#include "test_check.h"
#include "robot_def.h"
#include "robot_cmd.h"
#include "robot_safety.h"
#include "remote_control.h"
#include "message_center.h"
#include "topics/robot_topics.h"
#include "vision_protocol_v2.h"
#include "referee_snapshot.h"
#include "rm_referee.h"

#include <math.h>
#include <string.h>

UART_HandleTypeDef huart5, huart9;
CAN_HandleTypeDef hcan1;

static RC_ctrl_t remote_input;
static uint32_t test_tick = 100u;
static uint8_t remote_online = 1u;
static uint8_t pc_valid = 1u;
static uint8_t vision_online = 1u;
static Vision_Recv_s unused_wire_frame;
static RefereeSnapshot_t referee_snapshot;

uint32_t HAL_GetTick(void) { return test_tick; }
RC_ctrl_t *RemoteControlInit(UART_HandleTypeDef *handle) { (void)handle; return &remote_input; }
void RemoteControlGetSnapshot(RemoteControlSnapshot_s *snapshot)
{
    snapshot->input = remote_input;
    snapshot->remote_online = remote_online;
    snapshot->pc_valid = pc_valid;
}
Vision_Recv_s *VisionInit(UART_HandleTypeDef *handle) { (void)handle; return &unused_wire_frame; }
void VisionSend(void) {}
void VisionGetStatus(Vision_Status_t *status)
{
    memset(status, 0, sizeof(*status));
    status->online = vision_online;
    status->has_valid_frame = vision_online;
    status->last_rx_ms = test_tick;
}
void VisionGetSnapshot(Vision_Recv_s *frame, Vision_Status_t *status)
{
    if (frame != NULL) *frame = unused_wire_frame;
    if (status != NULL) VisionGetStatus(status);
}
RefereeStatus_t RefereeGetSnapshot(RefereeSnapshot_t *output)
{
    if (output == NULL) return REFEREE_ERROR_ARGUMENT;
    *output = referee_snapshot;
    return REFEREE_OK;
}

static int PublishVision(MessagePublisher_t *publisher, const void *message,
                         uint32_t source_time_ms, MessageClockDomain_t clock)
{
    return MessagePublishAt(publisher, message, source_time_ms, test_tick, test_tick, clock) == MESSAGE_OK ? 0 : 1;
}

static void PrepareReferee(void)
{
    memset(&referee_snapshot, 0, sizeof(referee_snapshot));
    referee_snapshot.command[REF_CMD_ROBOT_STATE].state = REFEREE_DATA_FRESH;
    referee_snapshot.command[REF_CMD_ROBOT_STATE].data.robot_state.robot_id = 3u;
    referee_snapshot.command[REF_CMD_ROBOT_STATE].data.robot_state.power_output_bits =
        REFEREE_OUTPUT_CHASSIS_MASK;
    referee_snapshot.command[REF_CMD_POWER_HEAT].state = REFEREE_DATA_FRESH;
}

int main(void)
{
    remote_input.rc.switch_left = RC_SW_UP;
    RobotSafetyInit();
    RobotSafetySetRcOnline(1u);
    RobotSafetySetImuValid(1u);
    RobotSafetySetCalibValid(1u);
    RobotSafetyRequestEnable();
    RobotSafetyUpdate();
    TEST_CHECK(RobotSafetyIsReady());
    TEST_CHECK(RobotCMDInit());

    MessagePublisher_t *feedback = MessagePublisherRegister(ROBOT_TOPIC_GIMBAL_FEEDBACK,
        ROBOT_TOPIC_ENDPOINT_GIMBAL);
    MessagePublisher_t *aim_publisher = MessagePublisherRegister(ROBOT_TOPIC_VISION_AIM,
        ROBOT_TOPIC_ENDPOINT_VISION);
    MessagePublisher_t *nav_publisher = MessagePublisherRegister(ROBOT_TOPIC_VISION_NAVIGATION,
        ROBOT_TOPIC_ENDPOINT_VISION);
    MessageSubscriber_t *chassis_subscriber = MessageSubscriberRegister(ROBOT_TOPIC_CHASSIS_COMMAND,
        ROBOT_TOPIC_ENDPOINT_CHASSIS);
    MessageSubscriber_t *gimbal_subscriber = MessageSubscriberRegister(ROBOT_TOPIC_GIMBAL_COMMAND,
        ROBOT_TOPIC_ENDPOINT_GIMBAL);
    MessageSubscriber_t *shoot_subscriber = MessageSubscriberRegister(ROBOT_TOPIC_SHOOT_COMMAND,
        ROBOT_TOPIC_ENDPOINT_SHOOT);
    TEST_CHECK(feedback != NULL && aim_publisher != NULL && nav_publisher != NULL &&
        chassis_subscriber != NULL && gimbal_subscriber != NULL && shoot_subscriber != NULL);

    Gimbal_Upload_Data_s pose = {0};
    pose.gimbal_imu_data.YawTotalAngle = 10.0f;
    pose.gimbal_imu_data.Pitch = 2.0f;
    TEST_CHECK(MessagePublishNow(feedback, &pose) == MESSAGE_OK);
    PrepareReferee();

    VisionV2AimCommand_t aim = {
        .mode = 1u, .yaw_rad = 0.1f, .yaw_rate_rad_s = 0.2f,
        .yaw_accel_rad_s2 = 0.3f, .pitch_rad = 0.05f,
        .pitch_rate_rad_s = -0.1f, .pitch_accel_rad_s2 = -0.2f,
    };
    VisionV2NavigationCommand_t navigation = {
        .valid_flags = VISION_V2_NAV_TRANSLATION_VALID | VISION_V2_NAV_YAW_RATE_VALID,
        .vx_m_s = 0.5f, .vy_m_s = -0.25f, .yaw_rate_rad_s = 0.1f,
    };
    TEST_CHECK(PublishVision(aim_publisher, &aim, 77u, MESSAGE_CLOCK_REMOTE_RAW) == 0);
    TEST_CHECK(PublishVision(nav_publisher, &navigation, 77u, MESSAGE_CLOCK_REMOTE_RAW) == 0);
    RobotCMDTask();

    Chassis_Ctrl_Cmd_s chassis;
    Gimbal_Ctrl_Cmd_s gimbal;
    Shoot_Ctrl_Cmd_s shoot;
    TEST_CHECK(MessageReadSnapshot(chassis_subscriber, &chassis, sizeof(chassis),
        test_tick, NULL) == MESSAGE_OK);
    TEST_CHECK(chassis.chassis_mode == CHASSIS_NO_FOLLOW && chassis.vx == 500.0f &&
        chassis.vy == -250.0f && fabsf(chassis.wz - 5.72957795f) < 0.0001f);
    TEST_CHECK(MessageReadSnapshot(gimbal_subscriber, &gimbal, sizeof(gimbal),
        test_tick, NULL) == MESSAGE_OK);
    TEST_CHECK(gimbal.gimbal_mode == GIMBAL_GYRO_MODE &&
        fabsf(gimbal.yaw - 5.72957795f) < 0.0001f &&
        fabsf(gimbal.yaw_vel - 11.4591559f) < 0.0001f);
    TEST_CHECK(MessageReadSnapshot(shoot_subscriber, &shoot, sizeof(shoot),
        test_tick, NULL) == MESSAGE_OK);
    TEST_CHECK(shoot.shoot_mode == SHOOT_OFF && shoot.friction_mode == FRICTION_OFF &&
        shoot.load_mode == LOAD_STOP);

    // 新导航不能延长旧瞄准；瞄准过期后云台保持当前姿态，导航仍可独立使用。
    test_tick += VISION_V2_AIM_MAX_AGE_MS + 1u;
    pose.gimbal_imu_data.YawTotalAngle = 12.0f;
    TEST_CHECK(MessagePublishNow(feedback, &pose) == MESSAGE_OK);
    TEST_CHECK(PublishVision(nav_publisher, &navigation, 78u, MESSAGE_CLOCK_REMOTE_RAW) == 0);
    RobotCMDTask();
    TEST_CHECK(MessageReadSnapshot(chassis_subscriber, &chassis, sizeof(chassis),
        test_tick, NULL) == MESSAGE_OK && chassis.vx == 500.0f);
    TEST_CHECK(MessageReadSnapshot(gimbal_subscriber, &gimbal, sizeof(gimbal),
        test_tick, NULL) == MESSAGE_OK && gimbal.yaw == 12.0f &&
        gimbal.yaw_vel == 0.0f && gimbal.pitch_vel == 0.0f);
    TEST_CHECK(MessageReadSnapshot(shoot_subscriber, &shoot, sizeof(shoot),
        test_tick, NULL) == MESSAGE_OK && shoot.shoot_mode == SHOOT_OFF);

    // 导航期限和裁判许可任一失效都会清零输出，旧命令不继续使用。
    test_tick += VISION_V2_NAV_MAX_AGE_MS + 1u;
    TEST_CHECK(MessagePublishNow(feedback, &pose) == MESSAGE_OK);
    RobotCMDTask();
    TEST_CHECK(MessageReadSnapshot(chassis_subscriber, &chassis, sizeof(chassis),
        test_tick, NULL) == MESSAGE_OK && chassis.chassis_mode == CHASSIS_ZERO_FORCE &&
        chassis.vx == 0.0f && chassis.vy == 0.0f && chassis.wz == 0.0f);

    test_tick += 1u;
    navigation.vx_m_s = 0.25f;
    TEST_CHECK(PublishVision(nav_publisher, &navigation, 79u, MESSAGE_CLOCK_REMOTE_RAW) == 0);
    referee_snapshot.command[REF_CMD_POWER_HEAT].state = REFEREE_DATA_EXPIRED;
    TEST_CHECK(MessagePublishNow(feedback, &pose) == MESSAGE_OK);
    RobotCMDTask();
    TEST_CHECK(MessageReadSnapshot(chassis_subscriber, &chassis, sizeof(chassis),
        test_tick, NULL) == MESSAGE_OK && chassis.chassis_mode == CHASSIS_ZERO_FORCE &&
        chassis.vx == 0.0f && chassis.vy == 0.0f && chassis.wz == 0.0f);

    /* 遥控器未选择自主档、PC未就绪或USB链路离线时都不得沿用导航移动。 */
    referee_snapshot.command[REF_CMD_ROBOT_STATE].state = REFEREE_DATA_FRESH;
    referee_snapshot.command[REF_CMD_POWER_HEAT].state = REFEREE_DATA_FRESH;
    test_tick += 1u;
    remote_input.rc.switch_left = RC_SW_UP;
    pc_valid = 1u;
    vision_online = 1u;
    TEST_CHECK(PublishVision(nav_publisher, &navigation, 80u, MESSAGE_CLOCK_REMOTE_RAW) == 0);
    TEST_CHECK(MessagePublishNow(feedback, &pose) == MESSAGE_OK);
    RobotCMDTask();
    TEST_CHECK(MessageReadSnapshot(chassis_subscriber, &chassis, sizeof(chassis),
        test_tick, NULL) == MESSAGE_OK && chassis.chassis_mode == CHASSIS_NO_FOLLOW &&
        chassis.vx == 250.0f);

    test_tick += 1u;
    remote_input.rc.switch_left = RC_SW_MID;
    TEST_CHECK(PublishVision(nav_publisher, &navigation, 81u, MESSAGE_CLOCK_REMOTE_RAW) == 0);
    TEST_CHECK(MessagePublishNow(feedback, &pose) == MESSAGE_OK);
    RobotCMDTask();
    TEST_CHECK(MessageReadSnapshot(chassis_subscriber, &chassis, sizeof(chassis),
        test_tick, NULL) == MESSAGE_OK && chassis.chassis_mode == CHASSIS_ZERO_FORCE &&
        chassis.vx == 0.0f && chassis.vy == 0.0f && chassis.wz == 0.0f);

    remote_input.rc.switch_left = RC_SW_UP;
    pc_valid = 0u;
    test_tick += 1u;
    TEST_CHECK(PublishVision(nav_publisher, &navigation, 82u, MESSAGE_CLOCK_REMOTE_RAW) == 0);
    TEST_CHECK(MessagePublishNow(feedback, &pose) == MESSAGE_OK);
    RobotCMDTask();
    TEST_CHECK(MessageReadSnapshot(chassis_subscriber, &chassis, sizeof(chassis),
        test_tick, NULL) == MESSAGE_OK && chassis.chassis_mode == CHASSIS_ZERO_FORCE &&
        chassis.vx == 0.0f && chassis.vy == 0.0f && chassis.wz == 0.0f);

    pc_valid = 1u;
    vision_online = 0u;
    test_tick += 1u;
    TEST_CHECK(PublishVision(nav_publisher, &navigation, 83u, MESSAGE_CLOCK_REMOTE_RAW) == 0);
    TEST_CHECK(MessagePublishNow(feedback, &pose) == MESSAGE_OK);
    RobotCMDTask();
    TEST_CHECK(MessageReadSnapshot(chassis_subscriber, &chassis, sizeof(chassis),
        test_tick, NULL) == MESSAGE_OK && chassis.chassis_mode == CHASSIS_ZERO_FORCE &&
        chassis.vx == 0.0f && chassis.vy == 0.0f && chassis.wz == 0.0f);

    vision_online = 1u;
    remote_online = 0u;
    test_tick += 1u;
    TEST_CHECK(PublishVision(nav_publisher, &navigation, 84u, MESSAGE_CLOCK_REMOTE_RAW) == 0);
    TEST_CHECK(MessagePublishNow(feedback, &pose) == MESSAGE_OK);
    RobotCMDTask();
    TEST_CHECK(MessageReadSnapshot(chassis_subscriber, &chassis, sizeof(chassis),
        test_tick, NULL) == MESSAGE_OK && chassis.chassis_mode == CHASSIS_ZERO_FORCE &&
        chassis.vx == 0.0f && chassis.vy == 0.0f && chassis.wz == 0.0f);

    puts("PASS: V2 aim/navigation expire independently, require current permits, and never request firing");
    return 0;
}
