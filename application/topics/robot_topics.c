/** @file robot_topics.c
 * @brief 声明单板控制与新版视觉业务的固定话题关系。
 * @note 当前应用话题均为最新快照；跨设备时间映射尚由后续对时模块提供。
 */
#include "robot_topics.h"

#include "message_center.h"
#include "../robot_def.h"
#include "vision_protocol_v2.h"
#include "vision_imu_sample.h"

#define ENDPOINT_MASK(endpoint) (1u << ((endpoint) - 1u))
#define SNAPSHOT_TOPIC(id, type, age, clock, publisher, subscriber) \
    { (id), (uint16_t)sizeof(type), (age), 1u, MESSAGE_DELIVERY_SNAPSHOT, \
      MESSAGE_OVERFLOW_REPLACE_OLDEST, (clock), ENDPOINT_MASK(publisher), \
      ENDPOINT_MASK(subscriber) }

static const MessageTopicDescriptor_t robot_topics[] = {
    SNAPSHOT_TOPIC(ROBOT_TOPIC_CHASSIS_COMMAND, Chassis_Ctrl_Cmd_s, 50u,
                   MESSAGE_CLOCK_LOCAL, ROBOT_TOPIC_ENDPOINT_COMMAND,
                   ROBOT_TOPIC_ENDPOINT_CHASSIS),
    SNAPSHOT_TOPIC(ROBOT_TOPIC_CHASSIS_FEEDBACK, Chassis_Upload_Data_s, 50u,
                   MESSAGE_CLOCK_LOCAL, ROBOT_TOPIC_ENDPOINT_CHASSIS,
                   ROBOT_TOPIC_ENDPOINT_COMMAND),
    SNAPSHOT_TOPIC(ROBOT_TOPIC_GIMBAL_COMMAND, Gimbal_Ctrl_Cmd_s, 50u,
                   MESSAGE_CLOCK_LOCAL, ROBOT_TOPIC_ENDPOINT_COMMAND,
                   ROBOT_TOPIC_ENDPOINT_GIMBAL),
    SNAPSHOT_TOPIC(ROBOT_TOPIC_GIMBAL_FEEDBACK, Gimbal_Upload_Data_s, 50u,
                   MESSAGE_CLOCK_LOCAL, ROBOT_TOPIC_ENDPOINT_GIMBAL,
                   ROBOT_TOPIC_ENDPOINT_COMMAND),
    SNAPSHOT_TOPIC(ROBOT_TOPIC_SHOOT_COMMAND, Shoot_Ctrl_Cmd_s, 50u,
                   MESSAGE_CLOCK_LOCAL, ROBOT_TOPIC_ENDPOINT_COMMAND,
                   ROBOT_TOPIC_ENDPOINT_SHOOT),
    SNAPSHOT_TOPIC(ROBOT_TOPIC_VISION_AIM, VisionV2AimCommand_t,
                   VISION_V2_AIM_MAX_AGE_MS, MESSAGE_CLOCK_REMOTE_RAW,
                   ROBOT_TOPIC_ENDPOINT_VISION, ROBOT_TOPIC_ENDPOINT_COMMAND),
    SNAPSHOT_TOPIC(ROBOT_TOPIC_VISION_NAVIGATION, VisionV2NavigationCommand_t,
                   VISION_V2_NAV_MAX_AGE_MS, MESSAGE_CLOCK_REMOTE_RAW,
                   ROBOT_TOPIC_ENDPOINT_VISION, ROBOT_TOPIC_ENDPOINT_COMMAND),
    SNAPSHOT_TOPIC(ROBOT_TOPIC_VISION_POSTURE, VisionV2PostureStatus_t, 100u,
                   MESSAGE_CLOCK_LOCAL, ROBOT_TOPIC_ENDPOINT_INS,
                   ROBOT_TOPIC_ENDPOINT_VISION),
    // 无效标记要能等到约150ms轮询的IMU发送槽；有效样本另按5ms源期限检查。
    SNAPSHOT_TOPIC(ROBOT_TOPIC_VISION_IMU_SAMPLE, VisionIMUSample_t, 300u,
                   MESSAGE_CLOCK_LOCAL, ROBOT_TOPIC_ENDPOINT_INS,
                   ROBOT_TOPIC_ENDPOINT_VISION),
};

uint8_t RobotTopicsInit(void)
{
    return (uint8_t)(MessageCenterInit(robot_topics,
        sizeof(robot_topics) / sizeof(robot_topics[0])) == MESSAGE_OK);
}
