/** @file robot_topics.h
 * @brief 整车控制应用使用的固定话题编号、端点和目录初始化入口。
 * @note 目录必须在发布者、订阅者注册前初始化；话题关系修改需同步验收测试。
 */
#ifndef ROBOT_TOPICS_H
#define ROBOT_TOPICS_H

#include <stdint.h>

enum
{
    ROBOT_TOPIC_CHASSIS_COMMAND = 0x0101,
    ROBOT_TOPIC_CHASSIS_FEEDBACK = 0x0102,
    ROBOT_TOPIC_GIMBAL_COMMAND = 0x0103,
    ROBOT_TOPIC_GIMBAL_FEEDBACK = 0x0104,
    ROBOT_TOPIC_SHOOT_COMMAND = 0x0105,
    ROBOT_TOPIC_VISION_AIM = 0x0201,
    ROBOT_TOPIC_VISION_NAVIGATION = 0x0202,
    ROBOT_TOPIC_VISION_POSTURE = 0x0203,
    ROBOT_TOPIC_VISION_IMU_SAMPLE = 0x0204
};

enum
{
    ROBOT_TOPIC_ENDPOINT_COMMAND = 1,
    ROBOT_TOPIC_ENDPOINT_CHASSIS = 2,
    ROBOT_TOPIC_ENDPOINT_GIMBAL = 3,
    ROBOT_TOPIC_ENDPOINT_SHOOT = 4,
    ROBOT_TOPIC_ENDPOINT_VISION = 5,
    ROBOT_TOPIC_ENDPOINT_INS = 6,
    ROBOT_TOPIC_ENDPOINT_CAN_BRIDGE = 7
};

/** @brief 复制并启动整车固定话题目录；相同目录重复调用安全。
 * @return 1为首次或相同目录已就绪；0为目录校验失败。
 * @note RTOS运行前调用；成功后不再允许增加话题或改变发布订阅关系。
 */
uint8_t RobotTopicsInit(void);

#endif
