/** @file robot_can_bridge.c
 * @brief 绑定双板底盘命令、反馈和 CAN 板间对时。
 */
#include "robot_can_bridge.h"
#include "robot_topics.h"
#include "chassis_topic_codec.h"
#include "can_topic_link.h"
#include "main.h"
#include "fdcan.h"
#include "FreeRTOS.h"
#include "task.h"
#include <string.h>
#include <limits.h>

static CanTopicLink_t robot_link;
static CanTimeSync_t robot_time_sync;
static TimeMappingState_t published_sync;
static uint32_t published_sync_ms;
static uint8_t published_sync_ready;

uint8_t RobotCanBridgeInit(void)
{
    if (!RobotTopicsInit()) return 0u;
#if defined(CHASSIS_BOARD)
    MessageSubscriber_t *outgoing=MessageSubscriberRegister(ROBOT_TOPIC_CHASSIS_FEEDBACK,ROBOT_TOPIC_ENDPOINT_CAN_BRIDGE);
    MessagePublisher_t *incoming=MessagePublisherRegister(ROBOT_TOPIC_CHASSIS_COMMAND,ROBOT_TOPIC_ENDPOINT_CAN_BRIDGE);
    const CanTopicCodec_t encode={ChassisFeedbackBridgeEncode,NULL,CHASSIS_FEEDBACK_WIRE_BYTES};
    const CanTopicCodec_t decode={ChassisCommandBridgeDecode,NULL,CHASSIS_COMMAND_WIRE_BYTES};
    CAN_Init_Config_s config={.can_handle=&hcan2,.tx_id=0x311u,.rx_id=0x312u};
#elif defined(GIMBAL_BOARD)
    MessageSubscriber_t *outgoing=MessageSubscriberRegister(ROBOT_TOPIC_CHASSIS_COMMAND,ROBOT_TOPIC_ENDPOINT_CAN_BRIDGE);
    MessagePublisher_t *incoming=MessagePublisherRegister(ROBOT_TOPIC_CHASSIS_FEEDBACK,ROBOT_TOPIC_ENDPOINT_CAN_BRIDGE);
    const CanTopicCodec_t encode={ChassisCommandBridgeEncode,NULL,CHASSIS_COMMAND_WIRE_BYTES};
    const CanTopicCodec_t decode={ChassisFeedbackBridgeDecode,NULL,CHASSIS_FEEDBACK_WIRE_BYTES};
    CAN_Init_Config_s config={.can_handle=&hcan1,.tx_id=0x312u,.rx_id=0x311u};
#else
#error "robot_can_bridge requires a dual-board role"
#endif
    if (outgoing==NULL || incoming==NULL) return 0u;
    if (!CanTopicLinkInit(&robot_link,&config,outgoing,incoming,&encode,&decode,NULL,NULL,50u))
        return 0u;
    // 对时使用同一个接收端点；失败时释放本次注册，不能启动无映射的双板控制。
    if (!CanTimeSyncInit(&robot_time_sync,robot_link.transport.can) ||
        !CanTopicLinkAttachTimeSync(&robot_link,&robot_time_sync)) {
        CANUnregisterInstance(robot_link.transport.can);
        memset(&robot_link,0,sizeof(robot_link));
        memset(&robot_time_sync,0,sizeof(robot_time_sync));
        return 0u;
    }
    return 1u;
}

void RobotCanBridgeService(void)
{
    const uint32_t now=HAL_GetTick();
    uint32_t request_id=0u; CanTopicBridgeStatus_t sync_status;
    (void)CanTopicLinkService(&robot_link,now,&request_id,&sync_status);
    TimeMappingState_t candidate;
    if (CanTimeSyncGetState(&robot_time_sync,now,&candidate)==TIME_MAPPING_OK) {
        // 先在唯一处理任务中取完整状态，再短暂保护副本供其他任务读取。
        taskENTER_CRITICAL();
        published_sync=candidate;
        published_sync_ms=now;
        published_sync_ready=1u;
        taskEXIT_CRITICAL();
    }
}

uint8_t RobotCanBridgeGetTimeSyncState(TimeMappingState_t *output, uint32_t *updated_ms)
{
    if (output==NULL || updated_ms==NULL) return 0u;
    taskENTER_CRITICAL();
    const uint8_t ready=published_sync_ready;
    TimeMappingState_t candidate=published_sync;
    const uint32_t captured_ms=published_sync_ms;
    taskEXIT_CRITICAL();
    if (!ready) return 0u;
    const uint32_t now=HAL_GetTick();
    const uint32_t copy_age=now-captured_ms;
    if (copy_age>5u) candidate.synchronized=0u;
    if (candidate.has_sample) {
        const uint32_t sample_age=now-candidate.last_local_sample_ms;
        candidate.sample_age_ms=sample_age <= INT32_MAX ? sample_age : UINT32_MAX;
        if (candidate.sample_age_ms>CAN_TIME_SYNC_VALID_MS) candidate.synchronized=0u;
    }
    *output=candidate;
    *updated_ms=captured_ms;
    return 1u;
}

void RobotCanBridgeServiceTx(void)
{
    // 额外机会只作用于当前桥接总线，避免改变其他总线每轮的提交次数。
#if defined(CHASSIS_BOARD)
    CANProcessTxBus(&hcan2);
#else
    CANProcessTxBus(&hcan1);
#endif
}
