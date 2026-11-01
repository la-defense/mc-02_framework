/** @file robot_can_bridge.c
 * @brief 绑定双板底盘命令、反馈和 CAN 板间对时。
 */
#include "robot_can_bridge.h"
#include "robot_topics.h"
#include "chassis_topic_codec.h"
#include "can_topic_link.h"
#include "board_can_stm32.h"
#include "main.h"
#include "fdcan.h"
#include "FreeRTOS.h"
#include "task.h"
#include <string.h>
#include <limits.h>
#if defined(CHASSIS_BOARD)
#include "robot_safety.h"
#endif

static CanTopicLink_t robot_link;
static CanTimeSync_t robot_time_sync;
static TimeMappingState_t published_sync;
static uint32_t published_sync_ms;
static uint8_t published_sync_ready;

#if defined(CHASSIS_BOARD)
static void RobotCanBridgeCommandInvalidated(void *context)
{
    (void)context;
    RobotSafetyRevokeRemoteChassisPermit();
}

static void RobotCanBridgeBeforeCommandPublish(void *context, const void *payload,
                                               size_t payload_size)
{
    (void)context;
    if (payload == NULL || payload_size != sizeof(Chassis_Ctrl_Cmd_s))
    {
        return;
    }
    Chassis_Ctrl_Cmd_s command;
    memcpy(&command, payload, sizeof(command));
    // 先撤销旧许可再发布停车命令，避免发送任务继续沿用旧许可。
    if (command.chassis_mode == CHASSIS_ZERO_FORCE)
    {
        RobotSafetyRevokeRemoteChassisPermit();
    }
}
#endif

uint8_t RobotCanBridgeInit(const RobotCanBridgeConfig_t *config)
{
    // 当前编解码只认识底盘命令和反馈；不支持的车型话题必须明确拒绝。
    if (config==NULL || config->tx_id>0x7ffu ||
        config->rx_id>0x7ffu || config->tx_id==config->rx_id ||
        config->command_topic!=ROBOT_TOPIC_CHASSIS_COMMAND ||
        config->feedback_topic!=ROBOT_TOPIC_CHASSIS_FEEDBACK) return 0u;
    FDCAN_HandleTypeDef *can_handle = BoardCanResolve(config->can_bus); // 在板级绑定中查找实际控制器。
    if (can_handle == NULL) return 0u; // 未声明总线时不能注册话题或通信。
    if (!RobotTopicsInit()) return 0u;
#if defined(CHASSIS_BOARD)
    MessageSubscriber_t *outgoing=MessageSubscriberRegister(config->feedback_topic,ROBOT_TOPIC_ENDPOINT_CAN_BRIDGE);
    MessagePublisher_t *incoming=MessagePublisherRegister(config->command_topic,ROBOT_TOPIC_ENDPOINT_CAN_BRIDGE);
    const CanTopicCodec_t encode={ChassisFeedbackBridgeEncode,NULL,CHASSIS_FEEDBACK_WIRE_BYTES};
    const CanTopicCodec_t decode={ChassisCommandBridgeDecode,NULL,CHASSIS_COMMAND_WIRE_BYTES};
#elif defined(GIMBAL_BOARD)
    MessageSubscriber_t *outgoing=MessageSubscriberRegister(config->command_topic,ROBOT_TOPIC_ENDPOINT_CAN_BRIDGE);
    MessagePublisher_t *incoming=MessagePublisherRegister(config->feedback_topic,ROBOT_TOPIC_ENDPOINT_CAN_BRIDGE);
    const CanTopicCodec_t encode={ChassisCommandBridgeEncode,NULL,CHASSIS_COMMAND_WIRE_BYTES};
    const CanTopicCodec_t decode={ChassisFeedbackBridgeDecode,NULL,CHASSIS_FEEDBACK_WIRE_BYTES};
#else
#error "robot_can_bridge requires a dual-board role"
#endif
    if (outgoing==NULL || incoming==NULL) return 0u;
    const CAN_Init_Config_s can_config={.can_handle=can_handle,.tx_id=config->tx_id,.rx_id=config->rx_id};
    if (!CanTopicLinkInit(&robot_link,&can_config,outgoing,incoming,&encode,&decode,NULL,NULL,50u))
        return 0u;
#if defined(CHASSIS_BOARD)
    // 时钟失效或新停车命令通过检查后，先撤销旧许可再让底盘任务看到新话题。
    if (CanTopicBridgeSetInvalidateCallback(&robot_link.bridge,
            RobotCanBridgeCommandInvalidated,NULL)!=CAN_BRIDGE_OK ||
        CanTopicBridgeSetBeforePublishCallback(&robot_link.bridge,
            RobotCanBridgeBeforeCommandPublish,NULL)!=CAN_BRIDGE_OK) {
        CANUnregisterInstance(robot_link.transport.can);
        memset(&robot_link,0,sizeof(robot_link));
        return 0u;
    }
#endif
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
    if (robot_link.ready && robot_link.transport.can!=NULL)
        CANProcessTxBus(robot_link.transport.can->can_handle);
}
