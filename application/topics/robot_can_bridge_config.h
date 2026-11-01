/** @file robot_can_bridge_config.h
 * @brief 车型声明的板间CAN总线和底盘话题绑定。
 */
#ifndef ROBOT_CAN_BRIDGE_CONFIG_H
#define ROBOT_CAN_BRIDGE_CONFIG_H

#include "../../bsp/board/platform_resources.h"
#include <stdint.h>

/** @brief 一块板所使用的板间总线与话题；修改后须两端同时核对。 */
typedef struct {
    PlatformCanBus_t can_bus; /**< 本板板级声明的逻辑 CAN 总线。 */
    uint16_t tx_id; /**< 本板发送标准ID。 */
    uint16_t rx_id; /**< 本板接收标准ID。 */
    uint16_t command_topic; /**< 底盘命令话题编号。 */
    uint16_t feedback_topic; /**< 底盘反馈话题编号。 */
} RobotCanBridgeConfig_t;

#endif
