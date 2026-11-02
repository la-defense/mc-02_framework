/** @file can_tx_public_header.c
 * @brief 检查 CAN 队列公共类型可在没有厂商或 RTOS 头路径时编译。
 */
#include "../../bsp/can/platform_can.h"

CAN_TxFrame_t can_tx_portable_frame;
CAN_TxResult_t can_tx_portable_result;
