#include "profile_io.h"
#include "../../../bsp/can/can_tx.h"

/** @brief 记录生产电机任务推进 CAN 发送调度器的次数。 */
void CANProcessTx(void);
typedef struct FDCAN_HandleTypeDef FDCAN_HandleTypeDef; // 适配器只用空句柄表示全部总线。
void CANProcessTxChecked(FDCAN_HandleTypeDef *handle, CAN_TxCheckStartFn_t check, void *context);
