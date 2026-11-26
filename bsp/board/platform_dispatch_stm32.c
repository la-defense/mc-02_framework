/** @file platform_dispatch_stm32.c
 * @brief 将任务派发入口绑定到 H723 的现有通信队列。
 */
#include "../os/platform_dispatch.h"
#include "bsp_can.h"
#include "bsp_usart.h"
#include <stddef.h>

void PlatformDispatchCanRx(void)
{
    CANProcessRx(); // 保留原有逐总线接收顺序及处理上限。
}

void PlatformDispatchUartRx(void)
{
    USARTProcessRx(); // 保留原有串口接收和错误恢复流程。
}

void PlatformDispatchCanTx(CAN_TxCheckStartFn_t check, void *context)
{
    CANProcessTxChecked(NULL, check, context); // 每条启用的总线都检查实际提交时刻。
}
