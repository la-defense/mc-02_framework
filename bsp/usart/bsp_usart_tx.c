/** @file bsp_usart_tx.c
 * @brief UART固定复制队列；唯一发送任务推进，ISR仅登记完成或失败。
 * @note 先注册接收对象和创建发送任务，再提交；禁止直接HAL发送绕过本模块。
 */
#include "usart_tx_private.h"
#include "bsp_hal_tick.h"
#include <limits.h>
#include <string.h>

typedef enum
{
    TX_FREE,
    TX_QUEUED,
    TX_STARTING,
    TX_ACTIVE,
    TX_RECOVER,
    TX_ABORTING,
    TX_DONE,
    TX_NOTIFY,
    TX_HELD
} TxPhase;
typedef struct
{
    uint8_t data[USART_TX_MAX_BYTES] __attribute__((aligned(32)));
    USARTTxRequest_t request;
    USARTTxEvent_t event;
    TxPhase phase;
    uint8_t lease, cancel, irq_done, irq_error;
    uint32_t irq_ms;
} TxEntry;
typedef struct
{
    TxEntry entries[USART_TX_DEPTH];
    uint8_t order[USART_TX_DEPTH], count;
    uint32_t next_id;
    USARTTxState_t state;
} TxPort;
// 链接脚本将普通.bss放在0x24000000的AXI RAM，DMA可访问；每项数据按32字节对齐。
static TxPort ports[DEVICE_USART_CNT];
static uint8_t service_ready, processing, recovery_next;

// 无符号差值允许零时刻和毫秒计数回绕；达到期限即停止使用本条。
static uint8_t TxDue(const TxEntry *entry, uint32_t now)
{
    return (uint32_t)(now - entry->event.submitted_ms) >= entry->request.timeout_ms;
}
// 只有硬件已停止时才释放计时；隔离项仍可能被DMA借用，保留其lease。
static void TxFinish(TxEntry *entry, USARTTxResult_t result, uint8_t stopped)
{
    entry->event.result = result;
    entry->event.completed_ms = HAL_GetTick();
    entry->phase = stopped ? TX_DONE : TX_HELD;
    if (stopped && entry->lease)
    {
        BSP_HALTickReleaseBlockingWait();
        entry->lease = 0u;
    }
}
void USARTTxSetServiceReady(uint8_t ready)
{
    uint32_t saved = USARTTxEnter();
    uint8_t empty = 1u;
    for (unsigned p = 0u; p < DEVICE_USART_CNT; ++p)
        if (ports[p].state.used)
            empty = 0u;
    if (empty)
        service_ready = (uint8_t)(ready != 0u);
    USARTTxExit(saved);
}
USARTSendStatus_t USARTSubmitTx(USARTInstance *instance, const USARTTxRequest_t *request, uint32_t *id)
{
    if (!USARTTxTaskContext() || request == NULL || id == NULL || request->data == NULL ||
        request->complete == NULL || request->length == 0u || request->length > USART_TX_MAX_BYTES ||
        request->timeout_ms == 0u || request->timeout_ms > INT32_MAX ||
        (request->mode != USART_TRANSFER_IT && request->mode != USART_TRANSFER_DMA))
        return USART_SEND_BAD_ARGUMENT;
    uint32_t saved = USARTTxEnter();
    int index = USARTRegisteredIndex(instance);
    if (index < 0)
    {
        USARTTxExit(saved);
        return USART_SEND_BAD_ARGUMENT;
    }
    TxPort *port = &ports[index];
    USARTSendStatus_t failure = USART_SEND_ACCEPTED;
    if (!service_ready)
        failure = USART_SEND_NOT_READY;
    else if (port->state.isolated)
        failure = USART_SEND_ISOLATED;
    else if (port->state.used == USART_TX_DEPTH)
        failure = USART_SEND_BUSY;
    else if (!USARTTxConfigured(instance->usart_handle, request->mode))
        failure = USART_SEND_ERROR;
    if (failure != USART_SEND_ACCEPTED || !BSP_HALTickTryAcquireBlockingWait())
    {
        ++port->state.rejected;
        USARTTxExit(saved);
        return failure != USART_SEND_ACCEPTED ? failure : USART_SEND_NOT_READY;
    }
    unsigned slot = 0u;
    while (port->entries[slot].phase != TX_FREE)
        ++slot;
    TxEntry *entry = &port->entries[slot];
    memset(entry, 0, sizeof(*entry));
    // 最多复制128字节，完整内容准备好后才把编号加入可发送列表。
    memcpy(entry->data, request->data, request->length);
    entry->request = *request;
    entry->request.data = entry->data;
    port->next_id = USARTTxNextId(port->next_id);
    entry->event.id = port->next_id;
    entry->event.submitted_ms = HAL_GetTick();
    entry->phase = TX_QUEUED;
    entry->lease = 1u;
    port->order[port->count++] = (uint8_t)slot;
    ++port->state.used;
    ++port->state.accepted;
    *id = entry->event.id;
    USARTTxExit(saved);
    return USART_SEND_ACCEPTED;
}
USARTSendStatus_t USARTCancelTx(USARTInstance *instance, uint32_t id)
{
    if (!USARTTxTaskContext() || id == 0u)
        return USART_SEND_BAD_ARGUMENT;
    uint32_t saved = USARTTxEnter();
    int index = USARTRegisteredIndex(instance);
    USARTSendStatus_t result = index < 0 ? USART_SEND_BAD_ARGUMENT : USART_SEND_NOT_FOUND;
    if (index >= 0)
    {
        TxPort *port = &ports[index];
        for (unsigned i = 0u; i < port->count; ++i)
        {
            TxEntry *entry = &port->entries[port->order[i]];
            if (entry->event.id != id)
                continue;
            if (entry->phase == TX_QUEUED)
                TxFinish(entry, USART_TX_CANCELLED, 1u);
            else if (entry->phase == TX_ACTIVE)
            {
                entry->event.result = USART_TX_CANCELLED;
                entry->phase = TX_RECOVER;
            }
            else if (entry->phase == TX_STARTING)
                entry->cancel = 1u;
            result = entry->phase == TX_HELD ? USART_SEND_ISOLATED : USART_SEND_ACCEPTED;
            break;
        }
    }
    USARTTxExit(saved);
    return result;
}
USARTSendStatus_t USARTGetTxState(const USARTInstance *instance, USARTTxState_t *output)
{
    if (output == NULL || !USARTTxTaskContext())
        return USART_SEND_BAD_ARGUMENT;
    uint32_t saved = USARTTxEnter();
    int index = USARTRegisteredIndex(instance);
    if (index < 0)
    {
        USARTTxExit(saved);
        return USART_SEND_BAD_ARGUMENT;
    }
    *output = ports[index].state;
    output->service_ready = service_ready;
    USARTTxExit(saved);
    return USART_SEND_ACCEPTED;
}
// HAL回调没有事务编号，只能接收当前已认领的事务；恢复和隔离期间忽略迟到通知。
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *handle)
{
    uint32_t saved = USARTTxEnter();
    for (unsigned p = 0u; p < DEVICE_USART_CNT; ++p)
    {
        USARTInstance *instance = USARTRegisteredAt(p);
        if (instance == NULL || instance->usart_handle != handle)
            continue;
        for (unsigned s = 0u; s < USART_TX_DEPTH; ++s)
        {
            TxEntry *entry = &ports[p].entries[s];
            if (entry->phase == TX_STARTING || entry->phase == TX_ACTIVE)
            {
                if (!entry->irq_done)
                {
                    entry->irq_done = 1u;
                    entry->irq_ms = HAL_GetTick();
                }
                break;
            }
        }
        break;
    }
    USARTTxExit(saved);
}
void USARTTxErrorISR(UART_HandleTypeDef *handle)
{
    uint32_t saved = USARTTxEnter();
    for (unsigned p = 0u; p < DEVICE_USART_CNT; ++p)
    {
        USARTInstance *instance = USARTRegisteredAt(p);
        if (instance == NULL || instance->usart_handle != handle)
            continue;
        for (unsigned s = 0u; s < USART_TX_DEPTH; ++s)
        {
            TxEntry *entry = &ports[p].entries[s];
            if (entry->phase == TX_STARTING || entry->phase == TX_ACTIVE)
                entry->irq_error = 1u;
        }
    }
    USARTTxExit(saved);
}
// 先汇总ISR登记，完成时间按UART TC发生时刻判断，不把任务调度延迟算作传输超时。
static void TxUpdate(TxPort *port, USARTInstance *instance)
{
    uint32_t saved = USARTTxEnter();
    for (unsigned s = 0u; s < USART_TX_DEPTH; ++s)
    {
        TxEntry *entry = &port->entries[s];
        if (entry->phase == TX_QUEUED && TxDue(entry, HAL_GetTick()))
            TxFinish(entry, USART_TX_EXPIRED, 1u);
        if (entry->phase != TX_ACTIVE)
            continue;
        if (entry->irq_error || (entry->irq_done && TxDue(entry, entry->irq_ms)) ||
            (!entry->irq_done && TxDue(entry, HAL_GetTick())))
        {
            entry->event.result = entry->irq_error ? USART_TX_FAILED : USART_TX_EXPIRED;
            entry->phase = TX_RECOVER;
        }
        else if (entry->irq_done)
        {
            if (USARTTxStopped(instance->usart_handle, entry->request.mode))
            {
                USARTTxClear(instance->usart_handle, entry->request.mode);
                TxFinish(entry, USART_TX_COMPLETE, 1u);
                entry->event.completed_ms = entry->irq_ms;
            }
            else
            {
                entry->event.result = USART_TX_FAILED;
                entry->phase = TX_RECOVER;
            }
        }
    }
    USARTTxExit(saved);
}
// 只通知队首，保持提交顺序；业务回调在开中断后执行。
static void TxNotify(TxPort *port)
{
    uint32_t saved = USARTTxEnter();
    if (port->count == 0u)
    {
        USARTTxExit(saved);
        return;
    }
    TxEntry *entry = &port->entries[port->order[0]];
    if (entry->phase != TX_DONE && entry->phase != TX_HELD)
    {
        USARTTxExit(saved);
        return;
    }
    uint8_t held = (uint8_t)(entry->phase == TX_HELD);
    USARTTxEvent_t event = entry->event;
    USARTTxCallback_t callback = entry->request.complete;
    void *context = entry->request.context;
    entry->phase = TX_NOTIFY;
    port->state.last_event = event;
    if (event.result == USART_TX_COMPLETE)
        ++port->state.completed;
    else
        ++port->state.failed;
    USARTTxExit(saved);
    callback(context, &event);
    saved = USARTTxEnter();
    // 通知结束后才回收名额；隔离项只移出通知列表，仍保留数据和lease。
    for (unsigned i = 1u; i < port->count; ++i)
        port->order[i - 1u] = port->order[i];
    --port->count;
    entry->phase = held ? TX_HELD : TX_FREE;
    if (!held)
        --port->state.used;
    USARTTxExit(saved);
}
// 先认领队首再检查业务身份；HAL_BUSY只代表尚未启动，允许继续等到总期限。
static void TxStartNext(TxPort *port, USARTInstance *instance)
{
    uint32_t saved = USARTTxEnter();
    if (port->state.isolated || port->count == 0u)
    {
        USARTTxExit(saved);
        return;
    }
    TxEntry *entry = &port->entries[port->order[0]];
    if (entry->phase != TX_QUEUED)
    {
        USARTTxExit(saved);
        return;
    }
    entry->phase = TX_STARTING;
    USARTTxExit(saved);
    uint8_t permitted = (instance->tx_guard == NULL ||
                         instance->tx_guard(instance->context, entry->data, entry->request.length)) &&
                        (entry->request.guard == NULL ||
                         entry->request.guard(entry->request.context, entry->data, entry->request.length));
    saved = USARTTxEnter();
    if (!permitted || entry->cancel || TxDue(entry, HAL_GetTick()))
    {
        TxFinish(entry, TxDue(entry, HAL_GetTick()) ? USART_TX_EXPIRED : USART_TX_CANCELLED, 1u);
        USARTTxExit(saved);
        return;
    }
    entry->event.started_ms = HAL_GetTick();
    port->state.active = 1u;
    USARTTxExit(saved);
    HAL_StatusTypeDef result = USARTTxConfigured(instance->usart_handle, entry->request.mode)
                                   ? USARTTxStart(instance->usart_handle, entry->data,
                                                  (uint16_t)entry->request.length, entry->request.mode)
                                   : HAL_ERROR;
    saved = USARTTxEnter();
    if (result == HAL_BUSY)
    {
        entry->phase = TX_QUEUED;
        entry->irq_done = entry->irq_error = 0u;
        if (entry->cancel)
            TxFinish(entry, USART_TX_CANCELLED, 1u);
        port->state.active = 0u;
    }
    else
    {
        entry->event.started = (uint8_t)(result == HAL_OK);
        if (result != HAL_OK || entry->irq_error || entry->cancel)
        {
            entry->event.result = entry->cancel ? USART_TX_CANCELLED : USART_TX_FAILED;
            if (USARTTxStopped(instance->usart_handle, entry->request.mode))
            {
                USARTTxClear(instance->usart_handle, entry->request.mode);
                TxFinish(entry, entry->event.result, 1u);
            }
            else
                entry->phase = TX_RECOVER;
        }
        else
            entry->phase = TX_ACTIVE;
    }
    USARTTxExit(saved);
}
void USARTProcessTx(void)
{
    if (!USARTTxTaskContext())
        return;
    uint32_t saved = USARTTxEnter();
    if (!service_ready || processing)
    {
        USARTTxExit(saved);
        return;
    }
    processing = 1u;
    USARTTxExit(saved);
    // 每轮只停止一条总线；轮转起点避免某一路反复占用恢复机会。
    uint8_t stopped_once = 0u;
    for (unsigned n = 0u; n < DEVICE_USART_CNT; ++n)
    {
        unsigned p = (recovery_next + n) % DEVICE_USART_CNT;
        USARTInstance *instance = USARTRegisteredAt(p);
        if (instance == NULL)
            continue;
        TxPort *port = &ports[p];
        TxUpdate(port, instance);
        saved = USARTTxEnter();
        TxEntry *recover = NULL;
        for (unsigned s = 0u; s < USART_TX_DEPTH; ++s)
            if (port->entries[s].phase == TX_RECOVER && !stopped_once)
            {
                recover = &port->entries[s];
                recover->phase = TX_ABORTING;
                break;
            }
        USARTTxExit(saved);
        if (recover != NULL)
        {
            stopped_once = 1u;
            HAL_StatusTypeDef result = USARTTxAbort(instance->usart_handle, recover->request.mode);
            saved = USARTTxEnter();
            if (result == HAL_OK && USARTTxStopped(instance->usart_handle, recover->request.mode))
            {
                USARTTxClear(instance->usart_handle, recover->request.mode);
                TxFinish(recover, recover->event.result, 1u);
            }
            else
            {
                port->state.isolated = 1u;
                TxFinish(recover, USART_TX_ISOLATED, 0u);
                for (unsigned s = 0u; s < USART_TX_DEPTH; ++s)
                    if (port->entries[s].phase == TX_QUEUED)
                        TxFinish(&port->entries[s], USART_TX_CANCELLED, 1u);
            }
            USARTTxExit(saved);
        }
        TxNotify(port);
        TxStartNext(port, instance);
        saved = USARTTxEnter();
        port->state.active = 0u;
        for (unsigned s = 0u; s < USART_TX_DEPTH; ++s)
            if (port->entries[s].phase == TX_ACTIVE || port->entries[s].phase == TX_RECOVER ||
                port->entries[s].phase == TX_HELD)
                port->state.active = 1u;
        USARTTxExit(saved);
    }
    saved = USARTTxEnter();
    recovery_next = (uint8_t)((recovery_next + 1u) % DEVICE_USART_CNT);
    processing = 0u;
    USARTTxExit(saved);
}
