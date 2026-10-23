/** @file usb_transport.c
 * @brief 固定USB接收队列、唯一在途发送和最多三次重建；后端负责实际停止确认。
 * @note USBTransportProcess由唯一普通任务调用，协议解析另由唯一接收任务执行。
 * @copyright Copyright (c) 2023; MC-02 contributors改造原USB接收队列。
 */
#include "usb_transport_port.h"
#include <limits.h>
#include <string.h>
#include <stdatomic.h>

#define USB_DETACH_MS 250u
#define USB_RETRY_MS 1000u
#define USB_ENUMERATION_MS 5000u

typedef struct {
    uint8_t bytes[USB_MAX_BYTES];
    size_t length;
    uint32_t received_ms, sequence, generation;
} RxEntry;

static struct {
    USBTransportConfig_t config;
    USBTransportState_t state;
    RxEntry rx[USB_RX_CAPACITY];
    uint8_t tx[USB_MAX_BYTES] __attribute__((aligned(32)));
    uint8_t head, tail, rx_processing, processing;
    uint8_t tx_done, tx_notify, tx_lease, service_lease, link_notify;
    uint8_t recover_requested, recovery_cycle, port_stopping;
    uint8_t have_command, have_completion;
    uint32_t rx_sequence, next_id, phase_since_ms;
    uint32_t command_ms, completion_ms, restart_ms;
} usb;
static volatile USBTransportObservation_t usb_observation __attribute__((aligned(32)));
_Static_assert(sizeof(USBTransportObservation_t) == 31u * sizeof(uint32_t), "USB只读发布长度改变");

// 阶段变化立即发布，普通计数最多每秒发布一次；读取者比较前后编号避免拼接两次发布。
static void PublishObservation(uint32_t now)
{
    if (usb_observation.magic == USB_OBSERVATION_MAGIC &&
        usb_observation.state_words[0] == (uint32_t)usb.state.phase &&
        usb_observation.state_words[8] == usb.state.generation &&
        usb_observation.state_words[9] == usb.state.restart_attempts &&
        (uint32_t)(now - usb_observation.published_ms) < 1000u)
        return;
    uint32_t generation = (usb_observation.publication_generation & ~1u) + 2u;
    if (generation == 0u) generation = 2u;
    usb_observation.publication_generation = generation - 1u;
    atomic_thread_fence(memory_order_release);
    usb_observation.layout = USB_OBSERVATION_LAYOUT;
    usb_observation.size_bytes = sizeof(usb_observation);
    usb_observation.published_ms = now;
    const USBTransportState_t *state = &usb.state;
    uint32_t *words = (uint32_t *)usb_observation.state_words;
    words[0] = (uint32_t)state->phase;
    words[1] = (uint32_t)state->last_link_reason;
    words[2] = (uint32_t)state->last_error;
    words[3] = state->last_tx.id;
    words[4] = state->last_tx.submitted_ms;
    words[5] = state->last_tx.completed_ms;
    words[6] = state->last_tx.generation;
    words[7] = (uint32_t)state->last_tx.result;
    words[8] = state->generation;
    words[9] = state->restart_attempts;
    words[10] = state->restart_total;
    words[11] = state->restart_exhausted;
    words[12] = state->accepted;
    words[13] = state->completed;
    words[14] = state->rejected;
    words[15] = state->cancelled;
    words[16] = state->timeouts;
    words[17] = state->tx_errors;
    words[18] = state->rx_blocks;
    words[19] = state->rx_dropped;
    words[20] = state->rx_oversized;
    words[21] = state->rx_expired;
    words[22] = state->rx_dispatched;
    words[23] = state->lifecycle_changes;
    words[24] = state->stable_since_ms;
    words[25] = (uint32_t)state->initialized | ((uint32_t)state->rx_used << 8) |
                ((uint32_t)state->tx_buffer_held << 16) | ((uint32_t)state->stable_tracking << 24);
    atomic_thread_fence(memory_order_release);
    usb_observation.publication_generation = generation;
    atomic_thread_fence(memory_order_release);
    usb_observation.magic = USB_OBSERVATION_MAGIC;
}

// 计数达到上限后保持，避免故障很多时回到0被误认为没有故障。
static void Count(uint32_t *value) { if (*value != UINT32_MAX) ++*value; }
static uint32_t Next(uint32_t value) { return USBTransportNextNonzero(value); }
static uint8_t Due(uint32_t now, uint32_t start, uint32_t duration)
{ return (uint8_t)((uint32_t)(now - start) >= duration); }

// 仅在已确认硬件不再借用发送数组后释放；调用者已进入短临界区。
static void ReleaseTx(void)
{
    usb.state.tx_buffer_held = 0u;
    if (usb.tx_lease) { USBPortReleaseTick(); usb.tx_lease = 0u; }
}
static void ReleaseService(void)
{
    if (usb.service_lease) { USBPortReleaseTick(); usb.service_lease = 0u; }
}
static void FinishTx(USBTxResult_t result, uint32_t now, uint8_t stopped)
{
    if (usb.state.tx_buffer_held && !usb.tx_done)
    {
        usb.tx_done = usb.tx_notify = 1u;
        usb.state.last_tx.result = result;
        usb.state.last_tx.completed_ms = now;
        if (result == USB_TX_COMPLETE) Count(&usb.state.completed);
        if (result == USB_TX_CANCELLED) Count(&usb.state.cancelled);
        if (result == USB_TX_TIMEOUT) Count(&usb.state.timeouts);
        if (result == USB_TX_ERROR) Count(&usb.state.tx_errors);
    }
    if (stopped) ReleaseTx();
}
static void Invalidate(USBLinkReason_t reason)
{
    usb.state.generation = Next(usb.state.generation);
    usb.state.last_link_reason = reason;
    Count(&usb.state.lifecycle_changes);
    if (UINT32_MAX - usb.state.rx_dropped < usb.state.rx_used)
        usb.state.rx_dropped = UINT32_MAX;
    else
        usb.state.rx_dropped += usb.state.rx_used;
    usb.head = usb.tail = usb.state.rx_used = 0u;
    usb.have_command = usb.have_completion = usb.state.stable_tracking = 0u;
    usb.link_notify = 1u;
}

USBStatus_t USBTransportInit(const USBTransportConfig_t *config)
{
    if (USBPortInISR()) return USB_CONTEXT_ERROR;
    uint32_t saved = USBPortEnter();
    if (usb.state.initialized) { USBPortExit(saved); return USB_ALREADY_INITIALIZED; }
    memset(&usb, 0, sizeof(usb));
    if (config != NULL) usb.config = *config;
    usb.state.initialized = 1u;
    usb.state.phase = USB_PHASE_STARTING;
    usb.state.generation = 1u;
    USBPortExit(saved);
    return USB_OK;
}

USBStatus_t USBTransportGetState(USBTransportState_t *output)
{
    if (USBPortInISR()) return USB_CONTEXT_ERROR;
    if (output == NULL) return USB_BAD_ARGUMENT;
    uint32_t saved = USBPortEnter();
    if (!usb.state.initialized) { USBPortExit(saved); return USB_NOT_INITIALIZED; }
    *output = usb.state;
    USBPortExit(saved);
    return USB_OK;
}

uint8_t USBTransportGenerationIsCurrent(uint32_t generation)
{
    uint32_t saved = USBPortEnter();
    uint8_t valid = (uint8_t)(usb.state.initialized && usb.state.phase == USB_PHASE_READY &&
        !usb.recover_requested && generation == usb.state.generation);
    USBPortExit(saved);
    return valid;
}

void USBTransportReportValidCommand(uint32_t generation, uint32_t received_ms)
{
    if (USBPortInISR()) return;
    uint32_t saved = USBPortEnter();
    if (USBTransportGenerationIsCurrent(generation) &&
        (uint32_t)(USBPortNowMs() - received_ms) <= USB_SOURCE_MAX_AGE_MS)
    {
        // 新命令到来前已有过空档，就不能把前后的正常时间拼成连续60秒。
        if (!usb.have_command || (uint32_t)(received_ms - usb.command_ms) > USB_SOURCE_MAX_AGE_MS)
            usb.state.stable_tracking = 0u;
        usb.command_ms = received_ms;
        usb.have_command = 1u;
    }
    USBPortExit(saved);
}

USBStatus_t USBTransportSubmit(const uint8_t *data, size_t length, uint32_t *id)
{
    if (USBPortInISR()) return USB_CONTEXT_ERROR;
    if (data == NULL || id == NULL || length == 0u || length > USB_MAX_BYTES)
        return USB_BAD_ARGUMENT;
    uint32_t saved = USBPortEnter();
    USBStatus_t result = USB_ACCEPTED;
    if (!usb.state.initialized) result = USB_NOT_INITIALIZED;
    else if (usb.state.phase == USB_PHASE_ISOLATED) result = USB_ISOLATED;
    else if (usb.state.phase != USB_PHASE_READY || usb.recover_requested) result = USB_NOT_CONFIGURED;
    else if (usb.state.tx_buffer_held || usb.tx_notify) result = USB_BUSY;
    else if (!USBPortAcquireTick()) result = USB_TIMEBASE_ERROR;
    if (result != USB_ACCEPTED)
    {
        if (usb.state.initialized) Count(&usb.state.rejected);
        USBPortExit(saved);
        return result;
    }
    usb.tx_lease = 1u;
    USBTxEvent_t previous = usb.state.last_tx;
    memcpy(usb.tx, data, length);
    usb.tx_done = 0u;
    usb.state.tx_buffer_held = 1u;
    usb.state.last_tx = (USBTxEvent_t){Next(usb.next_id), USBPortNowMs(), 0u,
                                    usb.state.generation, USB_TX_NONE};
    // 后端提交只设置端点并立即返回，不得等待；避免复制与提交之间被生命周期中断改写。
    result = USBPortSubmit(usb.tx, length);
    if (result == USB_ACCEPTED)
    {
        usb.next_id = usb.state.last_tx.id;
        *id = usb.next_id;
        Count(&usb.state.accepted);
    }
    else
    {
        if (result == USB_BUSY || result == USB_NOT_CONFIGURED || result == USB_BAD_ARGUMENT)
            ReleaseTx();
        else
        {
            // 后端报错不证明端点从未启动；保留数组直到停止确认。
            // 本次提交未被接受，因此不生成编号或完成通知。
            usb.tx_done = 1u;
        }
        usb.state.last_tx = previous;
        Count(&usb.state.rejected);
        usb.state.last_error = result;
        usb.state.stable_tracking = 0u;
        if (result != USB_BUSY && result != USB_NOT_CONFIGURED)
            usb.recover_requested = 1u;
    }
    USBPortExit(saved);
    return result;
}

void USBTransportOnReceive(const uint8_t *data, size_t length)
{
    uint32_t saved = USBPortEnter();
    if (!usb.state.initialized || length == 0u) { USBPortExit(saved); return; }
    ++usb.rx_sequence;
    if (data == NULL || length > USB_MAX_BYTES || usb.state.phase != USB_PHASE_READY ||
        usb.recover_requested || usb.state.rx_used == USB_RX_CAPACITY)
    {
        Count(&usb.state.rx_dropped);
        if (length > USB_MAX_BYTES) Count(&usb.state.rx_oversized);
        USBPortExit(saved);
        return;
    }
    RxEntry *entry = &usb.rx[usb.head];
    memcpy(entry->bytes, data, length);
    entry->length = length;
    entry->received_ms = USBPortNowMs();
    entry->generation = usb.state.generation;
    entry->sequence = usb.rx_sequence;
    usb.head = (uint8_t)((usb.head + 1u) % USB_RX_CAPACITY);
    ++usb.state.rx_used;
    Count(&usb.state.rx_blocks);
    USBPortExit(saved);
}

void USBTransportOnTxComplete(void)
{
    uint32_t saved = USBPortEnter();
    const uint32_t now = USBPortNowMs();
    if (usb.state.tx_buffer_held && !usb.tx_done && usb.state.phase == USB_PHASE_READY &&
        usb.state.last_tx.generation == usb.state.generation)
    {
        if (Due(now, usb.state.last_tx.submitted_ms, USB_TX_TIMEOUT_MS))
        {
            FinishTx(USB_TX_TIMEOUT, now, 1u);
            usb.recover_requested = 1u;
            usb.state.stable_tracking = 0u;
        }
        else
        {
            if (!usb.have_completion || Due(now, usb.completion_ms, USB_TX_TIMEOUT_MS))
                usb.state.stable_tracking = 0u;
            FinishTx(USB_TX_COMPLETE, now, 1u);
            usb.completion_ms = now;
            usb.have_completion = 1u;
        }
    }
    USBPortExit(saved);
}

void USBTransportOnLink(USBLinkReason_t reason, uint8_t stopped)
{
    uint32_t saved = USBPortEnter();
    if (!usb.state.initialized || usb.state.phase == USB_PHASE_ISOLATED)
    { USBPortExit(saved); return; }
    Invalidate(reason);
    FinishTx(reason == USB_LINK_TRANSFER_ERROR ? USB_TX_ERROR : USB_TX_CANCELLED,
             USBPortNowMs(), stopped);
    if (reason == USB_LINK_TRANSFER_ERROR) usb.state.last_error = USB_SEND_ERROR;
    if (!usb.port_stopping)
    {
        if ((reason == USB_LINK_CONFIGURED || reason == USB_LINK_RESUMED) && stopped)
        {
            usb.state.phase = USB_PHASE_READY;
            usb.recover_requested = usb.recovery_cycle = 0u;
            ReleaseService();
        }
        else
        {
            if (usb.state.phase != USB_PHASE_EXHAUSTED)
                usb.state.phase = USB_PHASE_WAIT_CONFIGURATION;
            if (reason == USB_LINK_TRANSFER_ERROR || usb.state.tx_buffer_held)
                usb.recover_requested = 1u;
        }
    }
    USBPortExit(saved);
}

USBStatus_t USBTransportProcessRx(void)
{
    if (USBPortInISR()) return USB_CONTEXT_ERROR;
    uint32_t saved = USBPortEnter();
    if (!usb.state.initialized) { USBPortExit(saved); return USB_NOT_INITIALIZED; }
    if (usb.rx_processing) { USBPortExit(saved); return USB_BUSY; }
    usb.rx_processing = 1u;
    USBPortExit(saved);
    // 使用本次调用独占的副本；中断清队列或下一块入队均不能改写正在解析的内容。
    for (unsigned i = 0u; i < USB_RX_BUDGET; ++i)
    {
        RxEntry entry;
        saved = USBPortEnter();
        if (usb.state.rx_used == 0u) { USBPortExit(saved); break; }
        entry = usb.rx[usb.tail];
        usb.tail = (uint8_t)((usb.tail + 1u) % USB_RX_CAPACITY);
        --usb.state.rx_used;
        uint8_t valid = USBTransportGenerationIsCurrent(entry.generation);
        if ((uint32_t)(USBPortNowMs() - entry.received_ms) > USB_SOURCE_MAX_AGE_MS)
        { Count(&usb.state.rx_expired); valid = 0u; }
        USBPortExit(saved);
        if (valid && usb.config.receive != NULL)
        {
            USBRxBlock_t block = {entry.bytes, entry.length, entry.received_ms,
                                  entry.sequence, entry.generation};
            usb.config.receive(usb.config.context, &block);
            saved = USBPortEnter();
            Count(&usb.state.rx_dispatched);
            USBPortExit(saved);
        }
    }
    saved = USBPortEnter();
    usb.rx_processing = 0u;
    USBPortExit(saved);
    return USB_OK;
}

// 普通数据停更不重启USB，仅停止累计正常时间；业务自己撤销过期目标。
static void CheckStable(uint32_t now)
{
    if (usb.state.phase != USB_PHASE_READY || usb.recover_requested || !usb.have_command ||
        !usb.have_completion || (uint32_t)(now - usb.command_ms) > USB_SOURCE_MAX_AGE_MS ||
        Due(now, usb.completion_ms, USB_TX_TIMEOUT_MS))
        usb.state.stable_tracking = 0u;
    else if (!usb.state.stable_tracking)
    {
        usb.state.stable_tracking = 1u;
        usb.state.stable_since_ms = now;
    }
    else if (Due(now, usb.state.stable_since_ms, USB_STABLE_MS))
        usb.state.restart_attempts = 0u;
}

USBStatus_t USBTransportProcess(void)
{
    if (USBPortInISR()) return USB_CONTEXT_ERROR;
    uint32_t saved = USBPortEnter();
    if (!usb.state.initialized) { USBPortExit(saved); return USB_NOT_INITIALIZED; }
    if (usb.processing) { USBPortExit(saved); return USB_BUSY; }
    usb.processing = 1u;
    uint32_t now = USBPortNowMs();
    if (usb.state.tx_buffer_held && !usb.tx_done &&
        Due(now, usb.state.last_tx.submitted_ms, USB_TX_TIMEOUT_MS))
    {
        FinishTx(USB_TX_TIMEOUT, now, 0u);
        usb.recover_requested = 1u;
    }
    if (usb.state.phase == USB_PHASE_WAIT_CONFIGURATION && usb.recovery_cycle &&
        Due(now, usb.phase_since_ms, USB_ENUMERATION_MS))
    {
        usb.state.phase = USB_PHASE_RETRY_WAIT;
        usb.phase_since_ms = now;
    }
    if (usb.state.phase == USB_PHASE_RETRY_WAIT && Due(now, usb.phase_since_ms, USB_RETRY_MS))
        usb.recover_requested = 1u;
    if (usb.recover_requested)
    {
        usb.recover_requested = 0u;
        if (usb.state.restart_attempts == USB_RESTART_LIMIT)
        {
            if (usb.state.phase != USB_PHASE_EXHAUSTED) Count(&usb.state.restart_exhausted);
            usb.state.phase = USB_PHASE_EXHAUSTED;
            ReleaseService();
            // 不覆盖仍可能在途的数组；主机复位并确认旧端点停止后才允许恢复。
        }
        else if (usb.state.restart_attempts != 0u && !Due(now, usb.restart_ms, USB_RETRY_MS))
        {
            usb.state.phase = USB_PHASE_RETRY_WAIT;
            usb.phase_since_ms = usb.restart_ms;
        }
        else
        {
            ++usb.state.restart_attempts;
            Count(&usb.state.restart_total);
            usb.restart_ms = now;
            usb.recovery_cycle = 1u;
            Invalidate(USB_LINK_RESTART);
            usb.state.phase = USB_PHASE_STOPPING;
        }
    }
    CheckStable(now);
    USBPhase_t phase = usb.state.phase;
    uint8_t start = (uint8_t)(phase == USB_PHASE_STARTING ||
        (phase == USB_PHASE_DISCONNECT_WAIT && Due(now, usb.phase_since_ms, USB_DETACH_MS)));
    uint8_t stop = (uint8_t)(phase == USB_PHASE_STOPPING);
    usb.port_stopping = stop;
    USBPortExit(saved);

    // 后端可能等待寄存器或HAL，不放在临界区；每轮只调用一次启动或停止。
    USBStatus_t operation = USB_OK;
    if (start || stop)
    {
        saved = USBPortEnter();
        if (!usb.service_lease)
        {
            usb.service_lease = USBPortAcquireTick();
            if (!usb.service_lease) operation = USB_TIMEBASE_ERROR;
        }
        USBPortExit(saved);
        if (operation == USB_OK) operation = stop ? USBPortStop() : USBPortStart();
        saved = USBPortEnter();
        usb.port_stopping = 0u;
        now = USBPortNowMs();
        usb.phase_since_ms = now;
        if (stop)
        {
            if (operation != USB_OK)
            {
                usb.state.phase = USB_PHASE_ISOLATED;
                usb.state.last_error = operation;
                // 停止失败时保留所有lease和发送存储，不能借恢复动作悄悄释放。
            }
            else
            {
                ReleaseTx();
                usb.state.phase = USB_PHASE_DISCONNECT_WAIT;
            }
        }
        else if (operation != USB_OK)
        {
            usb.state.last_error = operation;
            usb.state.phase = operation == USB_ISOLATED ? USB_PHASE_ISOLATED :
                (usb.recovery_cycle ? USB_PHASE_RETRY_WAIT : USB_PHASE_EXHAUSTED);
            if (!usb.recovery_cycle && operation != USB_ISOLATED) ReleaseService();
        }
        else
        {
            if (usb.state.phase != USB_PHASE_READY)
                usb.state.phase = USB_PHASE_WAIT_CONFIGURATION;
            if (!usb.recovery_cycle) ReleaseService();
        }
        USBPortExit(saved);
    }

    saved = USBPortEnter();
    uint8_t notify_tx = usb.tx_notify, notify_link = usb.link_notify;
    USBTxEvent_t event = usb.state.last_tx;
    uint32_t generation = usb.state.generation;
    USBLinkReason_t reason = usb.state.last_link_reason;
    usb.tx_notify = usb.link_notify = 0u;
    USBPortExit(saved);
    // 回调可以查询或尝试提交，但不能再次推进本模块，也不能等待。
    if (notify_link && usb.config.link_changed != NULL)
        usb.config.link_changed(usb.config.context, generation, reason);
    if (notify_tx && usb.config.complete != NULL)
        usb.config.complete(usb.config.context, &event);
    saved = USBPortEnter();
    PublishObservation(USBPortNowMs());
    usb.processing = 0u;
    USBPortExit(saved);
    return operation;
}
