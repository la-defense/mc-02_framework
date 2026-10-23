/** @file usb_diagnostic.c
 * @brief 以有限完成通知丢失触发USB恢复，记录阶段、MCU时间和额度。
 * @note 只允许bench_safe/single禁热诊断构建；普通固件不得链接本文件。
 */
#include "usb_diagnostic.h"
#include "usb_transport_port.h"
#include <stdatomic.h>

#if !MC02_BENCH_USB_DIAGNOSTICS || !defined(MC02_PROFILE_BENCH_SAFE) || MC02_HEATER_ENABLED
#error "USB诊断只能用于显式启用的bench_safe禁热固件"
#endif
_Static_assert(sizeof(USBDiagnosticRecord_t) == 215u * sizeof(uint32_t), "USB诊断固定记录与工具不一致");

volatile USBDiagnosticRecord_t usb_diagnostic_record __attribute__((aligned(32)));
static uint32_t remaining, suppressed, phase, event_sequence;
static uint32_t previous_core_phase = UINT32_MAX, previous_generation, previous_attempts;
static uint32_t previous_suppressed, previous_phase = UINT32_MAX;

uint8_t USBTransportDiagnosticSuppressCompletion(void)
{
    uint32_t saved = USBPortEnter();
    uint8_t skip = (uint8_t)(remaining != 0u);
    if (skip) { --remaining; ++suppressed; }
    USBPortExit(saved);
    return skip;
}

void USBTransportDiagnosticPoll(void)
{
    uint32_t saved = USBPortEnter();
    USBTransportState_t state;
    if (USBTransportGetState(&state) != USB_OK) { USBPortExit(saved); return; }
    uint32_t now = USBPortNowMs();
    if (phase == USB_DIAG_WAIT_STABLE && state.phase == USB_PHASE_READY &&
        state.restart_attempts == 0u && state.stable_tracking &&
        (uint32_t)(now - state.stable_since_ms) >= 5000u)
    {
        // 四次通知丢失用于覆盖三次重启及第四次超限；实际端点传输仍正常完成。
        remaining = 4u;
        phase = USB_DIAG_INJECTING;
    }
    if (phase == USB_DIAG_INJECTING && suppressed == 4u && state.phase == USB_PHASE_EXHAUSTED)
        phase = USB_DIAG_WAIT_RECONFIGURE;
    if (phase == USB_DIAG_WAIT_RECONFIGURE && state.phase == USB_PHASE_READY)
        phase = USB_DIAG_WAIT_QUOTA;
    if (phase == USB_DIAG_WAIT_QUOTA && state.restart_attempts == 0u)
        phase = USB_DIAG_COMPLETE;

    uint8_t changed = (uint8_t)(previous_core_phase != (uint32_t)state.phase ||
        previous_generation != state.generation || previous_attempts != state.restart_attempts ||
        previous_suppressed != suppressed || previous_phase != phase);
    if (!changed && (uint32_t)(now - usb_diagnostic_record.published_ms) < 1000u)
    { USBPortExit(saved); return; }
    volatile USBDiagnosticRecord_t *out = &usb_diagnostic_record;
    uint32_t publication = (out->publication_generation & ~1u) + 2u;
    if (publication == 0u) publication = 2u;
    out->publication_generation = publication - 1u;
    atomic_thread_fence(memory_order_release);
    out->version = 1u; out->size_bytes = sizeof(*out); out->published_ms = now;
    out->phase = phase; out->remaining = remaining; out->suppressed = suppressed;
    out->core_phase = state.phase; out->connection_generation = state.generation;
    out->attempts = state.restart_attempts; out->restarts = state.restart_total;
    out->exhausted = state.restart_exhausted; out->timeouts = state.timeouts;
    out->completed = state.completed; out->rx_blocks = state.rx_blocks;
    out->rx_dropped = state.rx_dropped; out->tx_errors = state.tx_errors;
    out->last_tx_id = state.last_tx.id;
    out->last_tx_submitted_ms = state.last_tx.submitted_ms;
    out->last_tx_completed_ms = state.last_tx.completed_ms;
    out->last_tx_result = state.last_tx.result;
    if (changed)
    {
        event_sequence = USBTransportNextNonzero(event_sequence);
        unsigned slot = (unsigned)((event_sequence - 1u) % USB_DIAGNOSTIC_EVENTS);
        volatile USBDiagnosticEvent_t *event = &out->events[slot];
        event->sequence = event_sequence; event->time_ms = now; event->core_phase = state.phase;
        event->connection_generation = state.generation; event->attempts = state.restart_attempts;
        event->suppressed = suppressed; out->event_count = event_sequence;
    }
    previous_core_phase = state.phase; previous_generation = state.generation;
    previous_attempts = state.restart_attempts; previous_suppressed = suppressed; previous_phase = phase;
    atomic_thread_fence(memory_order_release);
    out->publication_generation = publication;
    atomic_thread_fence(memory_order_release);
    out->magic = USB_DIAGNOSTIC_MAGIC;
    USBPortExit(saved);
}
