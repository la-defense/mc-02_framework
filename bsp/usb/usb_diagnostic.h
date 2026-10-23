/** @file usb_diagnostic.h
 * @brief 仅裸板USB诊断固件允许的四次完成通知丢失，以及有限只读运行记录。
 * @note 普通固件不链接实现；不会修改控制数据、启动执行器或请求整板复位。
 */
#ifndef MC02_USB_DIAGNOSTIC_H
#define MC02_USB_DIAGNOSTIC_H
#include <stdint.h>
#define USB_DIAGNOSTIC_MAGIC 0x55444231u
#define USB_DIAGNOSTIC_EVENTS 32u
typedef enum {
    USB_DIAG_WAIT_STABLE, USB_DIAG_INJECTING, USB_DIAG_WAIT_RECONFIGURE,
    USB_DIAG_WAIT_QUOTA, USB_DIAG_COMPLETE
} USBDiagnosticPhase_t;
typedef struct {
    uint32_t sequence, time_ms, core_phase, connection_generation, attempts, suppressed;
} USBDiagnosticEvent_t;
typedef struct {
    uint32_t magic, version, size_bytes, publication_generation, published_ms;
    uint32_t phase, remaining, suppressed, event_count;
    uint32_t core_phase, connection_generation, attempts, restarts, exhausted;
    uint32_t timeouts, completed, rx_blocks, rx_dropped, tx_errors;
    uint32_t last_tx_id, last_tx_submitted_ms, last_tx_completed_ms, last_tx_result;
    USBDiagnosticEvent_t events[USB_DIAGNOSTIC_EVENTS];
} USBDiagnosticRecord_t;

#if MC02_BENCH_USB_DIAGNOSTICS
extern volatile USBDiagnosticRecord_t usb_diagnostic_record;
/** @brief defaultTask每轮调用；双向通信正常5秒且未消耗额度后，登记一次四项注入。
 * @note 后续只发布数据；四次用完后等待主机重配和60秒正常通信，不会再次注入。
 */
void USBTransportDiagnosticPoll(void);
/** @brief CDC真实完成ISR先询问；四次返回1表示仅丢弃本次完成通知。
 * @return 1跳过向核心通知，0正常通知；不改ST端点状态或发送数组。
 */
uint8_t USBTransportDiagnosticSuppressCompletion(void);
#endif
#endif
