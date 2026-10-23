/** @file usb_transport_port.h
 * @brief USB内部后端接入；H723和主机模型实现同一组操作。
 * @note 不供业务直接调用。等待／停止只能在USB推进任务中执行。
 */
#ifndef MC02_USB_TRANSPORT_PORT_H
#define MC02_USB_TRANSPORT_PORT_H
#include "usb_transport.h"

// 编号和连接代号保留0表示未登记；该运算也由边界测试直接核对。
static inline uint32_t USBTransportNextNonzero(uint32_t value)
{ ++value; return value ? value : 1u; }

// 仅工具按固定布局读取；业务通过USBTransportGetState复制，不取得内部数组地址。
#define USB_OBSERVATION_MAGIC 0x55425331u
#define USB_OBSERVATION_LAYOUT 0x00010001u
typedef struct {
    uint32_t magic, layout, size_bytes, publication_generation, published_ms;
    /* 工具读取固定32位字，避免依赖枚举和字节成员的ABI填充。 */
    uint32_t state_words[26];
} USBTransportObservation_t;

uint32_t USBPortNowMs(void);
uint32_t USBPortEnter(void);
void USBPortExit(uint32_t saved);
uint8_t USBPortInISR(void);
uint8_t USBPortAcquireTick(void);
void USBPortReleaseTick(void);
USBStatus_t USBPortStart(void);
USBStatus_t USBPortStop(void);
USBStatus_t USBPortSubmit(const uint8_t *data, size_t length);

/** @brief 后端复制实际接收块；长度保持size_t，检查后再缩窄。
 * @param[in] data,length 原端点数据及字节数，函数返回后可重新武装端点。
 */
void USBTransportOnReceive(const uint8_t *data, size_t length);
/** @brief 后端在真实发送完成ISR中登记，不调用业务回调。
 */
void USBTransportOnTxComplete(void);
/** @brief 后端登记连接变化；stopped只有确认旧端点不再借用缓冲时才能传1。
 * @param[in] reason 本次生命周期原因。
 * @param[in] stopped 旧发送缓冲可复用为1，否则为0；失败不能假装释放。
 */
void USBTransportOnLink(USBLinkReason_t reason, uint8_t stopped);
#endif
