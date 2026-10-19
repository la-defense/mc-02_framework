/** @file usart_rx.h
 * @brief UART任务派发的只读接收块；描述符和数据均只借用到回调返回。
 * @note 不包含HAL类型。协议须复制跨回调残帧，不能保存data或描述符地址。
 */
#ifndef MC02_USART_RX_H
#define MC02_USART_RX_H
#include <stddef.h>
#include <stdint.h>
/** @brief 一次RxEvent通知产生的字节块，不等同于一份完整协议帧。 */
typedef struct {
    const uint8_t *data;    /**< 非空，只读length字节，回调期间不被DMA或队列生产者覆盖。 */
    size_t length;         /**< 实际字节数，1～注册容量，当前最大255。 */
    uint32_t received_ms;   /**< 原RxEvent通知的HAL毫秒值；0合法，不是逐字节物理到达时间。 */
    uint32_t sequence;      /**< 每次通知、错误及显式重启推进，丢块也消耗；允许回绕。不是协议序号。 */
} USART_RxBlock_t;
/** @brief 单任务串行接收回调，禁止递归派发或保存block/data供返回后使用。
 * @param[in,out] context 注册时借用的调用者上下文，可为NULL；非空时须长期有效。
 * @param[in] block 非空只读描述符；校验、分帧和来源有效性由协议处理。
 * @note 不连续的sequence表示丢弃、错误或重启，协议须清残帧；正常DMA重装不打断连续性。
 */
typedef void (*usart_module_callback)(void *context, const USART_RxBlock_t *block);
#endif
