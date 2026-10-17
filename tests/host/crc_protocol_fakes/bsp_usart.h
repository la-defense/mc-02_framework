/** @file bsp_usart.h
 * @brief CRC 协议测试的串口替身；只提供调用方实际使用的接收和发送接口。
 */
#ifndef MC02_CRC_TEST_USART_H
#define MC02_CRC_TEST_USART_H
#include <stdint.h>
#include "main.h"
typedef struct { int unused; } UART_HandleTypeDef;
typedef struct { uint8_t *recv_buff; uint16_t last_recv_size; } USARTInstance;
typedef struct {
    UART_HandleTypeDef *usart_handle;
    uint16_t recv_buff_size;
    void (*module_callback)(void);
} USART_Init_Config_s;
#define USART_TRANSFER_DMA 1
/** @brief 保存接收回调，供测试送入字节；不会访问硬件。
 * @param[in] config 含有效回调的配置，不能为 NULL。
 * @return 测试持有的固定串口对象，不需要释放。
 */
USARTInstance *USARTRegister(USART_Init_Config_s *config);
/** @brief 模拟重新接收，本替身不修改对象。
 * @param[in,out] instance 已注册串口对象。
 */
void USARTServiceInit(USARTInstance *instance);
/** @brief 将发送字节复制到测试记录中。
 * @param[in] instance 已注册串口对象。
 * @param[in] data 至少可读 length 字节，复制后可复用。
 * @param[in] length 字节数，不得超过测试记录容量512字节。
 * @param[in] mode 生产调用传入 USART_TRANSFER_DMA，替身不启动 DMA。
 */
void USARTSend(USARTInstance *instance, uint8_t *data, uint16_t length, int mode);
#endif
