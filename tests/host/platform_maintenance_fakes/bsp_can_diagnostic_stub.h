/** @file bsp_can_diagnostic_stub.h
 * @brief 回环诊断适配测试的后端声明。
 */
#ifndef TEST_CAN_DIAGNOSTIC_STUB_H
#define TEST_CAN_DIAGNOSTIC_STUB_H
#include <stdint.h>
#define DEVICE_CAN_CNT 3u // MC-02 的回环适配检查三条 CAN 总线。
uint8_t CANRunLoopbackSelfTest(void);
#endif
