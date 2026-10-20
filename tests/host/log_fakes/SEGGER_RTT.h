/** @file SEGGER_RTT.h
 * @brief 日志主机测试的RTT传输声明；常量沿用生产RTT，不初始化探针或连接硬件。
 */
#ifndef MC02_LOG_FAKE_RTT_H
#define MC02_LOG_FAKE_RTT_H
#define RTT_CTRL_RESET "\x1B[0m"
#define RTT_CTRL_CLEAR "\x1B[2J"
#define RTT_CTRL_TEXT_BRIGHT_RED "\x1B[1;31m"
#define RTT_CTRL_TEXT_BRIGHT_GREEN "\x1B[1;32m"
#define RTT_CTRL_TEXT_BRIGHT_YELLOW "\x1B[1;33m"
void SEGGER_RTT_Init(void);
unsigned SEGGER_RTT_WriteString(unsigned index, const char *text);
#endif
