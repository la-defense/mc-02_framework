/** @file SEGGER_RTT.h
 * @brief 日志端口编译所需的最小 RTT 缓冲结构。
 */
#ifndef MC02_LOG_PORT_FAKE_RTT_H
#define MC02_LOG_PORT_FAKE_RTT_H
typedef struct { char *pBuffer; unsigned SizeOfBuffer, WrOff, RdOff; } SEGGER_RTT_BUFFER_UP;
typedef struct { SEGGER_RTT_BUFFER_UP aUp[1]; } SEGGER_RTT_CB;
extern SEGGER_RTT_CB _SEGGER_RTT;
void SEGGER_RTT_Init(void);
#endif
