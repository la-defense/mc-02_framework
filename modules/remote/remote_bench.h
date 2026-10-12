#ifndef REMOTE_BENCH_H
#define REMOTE_BENCH_H
#include "remote_control.h"
uint8_t RemoteBenchInit(UART_HandleTypeDef *uart);
void RemoteBenchTask(void);
#endif
