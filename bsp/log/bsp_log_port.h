/** @file bsp_log_port.h
 * @brief 为日志队列提供时钟、临界区、RTT整条写入和静态任务入口。
 * @note 仅日志实现和主机适配使用；普通调用模块不直接依赖这些平台函数。
 */
#ifndef MC02_BSP_LOG_PORT_H
#define MC02_BSP_LOG_PORT_H

#include <stddef.h>
#include <stdint.h>

typedef void (*BSPLogWorkerEntry_t)(void const *argument);

/* 由日志核心交给静态任务端口；调度器运行后只有该任务能推进队列。 */
void BSPLogWorkerEntry(void const *argument);

uint32_t BSPLogPortNowMs(void);
uint8_t BSPLogPortInInterrupt(void);
uint8_t BSPLogPortSchedulerRunning(void);
uint8_t BSPLogPortIsWorker(void);
uint8_t BSPLogPortInit(void);
uint8_t BSPLogPortStartWorker(BSPLogWorkerEntry_t entry);
uint8_t BSPLogPortWriteWholeRecord(const char *data, size_t length);
void BSPLogPortEnterCritical(void);
void BSPLogPortExitCritical(void);
void BSPLogPortDelayMs(uint32_t milliseconds);

#endif /* MC02_BSP_LOG_PORT_H */
