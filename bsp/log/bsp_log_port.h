/** @file bsp_log_port.h
 * @brief 为日志核心提供时钟、临界区、RTT输出、USART10诊断帧和静态任务入口。
 * @note 诊断UART承载等级命令与按需健康快照，不镜像日志；普通模块不直接依赖平台函数。
 */
#ifndef MC02_BSP_LOG_PORT_H
#define MC02_BSP_LOG_PORT_H

#include <stddef.h>
#include <stdint.h>

typedef void (*BSPLogWorkerEntry_t)(void const *argument);
typedef void (*BSPLogDiagnosticRx_t)(const uint8_t *data, size_t length);

/** @brief 诊断串口发送队列的只读状态。 */
typedef struct
{
    uint32_t accepted;
    uint32_t completed;
    uint32_t failed;
    uint32_t rejected;
    uint16_t queue_depth;
    uint8_t available;
    uint8_t isolated;
} BSPLogPortDiagnosticState_t;

/* 由日志核心交给静态任务端口；调度器运行后只有该任务能推进队列。 */
void BSPLogWorkerEntry(void const *argument);

uint32_t BSPLogPortNowMs(void);
uint8_t BSPLogPortInInterrupt(void);
uint8_t BSPLogPortSchedulerRunning(void);
uint8_t BSPLogPortIsWorker(void);
uint8_t BSPLogPortInit(void);
uint8_t BSPLogPortStartWorker(BSPLogWorkerEntry_t entry);
uint8_t BSPLogPortWriteWholeRecord(const char *data, size_t length);
uint8_t BSPLogPortDiagnosticInit(BSPLogDiagnosticRx_t receive);
uint8_t BSPLogPortSubmitDiagnosticFrame(const uint8_t *data, size_t length);
uint8_t BSPLogPortGetDiagnosticState(BSPLogPortDiagnosticState_t *output);
void BSPLogPortEnterCritical(void);
void BSPLogPortExitCritical(void);
void BSPLogPortDelayMs(uint32_t milliseconds);

#endif /* MC02_BSP_LOG_PORT_H */
