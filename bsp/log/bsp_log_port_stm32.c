/** @file bsp_log_port_stm32.c
 * @brief 为 STM32H723 日志核心提供时钟、FreeRTOS 和整条非阻塞 RTT 写入。
 * @note 日志任务、栈和控制块使用静态 DTCM 存储；RTT 缓冲仅在关闭 DCache 时启用。
 */
#include "bsp_log_port.h"
#include "bsp_log.h"

#include "FreeRTOS.h"
#include "SEGGER_RTT.h"
#include "cmsis_os.h"
#include "task.h"
#include "main.h"
#include "usart.h"
#include "bsp_usart.h"
#include "usart_tx.h"

#include <limits.h>
#include <string.h>

#define BSP_LOG_WORKER_STACK_WORDS 1024u
#define BSP_LOG_RTT_UP_BUFFER 0u
/* CMSIS-RTOS v1 的 osPriorityBelowNormal映射到FreeRTOS优先级2。 */
#define BSP_LOG_WORKER_PRIORITY (tskIDLE_PRIORITY + (osPriorityBelowNormal - osPriorityIdle))

_Static_assert(BSP_LOG_WORKER_PRIORITY < configMAX_PRIORITIES,
               "log worker priority must fit the configured FreeRTOS range");
_Static_assert(BSP_LOG_DIAGNOSTIC_TX_FRAME_MAX <= USART_TX_MAX_BYTES,
               "health snapshot reply must fit the USART copied transmit queue");

#if defined(__GNUC__)
#define BSP_LOG_STATIC_DTCM __attribute__((section(".log_workspace"), aligned(32), used))
#else
#define BSP_LOG_STATIC_DTCM
#endif

static StackType_t log_worker_stack[BSP_LOG_WORKER_STACK_WORDS] BSP_LOG_STATIC_DTCM;
static StaticTask_t log_worker_tcb BSP_LOG_STATIC_DTCM;
static TaskHandle_t log_worker_handle;
static BSPLogWorkerEntry_t log_worker_entry;
static USARTInstance *diagnostic_uart_instance;
static BSPLogDiagnosticRx_t diagnostic_receive_callback;

static void BSPLogDiagnosticRx(void *context, const USART_RxBlock_t *block)
{
    (void)context;
    if (diagnostic_receive_callback != NULL && block != NULL)
        diagnostic_receive_callback(block->data, block->length);
}

static void BSPLogDiagnosticTxComplete(void *context, const USARTTxEvent_t *event)
{
    (void)context;
    (void)event;
    /* 发送结果由USART只读统计保留；回调不递归提交日志。 */
}

static void BSPLogWorkerTask(void *argument)
{
    if (log_worker_entry != NULL)
        log_worker_entry(argument);

    /* 日志入口本身常驻；若其意外返回，任务保持静止，避免错误重启循环。 */
    vTaskSuspend(NULL);
    for (;;)
        taskYIELD();
}

uint32_t BSPLogPortNowMs(void)
{
    return HAL_GetTick();
}

uint8_t BSPLogPortInInterrupt(void)
{
    return (__get_IPSR() != 0u) ? 1u : 0u;
}

uint8_t BSPLogPortSchedulerRunning(void)
{
    return (osKernelRunning() != 0u) ? 1u : 0u;
}

uint8_t BSPLogPortIsWorker(void)
{
    return (log_worker_handle != NULL && xTaskGetCurrentTaskHandle() == log_worker_handle) ? 1u : 0u;
}

uint8_t BSPLogPortInit(void)
{
    /* 链接段为NOLOAD，首次创建静态任务前先清理其栈与控制块。 */
    memset(log_worker_stack, 0, sizeof(log_worker_stack));
    memset(&log_worker_tcb, 0, sizeof(log_worker_tcb));

    /* DTCM 不参与 M7 DCache；若系统后来启用 DCache，本轮直接停用 RTT 后端。 */
    if ((SCB->CCR & SCB_CCR_DC_Msk) != 0u)
        return 0u;

    SEGGER_RTT_Init();
    return 1u;
}

uint8_t BSPLogPortStartWorker(BSPLogWorkerEntry_t entry)
{
    if (entry == NULL || log_worker_handle != NULL || BSPLogPortSchedulerRunning())
        return 0u;

    log_worker_entry = entry;
    log_worker_handle = xTaskCreateStatic(BSPLogWorkerTask,
                                          "logtask",
                                          BSP_LOG_WORKER_STACK_WORDS,
                                          NULL,
                                          BSP_LOG_WORKER_PRIORITY,
                                          log_worker_stack,
                                          &log_worker_tcb);
    if (log_worker_handle == NULL)
    {
        log_worker_entry = NULL;
        return 0u;
    }
    return 1u;
}

uint8_t BSPLogPortWriteWholeRecord(const char *data, size_t length)
{
    if (data == NULL || length == 0u || length > UINT_MAX)
        return 0u;

    SEGGER_RTT_BUFFER_UP *ring = &_SEGGER_RTT.aUp[BSP_LOG_RTT_UP_BUFFER];
    unsigned size = ring->SizeOfBuffer;
    unsigned write_offset = ring->WrOff;
    unsigned read_offset = ring->RdOff;
    if (ring->pBuffer == NULL || size < 2u || write_offset >= size || read_offset >= size)
        return 0u;

    size_t available = (read_offset > write_offset)
                           ? (size_t)(read_offset - write_offset - 1u)
                           : (size_t)(size - write_offset + read_offset - 1u);
    if (length > available)
        return 0u;

    size_t first = length;
    size_t until_end = (size_t)size - write_offset;
    if (first > until_end)
        first = until_end;
    memcpy(ring->pBuffer + write_offset, data, first);
    if (length > first)
        memcpy(ring->pBuffer, data + first, length - first);

    unsigned next = (unsigned)((write_offset + length) % size);
    __DMB();
    ring->WrOff = next;
    return 1u;
}

void BSPLogPortEnterCritical(void)
{
    taskENTER_CRITICAL();
}

void BSPLogPortExitCritical(void)
{
    taskEXIT_CRITICAL();
}

void BSPLogPortDelayMs(uint32_t milliseconds)
{
    osDelay(milliseconds);
}

uint8_t BSPLogPortDiagnosticInit(BSPLogDiagnosticRx_t receive)
{
    if (receive == NULL)
        return 0u;
    diagnostic_receive_callback = receive;
    if (diagnostic_uart_instance != NULL)
        return 1u;

    USART_Init_Config_s configuration;
    memset(&configuration, 0, sizeof(configuration));
    configuration.recv_buff_size = 64u;
    configuration.usart_handle = &huart10;
    configuration.module_callback = BSPLogDiagnosticRx;
    diagnostic_uart_instance = USARTRegister(&configuration);
    return diagnostic_uart_instance != NULL;
}

uint8_t BSPLogPortSubmitDiagnosticFrame(const uint8_t *data, size_t length)
{
    if (diagnostic_uart_instance == NULL || data == NULL || length == 0u ||
        length > USART_TX_MAX_BYTES)
        return 0u;

    USARTTxRequest_t request;
    memset(&request, 0, sizeof(request));
    request.data = data;
    request.length = length;
    request.mode = USART_TRANSFER_DMA;
    request.timeout_ms = 100u;
    request.complete = BSPLogDiagnosticTxComplete;
    uint32_t id = 0u;
    return USARTSubmitTx(diagnostic_uart_instance, &request, &id) == USART_SEND_ACCEPTED;
}

uint8_t BSPLogPortGetDiagnosticState(BSPLogPortDiagnosticState_t *output)
{
    if (output == NULL)
        return 0u;
    BSPLogPortDiagnosticState_t candidate;
    memset(&candidate, 0, sizeof(candidate));
    if (diagnostic_uart_instance == NULL)
    {
        *output = candidate;
        return 1u;
    }

    candidate.available = 1u;
    USARTTxState_t state;
    if (USARTGetTxState(diagnostic_uart_instance, &state) == USART_SEND_ACCEPTED)
    {
        candidate.accepted = state.accepted;
        candidate.completed = state.completed;
        candidate.failed = state.failed;
        candidate.rejected = state.rejected;
        candidate.queue_depth = state.used;
        candidate.isolated = state.isolated;
    }
    *output = candidate;
    return 1u;
}
