/**
 * @file bsp_usart.c
 * @author neozng
* @brief UART 注册与发送，中断复制入队，任务按接收顺序派发并处理错误恢复。
 * @version beta
 * @date 2022-11-01
 *
 * @copyright Copyright (c) 2022
 *
 */
#include "bsp_usart.h"
#include "usart_tx_private.h"
#include "message_freshness.h"
#include "bsp_log.h"
#include "bsp_dwt.h"
#include "stdlib.h"
#include "memory.h"

/* usart service instance, modules' info would be recoreded here using USARTRegister() */
/* usart服务实例,所有注册了usart的模块信息会被保存在这里 */
static uint8_t idx;
static USARTInstance *usart_instance[DEVICE_USART_CNT] = {NULL};

// 每路 UART 单独保存接收队列；中断复制数据后重新启动 DMA，任务再解析和恢复错误。
// 必须复制内容，DMA 会重复使用 recv_buff，直接保存其地址会在派发前被新数据覆盖。
#define USART_RXQ_DEPTH 4u    /* 每实例队列深度(裁判几十 Hz、视觉 200Hz, 1kHz 消费足够) */
#define USART_RXQ_HDR 16u     /* 长度、填充、原始tick、序号及重启代数，数据四字节对齐。 */
#define USART_RECOVER_RETRY 3u

typedef struct
{
    uint8_t *data;                 /* 队列内存: USART_RXQ_DEPTH 项, 每项 USART_RXQ_HDR + recv_buff_size */
    volatile uint16_t head;        /* 生产者(中断)写 */
    volatile uint16_t tail;        /* 消费者(任务)写 */
    volatile uint32_t drop;        /* 队列满丢弃帧数 */
    volatile uint16_t peak;        /* 历史最大占用(诊断队列深度是否够) */
    volatile uint32_t err_count;   /* 串口错误次数(ISR 只累加) */
    volatile uint8_t need_recover; /* 1 = 中断报告出错, 等任务来恢复 */
    volatile uint32_t recover_count;  /* 成功恢复次数 */
    volatile uint8_t recover_failed;  /* 1 = 连续重试仍失败 */
    LogRateLimit_t log_rl;         /* 错误日志限速(在任务里打印) */
} USARTQueue_t;

static USARTQueue_t usart_q[DEVICE_USART_CNT];

// 队列按各实例所需大小从静态池取空间，容量不足在注册时返回失败，不在收包中途申请内存。
#define USART_RXQ_POOL_SIZE 3072u
static uint8_t usart_rxq_pool[USART_RXQ_POOL_SIZE] __attribute__((aligned(4)));
static uint16_t usart_rxq_used = 0u;

/* 性能探针(2026-09-26): 中断侧入队耗时 / 任务侧派发耗时 */
volatile DWT_Probe_t usart_prof_rx_isr = {0};
volatile DWT_Probe_t usart_prof_dispatch = {0};
/* 汇总(供 OpenOCD / LCD 读取) */
volatile uint32_t usart_rxq_drop_total = 0;
volatile uint32_t usart_err_total = 0;
volatile uint32_t usart_recover_total = 0;
volatile uint32_t usart_rx_oversize_total = 0;

static uint32_t USARTQueueItemSize(USARTInstance *ins)
{
    /* 每项大小向上对齐到 4 字节: 否则第 2 项起地址就不是 4 的倍数,
       而我们要用 *(uint16_t*)item 存长度 —— Cortex-M7 上非对齐访问会触发
       UsageFault(见 docs/学习笔记/09)。 */
    return ((uint32_t)USART_RXQ_HDR + (uint32_t)ins->recv_buff_size + 3u) & ~3u;
}

static uint8_t *USARTQueueItem(USARTInstance *ins, uint8_t slot, uint16_t slot_idx)
{
    return usart_q[slot].data + (uint32_t)slot_idx * USARTQueueItemSize(ins);
}

// 具体参数及失败返回见头文件；启动 DMA 成功后关闭半传输通知。
static uint8_t USARTStartRx(USARTInstance *_instance)
{
    if (_instance == NULL || _instance->usart_handle == NULL || _instance->recv_buff_size == 0u ||
        _instance->usart_handle->hdmarx == NULL)
        return 0u;
    HAL_StatusTypeDef status = HAL_UARTEx_ReceiveToIdle_DMA(_instance->usart_handle,
                                                            _instance->recv_buff,
                                                            _instance->recv_buff_size);
    if (status != HAL_OK)
        return 0u;
    // 关闭dma half transfer中断防止两次进入HAL_UARTEx_RxEventCallback()
    // HAL 会通知半传输、传输完成及空闲；这里只保留完成和空闲通知，避免重复处理半包。
    // 我们只希望处理第一种和第三种情况,因此直接关闭DMA半传输中断
    __HAL_DMA_DISABLE_IT(_instance->usart_handle->hdmarx, DMA_IT_HT);
    return 1u;
}

uint8_t USARTServiceInit(USARTInstance *instance)
{
    if (instance == NULL) return 0u;
    // 显式重启打断协议残帧，即使HAL本次拒绝也不能继续拼接旧数据。
    uint32_t mask = __get_PRIMASK();
    __disable_irq();
    ++instance->rx_sequence;
    ++instance->rx_generation;
    for (uint8_t i = 0; i < idx; ++i)
        if (usart_instance[i] == instance)
        {
            usart_q[i].need_recover = 0u;
            usart_q[i].recover_failed = 0u;
        }
    __set_PRIMASK(mask);
    uint8_t result = USARTStartRx(instance);
    if (!result)
        for (uint8_t i = 0; i < idx; ++i)
            if (usart_instance[i] == instance) usart_q[i].need_recover = 1u;
    return result;
}

USARTInstance *USARTRegister(USART_Init_Config_s *init_config)
{
    if (init_config == NULL || init_config->usart_handle == NULL ||
        init_config->module_callback == NULL || init_config->recv_buff_size == 0u ||
        idx >= DEVICE_USART_CNT)
        return NULL;

    for (uint8_t i = 0; i < idx; i++)
        if (usart_instance[i]->usart_handle == init_config->usart_handle)
            return NULL;

    USARTInstance *instance = (USARTInstance *)malloc(sizeof(USARTInstance));
    if (instance == NULL)
        return NULL;
    memset(instance, 0, sizeof(USARTInstance));

    instance->usart_handle = init_config->usart_handle;
    instance->recv_buff_size = init_config->recv_buff_size;
    instance->module_callback = init_config->module_callback;
    instance->context = init_config->context;
    instance->tx_guard = init_config->tx_guard;
    instance->tx_gate = init_config->tx_gate;
    instance->tx_observer = init_config->tx_observer;

    /* 接收队列内存: 从静态池里按本实例的 recv_buff_size 切一块
       (裁判255、遥控18、备用UART视觉128字节，按实际容量使用池空间)。
       池子不够说明配置有问题, 启动阶段就报错暴露出来。 */
    uint32_t need = USART_RXQ_DEPTH * USARTQueueItemSize(instance);
    if ((uint32_t)usart_rxq_used + need > USART_RXQ_POOL_SIZE)
    {
        free(instance);
        return NULL;
    }
    // 先登记存储，再启动接收；启动阶段串行注册，避免首个中断找不到实例。
    usart_q[idx].data = &usart_rxq_pool[usart_rxq_used];
    memset(usart_q[idx].data, 0, need);
    usart_rxq_used = (uint16_t)(usart_rxq_used + need);
    usart_instance[idx++] = instance;
    if (!USARTStartRx(instance))
    {
        instance->module_callback = NULL;
        if (HAL_UART_AbortReceive(instance->usart_handle) == HAL_OK)
        {
            uint32_t mask = __get_PRIMASK();
            __disable_irq();
            --idx;
            usart_instance[idx] = NULL;
            memset(&usart_q[idx], 0, sizeof(usart_q[idx]));
            usart_rxq_used = (uint16_t)(usart_rxq_used - need);
            __set_PRIMASK(mask);
            free(instance);
        }
        // 中止失败时保留对象及队列，硬件仍可能借用其存储；不再调用业务回调。
        return NULL;
    }
    return instance;
}

// 只比较已登记地址，未注册指针不被解引用。
int USARTRegisteredIndex(const USARTInstance *instance)
{
    for (uint8_t i = 0u; i < idx; ++i)
        if (usart_instance[i] == instance && instance->module_callback != NULL) return i;
    return -1;
}
USARTInstance *USARTRegisteredAt(unsigned index)
{ return index < idx ? usart_instance[index] : NULL; }

/**
 * @brief 把本次收到的数据压进队列
 * @note  只能在对应串口的 RxEvent 中断里调用(单生产者的前提)。
 *        数据先写完、__DMB 之后再发布 head, 保证消费者看到 head 时数据已就绪。
 */
static void USARTQueuePush(uint8_t slot, USARTInstance *ins, uint16_t len, uint32_t received_ms)
{
    if (usart_q[slot].data == NULL)
        return;
    uint32_t sequence = ++ins->rx_sequence;
    if (len == 0u || len > ins->recv_buff_size)
    {
        usart_q[slot].drop++;
        if (len > ins->recv_buff_size) usart_rx_oversize_total++;
        return;
    }

    uint16_t head = usart_q[slot].head;
    uint16_t next = (uint16_t)((head + 1u) % USART_RXQ_DEPTH);
    if (next == usart_q[slot].tail)
    {
        usart_q[slot].drop++; /* 队列满: 丢新帧 + 计数 */
        return;
    }

    uint8_t *item = USARTQueueItem(ins, slot, head);
    *(uint16_t *)item = len;
    *(uint32_t *)(item + 4u) = received_ms; // 保存RxEvent入口记录的毫秒值，派发时不改写。
    *(uint32_t *)(item + 8u) = sequence;
    *(uint32_t *)(item + 12u) = ins->rx_generation;
    memcpy(item + USART_RXQ_HDR, ins->recv_buff, len);        /* 后面: 数据 */

    __DMB();
    usart_q[slot].head = next;

    uint16_t used = (uint16_t)((uint16_t)(head - usart_q[slot].tail + USART_RXQ_DEPTH) %
                               USART_RXQ_DEPTH) + 1u;
    if (used > usart_q[slot].peak)
        usart_q[slot].peak = used;
}

/**
 * @brief 串口出错后的恢复: Abort 掉旧接收再重新武装 DMA, 最多重试 N 次
 * @note  必须在任务上下文调用 —— HAL_UART_AbortReceive() 是阻塞版, 会轮询等待
 *        UART/DMA 回到 READY, 放在中断里可能卡到超时。
 */
static void USARTTryRecover(uint8_t slot, USARTInstance *ins)
{
    for (uint8_t attempt = 0; attempt < USART_RECOVER_RETRY; ++attempt)
    {
        if (HAL_UART_AbortReceive(ins->usart_handle) != HAL_OK)
            continue;
        if (HAL_UARTEx_ReceiveToIdle_DMA(ins->usart_handle, ins->recv_buff,
                                         ins->recv_buff_size) != HAL_OK)
            continue;
        __HAL_DMA_DISABLE_IT(ins->usart_handle->hdmarx, DMA_IT_HT);
        usart_q[slot].recover_count++;
        usart_q[slot].recover_failed = 0u;
        return;
    }
    usart_q[slot].recover_failed = 1u; /* 连续重试仍失败, 置标志供观测 */
}

/**
 * @brief 逐帧派发所有串口收到的数据 + 处理错误恢复(在任务上下文调用)
 * @note  全工程只允许一个调用者(当前是 1kHz 的 MotorControlTask), 否则队列 tail
 *        会被多任务竞争。放在使用数据前调用，每轮处理入口时已有块，不承诺物理耗时。
 */
void USARTProcessRx(void)
{
    // 暂时关普通中断检查并设置 processing；已有派发正在执行时，本轮直接返回。
    static volatile uint8_t processing = 0;
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    if (processing)
    {
        __set_PRIMASK(primask);
        return;
    }
    processing = 1u;
    uint16_t ends[DEVICE_USART_CNT];
    for (uint8_t i = 0; i < idx; ++i) ends[i] = usart_q[i].head;
    __DMB();
    __set_PRIMASK(primask);

    uint32_t probe = DWT_ProbeStart();
    uint32_t drop_sum = 0u, err_sum = 0u, recover_sum = 0u;

    for (uint8_t slot = 0; slot < idx; ++slot)
    {
        USARTInstance *ins = usart_instance[slot];
        if (ins == NULL || usart_q[slot].data == NULL)
            continue;

        /* ① 中断报告过错误 → 在这里恢复(阻塞 Abort 在任务里是安全的) */
        uint32_t mask = __get_PRIMASK();
        __disable_irq();
        uint8_t recover = usart_q[slot].need_recover;
        usart_q[slot].need_recover = 0u;
        __set_PRIMASK(mask);
        if (recover)
        {
            USARTTryRecover(slot, ins);
        }

        // 只派发入口时已有的块，每路最多三块；处理中到达的数据留到下一轮。
        while (usart_q[slot].tail != ends[slot])
        {
            uint8_t *item = USARTQueueItem(ins, slot, usart_q[slot].tail);
            __DMB();
            uint16_t len = *(uint16_t *)item;
            uint32_t received = *(uint32_t *)(item + 4u);
            if (*(uint32_t *)(item + 12u) != ins->rx_generation || len == 0u || len > ins->recv_buff_size ||
                (uint32_t)(HAL_GetTick() - received) > MESSAGE_DEFAULT_MAX_AGE_MS)
                usart_q[slot].drop++;
            else if (ins->module_callback != NULL)
            {
                const USART_RxBlock_t block = {
                    .data = item + USART_RXQ_HDR, .length = len,
                    .received_ms = received, .sequence = *(uint32_t *)(item + 8u)
                };
                ins->module_callback(ins->context, &block);
            }
            __DMB();
            usart_q[slot].tail = (uint16_t)((usart_q[slot].tail + 1u) % USART_RXQ_DEPTH);
        }

        /* ③ 错误日志: 中断里只计数, 这里限速打印 */
        if (usart_q[slot].err_count != 0u &&
            LogRateLimitAllow(&usart_q[slot].log_rl, 1000u))
        {
            LOGWARNING("[bsp_usart] 串口错误 slot=%u err=%lu recover=%lu fail=%u",
                       (unsigned)slot, (unsigned long)usart_q[slot].err_count,
                       (unsigned long)usart_q[slot].recover_count,
                       (unsigned)usart_q[slot].recover_failed);
        }

        drop_sum += usart_q[slot].drop;
        err_sum += usart_q[slot].err_count;
        recover_sum += usart_q[slot].recover_count;
    }

    usart_rxq_drop_total = drop_sum;
    usart_err_total = err_sum;
    usart_recover_total = recover_sum;

    DWT_ProbeDone(&usart_prof_dispatch, probe);
    processing = 0u;
}

/** @brief DMA 完成或 UART 空闲时复制本次数据入队，重新准备下一轮接收，不在中断里解析。
* @param[in] huart 来源句柄。
* @param[in] Size 本次实际字节数，超过注册容量时丢弃。
 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    uint32_t received_ms = HAL_GetTick();
    uint32_t probe = DWT_ProbeStart();

    for (uint8_t i = 0; i < idx; ++i)
    { // find the instance which is being handled
        if (huart == usart_instance[i]->usart_handle)
        { // call the callback function if it is not NULL
            if (usart_q[i].need_recover || usart_q[i].recover_failed)
            {
                // 错误恢复完成前不接受可能属于旧DMA事务的回调。
                ++usart_instance[i]->rx_sequence;
                ++usart_q[i].drop;
                break;
            }
            /* 只入队, 不在中断里解析(BSPUART-02): 协议解析(裁判要在 255 字节里
               滑动找帧 + 逐帧 CRC 校验)搬到任务上下文, 由 USARTProcessRx() 逐帧执行。
               必须"拷贝"而不是传指针 —— DMA 会继续往 recv_buff 里写。 */
            USARTQueuePush(i, usart_instance[i], Size, received_ms);

            /* 立刻重新武装接收(不然只收得到这一包) */
            if (!USARTStartRx(usart_instance[i]))
            {
                ++usart_instance[i]->rx_sequence;
                ++usart_instance[i]->rx_generation;
                ++usart_q[i].err_count;
                usart_q[i].need_recover = 1u;
            }
            break; // break the loop
        }
    }

    DWT_ProbeDone(&usart_prof_rx_isr, probe);
}

/** @brief UART 错误中断只记录次数并请求恢复，实际中止和重新启动接收由任务执行。
* @param[in] huart 发生错误的 UART 句柄。
 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    USARTTxErrorISR(huart);
    for (uint8_t i = 0; i < idx; ++i)
    {
        if (huart == usart_instance[i]->usart_handle)
        {
            /* 中断里只做两件事: 累加计数 + 置"需要恢复"标志。
               真正的 Abort/重启交给 USARTProcessRx() 在任务上下文做 ——
               HAL_UART_AbortReceive() 是阻塞版, 会轮询等 UART/DMA 回到 READY,
               在中断里可能一直等到超时; 写 RTT 日志同样是长耗时操作。 */
            ++usart_instance[i]->rx_sequence;
            ++usart_instance[i]->rx_generation;
            usart_q[i].err_count++;
            usart_q[i].need_recover = 1u;
            return;
        }
    }
}
