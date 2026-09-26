/**
 * @file bsp_usart.c
 * @author neozng
 * @brief  串口bsp层的实现
 * @version beta
 * @date 2022-11-01
 *
 * @copyright Copyright (c) 2022
 *
 */
#include "bsp_usart.h"
#include "bsp_log.h"
#include "bsp_dwt.h"
#include "stdlib.h"
#include "memory.h"

/* usart service instance, modules' info would be recoreded here using USARTRegister() */
/* usart服务实例,所有注册了usart的模块信息会被保存在这里 */
static uint8_t idx;
static USARTInstance *usart_instance[DEVICE_USART_CNT] = {NULL};

/* ============================================================================
   接收队列 + 错误恢复(BSPUART-02, 2026-09-26)
   ----------------------------------------------------------------------------
   原来 HAL_UARTEx_RxEventCallback() 在中断里直接调模块回调做协议解析(裁判要
   在 255 字节里滑动找帧 + 逐帧 CRC 校验), HAL_UART_ErrorCallback() 还在中断里
   调**阻塞版** HAL_UART_AbortReceive() 并写 RTT 日志 —— 都是不该在中断里做的事。
   现在: 中断只把数据拷进队列 + 重新武装 DMA; 解析、错误恢复、日志全搬到任务
   (USARTProcessRx, 由 1kHz 的 MotorControlTask 调用)。
   为什么要"拷进队列"而不是把缓冲区指针交给任务: DMA 一直在往同一个 recv_buff
   里写, 任务解析时可能被新数据覆盖 —— 必须拷出来。
   并发模型: 单生产者(该串口的 RxEvent 中断) / 单消费者(USARTProcessRx)。
   ========================================================================== */
#define USART_RXQ_DEPTH 4u    /* 每实例队列深度(裁判几十 Hz、视觉 200Hz, 1kHz 消费足够) */
#define USART_RXQ_HDR 4u      /* 每项前 4 字节: 2 字节长度 + 2 字节填充(保证 data 四字节对齐) */
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

/* 队列内存池: 用静态内存而不是 malloc —— FreeRTOS 堆(configTOTAL_HEAP_SIZE=25600)
   还要放各任务的栈和所有模块实例, 再从里面抠 1~2KB 有耗尽的风险; 静态池用 .bss,
   不够时在注册阶段就能立刻发现(而不是运行期某个场景才失败)。
   各实例按自己的 recv_buff_size 从这里切分, 所以实际占用远小于池大小。 */
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

/**
 * @brief 启动串口服务,会在每个实例注册之后自动启用接收,当前实现为DMA接收,后续可能添加IT和BLOCKING接收
 *
 * @todo 串口服务会在每个实例注册之后自动启用接收,当前实现为DMA接收,后续可能添加IT和BLOCKING接收
 *       可能还要将此函数修改为extern,使得module可以控制串口的启停
 *
 * @param _instance instance owned by module,模块拥有的串口实例
 */
void USARTServiceInit(USARTInstance *_instance)
{
    HAL_UARTEx_ReceiveToIdle_DMA(_instance->usart_handle, _instance->recv_buff, _instance->recv_buff_size);
    // 关闭dma half transfer中断防止两次进入HAL_UARTEx_RxEventCallback()
    // 这是HAL库的一个设计失误,发生DMA传输完成/半完成以及串口IDLE中断都会触发HAL_UARTEx_RxEventCallback()
    // 我们只希望处理第一种和第三种情况,因此直接关闭DMA半传输中断
    __HAL_DMA_DISABLE_IT(_instance->usart_handle->hdmarx, DMA_IT_HT);
}

USARTInstance *USARTRegister(USART_Init_Config_s *init_config)
{
    if (idx >= DEVICE_USART_CNT) // 超过最大实例数
        while (1)
            LOGERROR("[bsp_usart] USART exceed max instance count!");

    for (uint8_t i = 0; i < idx; i++) // 检查是否已经注册过
        if (usart_instance[i]->usart_handle == init_config->usart_handle)
            while (1)
                LOGERROR("[bsp_usart] USART instance already registered!");

    USARTInstance *instance = (USARTInstance *)malloc(sizeof(USARTInstance));
    memset(instance, 0, sizeof(USARTInstance));

    instance->usart_handle = init_config->usart_handle;
    instance->recv_buff_size = init_config->recv_buff_size;
    instance->module_callback = init_config->module_callback;

    /* 接收队列内存: 从静态池里按本实例的 recv_buff_size 切一块
       (裁判 255 / 遥控 18 / 视觉 40, 比统一按 USART_RXBUFF_LIMIT 省很多)。
       池子不够说明配置有问题, 启动阶段就报错暴露出来。 */
    {
        uint32_t need = USART_RXQ_DEPTH * USARTQueueItemSize(instance);
        if ((uint32_t)usart_rxq_used + need > USART_RXQ_POOL_SIZE)
            while (1)
                LOGERROR("[bsp_usart] 接收队列内存池不足: 需要 %u, 池大小 %u",
                         (unsigned)need, (unsigned)USART_RXQ_POOL_SIZE);
        usart_q[idx].data = &usart_rxq_pool[usart_rxq_used];
        usart_rxq_used = (uint16_t)(usart_rxq_used + need);
        memset(usart_q[idx].data, 0, need);
    }

    usart_instance[idx++] = instance;
    USARTServiceInit(instance);
    return instance;
}

/* @todo 当前仅进行了形式上的封装,后续要进一步考虑是否将module的行为与bsp完全分离 */
void USARTSend(USARTInstance *_instance, uint8_t *send_buf, uint16_t send_size, USART_TRANSFER_MODE mode)
{
    switch (mode)
    {
    case USART_TRANSFER_BLOCKING:
        HAL_UART_Transmit(_instance->usart_handle, send_buf, send_size, 100);
        break;
    case USART_TRANSFER_IT:
        HAL_UART_Transmit_IT(_instance->usart_handle, send_buf, send_size);
        break;
    case USART_TRANSFER_DMA:
        HAL_UART_Transmit_DMA(_instance->usart_handle, send_buf, send_size);
        break;
    default:
        while (1)
            ; // illegal mode! check your code context! 检查定义instance的代码上下文,可能出现指针越界
        break;
    }
}

/* 串口发送时,gstate会被设为BUSY_TX */
uint8_t USARTIsReady(USARTInstance *_instance)
{
    /* HAL 状态值 READY=0x20 / BUSY_TX=0x21 共享位，位与判断同样恒非 0，需用相等比较 */
    return (_instance->usart_handle->gState == HAL_UART_STATE_READY) ? 1 : 0;
}

/**
 * @brief 把本次收到的数据压进队列
 * @note  只能在对应串口的 RxEvent 中断里调用(单生产者的前提)。
 *        数据先写完、__DMB 之后再发布 head, 保证消费者看到 head 时数据已就绪。
 */
static void USARTQueuePush(uint8_t slot, USARTInstance *ins, uint16_t len)
{
    if (usart_q[slot].data == NULL)
        return;
    if (len > ins->recv_buff_size)
        len = ins->recv_buff_size;

    uint16_t head = usart_q[slot].head;
    uint16_t next = (uint16_t)((head + 1u) % USART_RXQ_DEPTH);
    if (next == usart_q[slot].tail)
    {
        usart_q[slot].drop++; /* 队列满: 丢新帧 + 计数 */
        return;
    }

    uint8_t *item = USARTQueueItem(ins, slot, head);
    *(uint16_t *)item = len;                                  /* 前 2 字节: 长度 */
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
 *        会被多任务竞争。放在"用数据之前"调用, 解析延迟就是任务周期(≤1ms)。
 */
void USARTProcessRx(void)
{
    /* 双消费者保护(与 CANProcessRx 同一套做法): 原子的 test-and-set */
    static volatile uint8_t processing = 0;
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    if (processing)
    {
        __set_PRIMASK(primask);
        return;
    }
    processing = 1u;
    __set_PRIMASK(primask);

    uint32_t probe = DWT_ProbeStart();
    uint32_t drop_sum = 0u, err_sum = 0u, recover_sum = 0u;

    for (uint8_t slot = 0; slot < idx; ++slot)
    {
        USARTInstance *ins = usart_instance[slot];
        if (ins == NULL || usart_q[slot].data == NULL)
            continue;

        /* ① 中断报告过错误 → 在这里恢复(阻塞 Abort 在任务里是安全的) */
        if (usart_q[slot].need_recover)
        {
            usart_q[slot].need_recover = 0u;
            USARTTryRecover(slot, ins);
        }

        /* ② 逐帧派发(按到达顺序) */
        while (usart_q[slot].tail != usart_q[slot].head)
        {
            uint8_t *item = USARTQueueItem(ins, slot, usart_q[slot].tail);
            uint16_t len = *(uint16_t *)item;
            if (len > ins->recv_buff_size)
                len = ins->recv_buff_size;

            if (len > 0u)
            {
                memcpy(ins->recv_buff, item + USART_RXQ_HDR, len);
                ins->last_recv_size = len;
                if (ins->module_callback != NULL)
                    ins->module_callback();
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

/**
 * @brief 每次dma/idle中断发生时，都会调用此函数.对于每个uart实例会调用对应的回调进行进一步的处理
 *        例如:视觉协议解析/遥控器解析/裁判系统解析
 *
 * @note  通过__HAL_DMA_DISABLE_IT(huart->hdmarx,DMA_IT_HT)关闭dma half transfer中断防止两次进入HAL_UARTEx_RxEventCallback()
 *        这是HAL库的一个设计失误,发生DMA传输完成/半完成以及串口IDLE中断都会触发HAL_UARTEx_RxEventCallback()
 *        我们只希望处理，因此直接关闭DMA半传输中断第一种和第三种情况
 *
 * @param huart 发生中断的串口
 * @param Size 此次接收到的实际字节数,会记录到 instance->last_recv_size 供模块使用
 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    uint32_t probe = DWT_ProbeStart();

    for (uint8_t i = 0; i < idx; ++i)
    { // find the instance which is being handled
        if (huart == usart_instance[i]->usart_handle)
        { // call the callback function if it is not NULL
            /* 只入队, 不在中断里解析(BSPUART-02): 协议解析(裁判要在 255 字节里
               滑动找帧 + 逐帧 CRC 校验)搬到任务上下文, 由 USARTProcessRx() 逐帧执行。
               必须"拷贝"而不是传指针 —— DMA 会继续往 recv_buff 里写。 */
            USARTQueuePush(i, usart_instance[i], Size);

            /* 立刻重新武装接收(不然只收得到这一包) */
            HAL_UARTEx_ReceiveToIdle_DMA(usart_instance[i]->usart_handle, usart_instance[i]->recv_buff, usart_instance[i]->recv_buff_size);
            __HAL_DMA_DISABLE_IT(usart_instance[i]->usart_handle->hdmarx, DMA_IT_HT);
            break; // break the loop
        }
    }

    DWT_ProbeDone(&usart_prof_rx_isr, probe);
}

/**
 * @brief 当串口发送/接收出现错误时,会调用此函数,此时这个函数要做的就是重新启动接收
 *
 * @note  最常见的错误:奇偶校验/溢出/帧错误
 *
 * @param huart 发生错误的串口
 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    for (uint8_t i = 0; i < idx; ++i)
    {
        if (huart == usart_instance[i]->usart_handle)
        {
            /* 中断里只做两件事: 累加计数 + 置"需要恢复"标志。
               真正的 Abort/重启交给 USARTProcessRx() 在任务上下文做 ——
               HAL_UART_AbortReceive() 是阻塞版, 会轮询等 UART/DMA 回到 READY,
               在中断里可能一直等到超时; 写 RTT 日志同样是长耗时操作。 */
            usart_q[i].err_count++;
            usart_q[i].need_recover = 1u;
            return;
        }
    }
}
