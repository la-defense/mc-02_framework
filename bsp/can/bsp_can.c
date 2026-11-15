/** @file bsp_can.c
* @brief 按实例注册收发 ID；中断查找接收者并复制入队，任务逐帧解析和检查总线状态。
 */
#include "bsp_can.h"
#include "message_freshness.h"
#include "main.h"
#include "memory.h"
#include "stdlib.h"
#include "bsp_dwt.h"
#include "bsp_log.h"
#include "cmsis_os.h"
#include "FreeRTOS.h"
#include "task.h"
#include <limits.h>
#include <stddef.h>

// 注册表用于 ID 查找、过滤规则重装及注销；中断不直接运行模块解析。
static CANInstance *can_instance[CAN_MX_REGISTER_CNT] = {NULL};
static uint8_t idx;
static volatile uint8_t can_service_started; // 任务与中断读取每条总线的启动状态。
static volatile uint8_t can_recovering; // 失败总线重新启动期间禁止提交、发送和接收派发。
static uint8_t can_filter_quarantined; // 模式恢复失败后锁定该总线，直到系统重启。
// 只查询有效总线的启动位；未配置的总线不能调用 HAL 收发。
static uint8_t CANBusIsActive(uint8_t bus_index)
{
    return (uint8_t)(bus_index < DEVICE_CAN_CNT &&
                     (can_service_started & (uint8_t)(1u << bus_index)) != 0u &&
                     (can_recovering & (uint8_t)(1u << bus_index)) == 0u);
}
#ifdef BXCAN
static uint8_t can1_filter_idx;
static uint8_t can2_filter_idx = 14u;
#endif

/* ---------------- 总线状态与接收查表(2026-09 CAN 重构) ----------------
   - 每条总线一份统计: 发送丢帧/完成、BusOff、错误计数;
   - rx_lookup[总线][rx_id & 0x1FF] 做 O(1) 派发, 命中后再校验完整 rx_id;
     512×4B×3 总线 = 6KB RAM(用完整 11 位做直表要 24KB, 太贵)。 */
#define CAN_RX_LOOKUP_SIZE 0x200u

// 每路总线独立排队：中断复制帧，MotorControlTask 在控制计算前按顺序派发。
// 电机滤波和多圈角度需要连续帧，队列满时丢新帧并计数，不覆盖任务正在读取的旧项。
#define CAN_RXQ_DEPTH 32u /* 每条总线 32 帧 ≈ 32ms 缓冲 @1k 帧/s */

typedef struct
{
    CANInstance *instance; /* 已由 rx_lookup 查好的归属实例 */
    uint8_t len;           /* 数据长度 0~8 */
    uint8_t data[8];       /* 帧数据 */
    uint32_t timestamp_ms;
    uint32_t timestamp_cycles;
    uint32_t generation;   /* 入队时的总线代际，用于丢弃故障前的旧反馈。 */
} CANRxItem_t;

/* 接收中断只保留"有新帧"和"丢帧诊断"两种:
   FULL / WATERMARK 与 NEW_MESSAGE 触发的是同一个处理函数, 全开等于同一事件多次进中断 */
#define FDCAN_RX_ACTIVE_ITS                                            \
    (FDCAN_IT_RX_FIFO0_NEW_MESSAGE | FDCAN_IT_RX_FIFO0_MESSAGE_LOST |  \
     FDCAN_IT_RX_FIFO1_NEW_MESSAGE | FDCAN_IT_RX_FIFO1_MESSAGE_LOST)
/* 发送完成中断要按 TX 缓冲位掩码使能; 必须与 fdcan.c 的 TxFifoQueueElmtsNbr(16) 一致 */
#define FDCAN_TX_BUFFER_IT_MASK 0xFFFFu
#define FDCAN_TX_ACTIVE_ITS (FDCAN_IT_TX_COMPLETE | FDCAN_IT_TX_ABORT_COMPLETE)
#define CAN_TX_RESULT_HISTORY 32u
#define CAN_TX_PERIODIC_STREAMS 16u
#define CAN_TX_MAX_HW_BUFFERS 16u
#define CAN_TX_ATTEMPTS_TOTAL 4u /* 首次尝试加用户选择的最多3次未接受重试。 */
#define CAN_TX_INVALID_RESULT_INDEX 0xffu

typedef struct
{
    uint32_t identifier;
    uint8_t id_type;
    uint8_t length;
    uint8_t data[8];
} CANQueuedFrame_t;

typedef struct
{
    uint8_t used;
    uint8_t periodic;
    uint8_t active;
    uint8_t frame_count;
    uint8_t next_frame;
    uint8_t attempts;
    uint8_t result_index;
    uint8_t priority;
    uint8_t interruptible;
    uint32_t expiry_ms;
    const void *source;
    uint32_t key;
    CAN_TxPeriodicPrepareFn_t periodic_prepare;
    CANQueuedFrame_t frames[CAN_TX_MAX_BATCH_FRAMES];
} CANQueuedRequest_t;

typedef struct
{
    uint8_t used;
    uint8_t pending_hw;
    uint8_t accepted_frames;
    uint8_t completed_frames;
    uint8_t frame_count;
    uint32_t request_id;
    CAN_TxResultState_t state;
    uint32_t last_hal_error;
} CANTxResultSlot_t;

typedef struct
{
    uint8_t used;
    const void *source;
    uint32_t key;
    uint16_t frame_bits;
    uint32_t period_us;
} CANPeriodicReservation_t;

typedef struct
{
    uint8_t initialized;
    CAN_TxBusConfig_t config;
    CANQueuedRequest_t queue[CAN_TX_MAX_QUEUE_ITEMS];
    CANTxResultSlot_t results[CAN_TX_RESULT_HISTORY];
    CANPeriodicReservation_t reservations[CAN_TX_PERIODIC_STREAMS];
    uint8_t hw_result[CAN_TX_MAX_HW_BUFFERS];
    uint8_t active_queue_index;
    uint32_t next_request_id;
    uint16_t result_cursor;
    uint16_t queued_count;
    uint16_t periodic_reserved_permille;
    CAN_TxSchedulerStats_t stats;
} CANTxScheduler_t;

typedef struct
{
    FDCAN_HandleTypeDef *handle;
    CAN_TxStats_t tx;
    CAN_BusStats_t bus;
    uint8_t tx_ownership_uncertain; /* HAL 已接受但无法关联硬件槽，重启前必须隔离。 */
    volatile uint32_t rx_generation; /* 本总线故障时递增，其他总线不受影响。 */
    CANTxScheduler_t tx_scheduler;
    /* ---- 接收队列(SPSC): 中断入队, 任务用 CANProcessRx() 消费 ---- */
    CANRxItem_t rxq[CAN_RXQ_DEPTH];
    volatile uint16_t rxq_head;    /* 生产者(中断)写 */
    volatile uint16_t rxq_tail;    /* 消费者(任务)写 */
    volatile uint32_t rxq_drop;    /* 队列满丢弃帧数 */
    volatile uint16_t rxq_peak;    /* 历史最大占用(诊断: 判断深度够不够) */
    volatile uint32_t rx_dispatch; /* 已派发给回调的帧数 */
} CANBusState_t;

#if defined(__arm__) && defined(__GNUC__)
/* H723主RAM余量很小；CAN收发只由CPU访问，可把固定工作区放到未使用的RAM_D2。 */
#define CAN_BUS_WORKSPACE __attribute__((section(".can_workspace"), aligned(32), used))
#else
#define CAN_BUS_WORKSPACE
#endif
static CANBusState_t can_bus[DEVICE_CAN_CNT] CAN_BUS_WORKSPACE;
static volatile uint8_t can_bus_state_initialized; // 中断只在工作区完整发布后访问它。
static void CANBusStateEnsureInitialized(void);
static CANInstance *rx_lookup[DEVICE_CAN_CNT][CAN_RX_LOOKUP_SIZE];
/* 每条总线的标准与扩展过滤器各自记录已用槽；注册表最多16项。 */
static uint32_t can_filter_used[DEVICE_CAN_CNT][2];

/* 诊断: 最近一次总线配置的结果(HAL 返回码/状态/中断寄存器), 用于排查"中断不触发" */
volatile uint32_t can_diag_start_ret = 0;
volatile uint32_t can_diag_state = 0;
volatile uint32_t can_diag_act_ret = 0;
volatile uint32_t can_diag_ie = 0;
volatile uint32_t can_diag_ile = 0;
volatile uint32_t can_diag_ils = 0;

/* 查询当前控制器为对应 ID 类型分配的过滤器数量。 */
static uint32_t CANFilterLimit(uint8_t bus_idx, CAN_TxIdType_t id_type)
{
    if (bus_idx >= DEVICE_CAN_CNT || (id_type != CAN_TX_ID_STANDARD && id_type != CAN_TX_ID_EXTENDED))
        return 0u; // 非法总线或类型没有槽位。
    const FDCAN_HandleTypeDef *handle = can_bus[bus_idx].handle; // 读取该总线自己的硬件配置。
    const uint32_t configured = id_type == CAN_TX_ID_STANDARD ?
        handle->Init.StdFiltersNbr : handle->Init.ExtFiltersNbr; // 不混用两类过滤器区域。
    return configured < CAN_MX_REGISTER_CNT ? configured : CAN_MX_REGISTER_CNT; // 登记表最多允许16项。
}

/* ---------------- 性能探针(见 docs/学习笔记/03) ---------------- */
volatile DWT_Probe_t can_prof_tx = {0};
volatile uint32_t can_prof_tx_spins = 0;   /* 保留: 自旋已删除, 恒为 0 */
volatile uint32_t can_prof_tx_full = 0;    /* 保留: 兼容旧观察, 等于 tx_drop 累计 */
volatile DWT_Probe_t can_prof_rx_isr = {0};
volatile uint32_t can_prof_rx_frames = 0;
volatile DWT_Probe_t can_prof_rx_dispatch = {0}; /* 任务侧逐帧派发耗时(改造后新增) */
volatile uint32_t can_rxq_drop_total = 0;        /* 三条总线接收队列满丢帧合计 */
volatile uint16_t can_rxq_peak_max = 0;          /* 三条总线里队列占用的历史峰值 */

// 将已知 HAL 句柄对应到本地总线记录；未找到返回 NULL。
static CANBusState_t *CANBusFromHandle(FDCAN_HandleTypeDef *_handle)
{
    CANBusStateEnsureInitialized();
    for (uint8_t i = 0; i < DEVICE_CAN_CNT; ++i)
    {
        if (can_bus[i].handle == _handle)
            return &can_bus[i];
    }
    return NULL;
}

// 查询三条总线的数组编号，未知句柄返回 0xff。
static uint8_t CANBusIndexFromHandle(FDCAN_HandleTypeDef *_handle)
{
    CANBusStateEnsureInitialized();
    for (uint8_t i = 0; i < DEVICE_CAN_CNT; ++i)
    {
        if (can_bus[i].handle == _handle)
            return i;
    }
    return 0xFFu;
}

static const CAN_TxBusConfig_t can_tx_default_config[DEVICE_CAN_CNT] = {
    {.queue_capacity = 16u, .utilization_limit_permille = 700u, .nominal_bitrate_bps = 1000000u},
    {.queue_capacity = 12u, .utilization_limit_permille = 650u, .nominal_bitrate_bps = 1000000u},
    {.queue_capacity = 8u, .utilization_limit_permille = 500u, .nominal_bitrate_bps = 1000000u},
};

static void CANBusStateEnsureInitialized(void)
{
    if (can_bus_state_initialized != 0u)
        return;

    // RAM_D2工作区为NOLOAD，首次使用时显式清零并恢复各总线的默认配置。
    memset(can_bus, 0, sizeof(can_bus));
    can_bus[0].handle = &hfdcan1;
    can_bus[1].handle = &hfdcan2;
    can_bus[2].handle = &hfdcan3;
    for (uint8_t bus_index = 0u; bus_index < DEVICE_CAN_CNT; ++bus_index)
    {
        CANTxScheduler_t *scheduler = &can_bus[bus_index].tx_scheduler;
        scheduler->config = can_tx_default_config[bus_index];
        scheduler->active_queue_index = CAN_TX_INVALID_RESULT_INDEX;
        scheduler->next_request_id = 1u;
        scheduler->initialized = 1u;
        for (uint8_t i = 0u; i < CAN_TX_MAX_HW_BUFFERS; ++i)
            scheduler->hw_result[i] = CAN_TX_INVALID_RESULT_INDEX;
    }
    __DMB(); // 先完成工作区写入，再允许中断读取。
    can_bus_state_initialized = 1u;
}

void CANInitWorkspace(void)
{
    CANBusStateEnsureInitialized();
}

static void CANTxSchedulerEnsureInitialized(uint8_t bus_index)
{
    (void)bus_index;
    CANBusStateEnsureInitialized();
}

static uint8_t CANFrameValid(const CAN_TxFrame_t *frame)
{
    if (frame == NULL || frame->length > 8u || (frame->length != 0u && frame->data == NULL))
        return 0u;
    if (frame->identifier_type == CAN_TX_ID_STANDARD)
        return (uint8_t)(frame->identifier <= 0x7ffu);
    if (frame->identifier_type == CAN_TX_ID_EXTENDED)
        return (uint8_t)(frame->identifier <= 0x1fffffffu);
    return 0u;
}

uint16_t CANClassicFrameWorstCaseBits(CAN_TxIdType_t identifier_type, uint8_t data_length)
{
    if (data_length > 8u)
        return 0u;

    uint32_t stuffable_bits;
    if (identifier_type == CAN_TX_ID_STANDARD)
        stuffable_bits = 34u + 8u * data_length;
    else if (identifier_type == CAN_TX_ID_EXTENDED)
        stuffable_bits = 54u + 8u * data_length;
    else
        return 0u;

    /* 固定部分含 CRC/ACK/帧间隔；可填充部分按每连续4位插入一位保守估算。 */
    return (uint16_t)(stuffable_bits + 14u + (stuffable_bits - 1u) / 4u);
}

static uint32_t CANTxReservationPermille(const CANTxScheduler_t *scheduler,
                                        const CANPeriodicReservation_t *replacement,
                                        uint8_t replacement_index)
{
    uint64_t total_permille = 0u;
    for (uint8_t i = 0u; i < CAN_TX_PERIODIC_STREAMS; ++i)
    {
        const CANPeriodicReservation_t *entry = &scheduler->reservations[i];
        if (i == replacement_index && replacement != NULL)
            entry = replacement;
        if (entry->used == 0u)
            continue;

        const uint64_t numerator = (uint64_t)entry->frame_bits * 1000000000ull;
        const uint64_t denominator = (uint64_t)entry->period_us *
                                     scheduler->config.nominal_bitrate_bps;
        if (denominator == 0u)
            return UINT32_MAX;
        /* 用商和余数向上取整，避免高位速率配置下 numerator+denominator 溢出。 */
        total_permille += numerator / denominator +
                          (numerator % denominator != 0u ? 1u : 0u);
        if (total_permille > UINT32_MAX)
            return UINT32_MAX;
    }
    return (uint32_t)total_permille;
}

static int CANFindPeriodicReservation(const CANTxScheduler_t *scheduler,
                                      const void *source, uint32_t key)
{
    for (uint8_t i = 0u; i < CAN_TX_PERIODIC_STREAMS; ++i)
    {
        if (scheduler->reservations[i].used != 0u &&
            scheduler->reservations[i].source == source &&
            scheduler->reservations[i].key == key)
            return (int)i;
    }
    return -1;
}

static int CANTxFindFreeQueueSlot(const CANTxScheduler_t *scheduler)
{
    for (uint8_t i = 0u; i < CAN_TX_MAX_QUEUE_ITEMS; ++i)
        if (scheduler->queue[i].used == 0u)
            return (int)i;
    return -1;
}

static int CANTxFindPeriodicQueueSlot(const CANTxScheduler_t *scheduler,
                                     const void *source, uint32_t key)
{
    for (uint8_t i = 0u; i < CAN_TX_MAX_QUEUE_ITEMS; ++i)
    {
        const CANQueuedRequest_t *entry = &scheduler->queue[i];
        if (entry->used != 0u && entry->periodic != 0u &&
            entry->source == source && entry->key == key)
            return (int)i;
    }
    return -1;
}

static int CANTxFindResultById(uint32_t request_id)
{
    if (request_id == 0u)
        return -1;
    for (uint8_t bus_index = 0u; bus_index < DEVICE_CAN_CNT; ++bus_index)
    {
        CANTxScheduler_t *scheduler = &can_bus[bus_index].tx_scheduler;
        if (scheduler->initialized == 0u)
            continue;
        for (uint8_t i = 0u; i < CAN_TX_RESULT_HISTORY; ++i)
            if (scheduler->results[i].used != 0u &&
                scheduler->results[i].request_id == request_id)
                return (int)(((uint16_t)bus_index << 8u) | i);
    }
    return -1;
}

static int CANTxAllocateResult(CANTxScheduler_t *scheduler)
{
    for (uint8_t attempt = 0u; attempt < CAN_TX_RESULT_HISTORY; ++attempt)
    {
        uint8_t index = (uint8_t)((scheduler->result_cursor + attempt) % CAN_TX_RESULT_HISTORY);
        CANTxResultSlot_t *result = &scheduler->results[index];
        uint8_t queued_reference = 0u;
        // 中断可能先标记失败，任务稍后才清理该组；期间结果槽仍被队列引用。
        if (result->used && result->pending_hw == 0u &&
            (result->state == CAN_TX_RESULT_COMPLETE || result->state == CAN_TX_RESULT_FAILED ||
             result->state == CAN_TX_RESULT_EXPIRED || result->state == CAN_TX_RESULT_CANCELLED ||
             result->state == CAN_TX_RESULT_SUPERSEDED))
            for (uint8_t q = 0u; q < CAN_TX_MAX_QUEUE_ITEMS; ++q)
                if (scheduler->queue[q].used && scheduler->queue[q].result_index == index) queued_reference = 1u;
        if (result->used == 0u ||
            ((result->state == CAN_TX_RESULT_COMPLETE || result->state == CAN_TX_RESULT_FAILED ||
              result->state == CAN_TX_RESULT_EXPIRED || result->state == CAN_TX_RESULT_CANCELLED ||
              result->state == CAN_TX_RESULT_SUPERSEDED) && result->pending_hw == 0u && !queued_reference))
        {
            scheduler->result_cursor = (uint16_t)((index + 1u) % CAN_TX_RESULT_HISTORY);
            memset(result, 0, sizeof(*result));
            result->used = 1u;
            return index;
        }
    }
    return -1;
}

static uint32_t CANTxNextRequestId(CANTxScheduler_t *scheduler, uint8_t bus_index)
{
    uint32_t sequence = scheduler->next_request_id++ & 0x3fffffffu;
    if (sequence == 0u)
        sequence = scheduler->next_request_id++ & 0x3fffffffu;
    if ((scheduler->next_request_id & 0x3fffffffu) == 0u)
        scheduler->next_request_id = 1u;
    return ((uint32_t)(bus_index + 1u) << 30u) | sequence;
}

static void CANTxSetTerminal(CANTxScheduler_t *scheduler, uint8_t result_index,
                             CAN_TxResultState_t state)
{
    CANTxResultSlot_t *result = &scheduler->results[result_index];
    if (result->state == CAN_TX_RESULT_COMPLETE || result->state == CAN_TX_RESULT_FAILED ||
        result->state == CAN_TX_RESULT_EXPIRED || result->state == CAN_TX_RESULT_CANCELLED ||
        result->state == CAN_TX_RESULT_SUPERSEDED)
        return;

    result->state = state;
    switch (state)
    {
    case CAN_TX_RESULT_COMPLETE: scheduler->stats.completed++; break;
    case CAN_TX_RESULT_FAILED: scheduler->stats.failed++; break;
    case CAN_TX_RESULT_EXPIRED: scheduler->stats.expired++; break;
    case CAN_TX_RESULT_CANCELLED: scheduler->stats.cancelled++; break;
    case CAN_TX_RESULT_SUPERSEDED: scheduler->stats.superseded++; break;
    default: break;
    }
}

static uint8_t CANSubmitContextValid(void)
{
    return (uint8_t)(__get_IPSR() == 0u);
}

static void CANTxUpdateQueueStats(CANTxScheduler_t *scheduler)
{
    scheduler->stats.queued = scheduler->queued_count;
    if (scheduler->queued_count > scheduler->stats.queue_peak)
        scheduler->stats.queue_peak = scheduler->queued_count;
    scheduler->stats.periodic_reserved_permille = scheduler->periodic_reserved_permille;
}

static void CANTxExpireQueue(CANTxScheduler_t *scheduler, uint32_t now);

static uint8_t CANTxConfigValid(const CAN_TxBusConfig_t *config)
{
    return (uint8_t)(config != NULL && config->queue_capacity > 0u &&
                     config->queue_capacity <= CAN_TX_MAX_QUEUE_ITEMS &&
                     config->utilization_limit_permille > 0u &&
                     config->utilization_limit_permille <= 1000u &&
                     config->nominal_bitrate_bps > 0u);
}

CAN_TxSubmitStatus_t CANConfigureTxBus(FDCAN_HandleTypeDef *hcan,
                                       const CAN_TxBusConfig_t *config)
{
    const uint8_t bus_index = CANBusIndexFromHandle(hcan);
    if (bus_index == 0xffu || !CANTxConfigValid(config))
        return CAN_TX_SUBMIT_INVALID;
    if ((can_service_started & (uint8_t)(1u << bus_index)) != 0u)
        return CAN_TX_SUBMIT_NOT_READY;

    CANTxSchedulerEnsureInitialized(bus_index);
    taskENTER_CRITICAL();
    if ((can_service_started & (uint8_t)(1u << bus_index)) != 0u)
    {
        taskEXIT_CRITICAL(); // 配置与总线启动竞争时，启动后不能修改发送预算。
        return CAN_TX_SUBMIT_NOT_READY;
    }
    CANTxScheduler_t *scheduler = &can_bus[bus_index].tx_scheduler;
    if (scheduler->queued_count != 0u || scheduler->periodic_reserved_permille != 0u)
    {
        taskEXIT_CRITICAL();
        return CAN_TX_SUBMIT_NOT_READY;
    }
    scheduler->config = *config;
    taskEXIT_CRITICAL();
    return CAN_TX_SUBMIT_ACCEPTED;
}

static uint8_t CANTxRequestDeadlineValid(uint32_t deadline_ms)
{
    return (uint8_t)(deadline_ms > 0u && deadline_ms <= (uint32_t)INT32_MAX);
}

static int CANTxFindFreeReservation(const CANTxScheduler_t *scheduler)
{
    for (uint8_t i = 0u; i < CAN_TX_PERIODIC_STREAMS; ++i)
        if (scheduler->reservations[i].used == 0u)
            return (int)i;
    return -1;
}

static void CANTxCopyFrame(CANQueuedFrame_t *destination, const CAN_TxFrame_t *source)
{
    destination->identifier = source->identifier;
    destination->id_type = (uint8_t)source->identifier_type;
    destination->length = source->length;
    memset(destination->data, 0, sizeof(destination->data));
    if (source->length != 0u)
        memcpy(destination->data, source->data, source->length);
}

static uint32_t CANTxAllocateRequestId(CANTxScheduler_t *scheduler, uint8_t bus_index,
                                      uint8_t result_index)
{
    uint32_t id = CANTxNextRequestId(scheduler, bus_index);
    CANTxResultSlot_t *result = &scheduler->results[result_index];
    result->request_id = id;
    return id;
}

CAN_TxSubmitStatus_t CANSubmitPeriodicPrepared(FDCAN_HandleTypeDef *hcan, const void *source,
                                       uint32_t key, const CAN_TxFrame_t *frame,
                                       CAN_TxPriority_t priority, uint32_t deadline_ms,
                                       uint32_t period_us, CAN_TxPeriodicPrepareFn_t prepare,
                                       uint32_t *request_id)
{
    const uint8_t bus_index = CANBusIndexFromHandle(hcan);
    if (request_id == NULL || source == NULL || !CANFrameValid(frame) ||
        priority < CAN_TX_PRIORITY_URGENT || priority > CAN_TX_PRIORITY_BACKGROUND ||
        !CANTxRequestDeadlineValid(deadline_ms) ||
        period_us == 0u)
        return CAN_TX_SUBMIT_INVALID;
    if (!CANSubmitContextValid())
        return CAN_TX_SUBMIT_CONTEXT;
    if (!CANBusIsActive(bus_index))
        return CAN_TX_SUBMIT_NOT_READY;

    CANTxSchedulerEnsureInitialized(bus_index);
    taskENTER_CRITICAL();
    if (!CANBusIsActive(bus_index))
    {
        taskEXIT_CRITICAL(); // 总线可能在进入临界区前被故障处理暂停。
        return CAN_TX_SUBMIT_NOT_READY;
    }
    CANTxScheduler_t *scheduler = &can_bus[bus_index].tx_scheduler;
    const uint32_t now = HAL_GetTick();
    CANTxExpireQueue(scheduler, now);
    const int queue_index = CANTxFindPeriodicQueueSlot(scheduler, source, key);
    const int reservation_index = CANFindPeriodicReservation(scheduler, source, key);
    const int target_reservation_index = reservation_index >= 0
        ? reservation_index : CANTxFindFreeReservation(scheduler);
    if (target_reservation_index < 0)
    {
        scheduler->stats.rejected++;
        scheduler->stats.bandwidth_rejected++;
        taskEXIT_CRITICAL();
        return CAN_TX_SUBMIT_BANDWIDTH;
    }

    CANPeriodicReservation_t candidate_reservation = {
        .used = 1u,
        .source = source,
        .key = key,
        .frame_bits = CANClassicFrameWorstCaseBits(frame->identifier_type, frame->length),
        .period_us = period_us,
    };
    const uint32_t reserved_permille = CANTxReservationPermille(
        scheduler, &candidate_reservation, (uint8_t)target_reservation_index);
    if (reserved_permille > scheduler->config.utilization_limit_permille)
    {
        scheduler->stats.rejected++;
        scheduler->stats.bandwidth_rejected++;
        taskEXIT_CRITICAL();
        return CAN_TX_SUBMIT_BANDWIDTH;
    }

    const int target_queue_index = queue_index >= 0 ? queue_index : CANTxFindFreeQueueSlot(scheduler);
    if (target_queue_index < 0 ||
        (queue_index < 0 && scheduler->queued_count >= scheduler->config.queue_capacity))
    {
        scheduler->stats.rejected++;
        taskEXIT_CRITICAL();
        return CAN_TX_SUBMIT_FULL;
    }

    const int result_index = CANTxAllocateResult(scheduler);
    if (result_index < 0)
    {
        scheduler->stats.rejected++;
        taskEXIT_CRITICAL();
        return CAN_TX_SUBMIT_FULL;
    }

    const uint32_t token = CANTxAllocateRequestId(scheduler, bus_index, (uint8_t)result_index);
    CANTxResultSlot_t *result = &scheduler->results[result_index];
    result->frame_count = 1u;

    CANQueuedRequest_t *queued = &scheduler->queue[target_queue_index];
    if (queued->used != 0u)
    {
        CANTxSetTerminal(scheduler, queued->result_index, CAN_TX_RESULT_SUPERSEDED);
        scheduler->stats.coalesced++;
    }
    else
    {
        scheduler->queued_count++;
        scheduler->stats.submitted++;
    }

    const uint8_t was_active = queued->active;
    memset(queued, 0, sizeof(*queued));
    queued->used = 1u;
    queued->periodic = 1u;
    queued->active = was_active;
    queued->frame_count = 1u;
    queued->result_index = (uint8_t)result_index;
    queued->priority = (uint8_t)priority;
    queued->expiry_ms = now + deadline_ms;
    queued->source = source;
    queued->key = key;
    queued->periodic_prepare = prepare;
    CANTxCopyFrame(&queued->frames[0], frame);

    scheduler->reservations[target_reservation_index] = candidate_reservation;
    scheduler->periodic_reserved_permille = (uint16_t)reserved_permille;
    result->state = queued->active != 0u ? CAN_TX_RESULT_ACTIVE : CAN_TX_RESULT_QUEUED;
    CANTxUpdateQueueStats(scheduler);
    *request_id = token;
    taskEXIT_CRITICAL();
    return queue_index >= 0 ? CAN_TX_SUBMIT_COALESCED : CAN_TX_SUBMIT_ACCEPTED;
}

CAN_TxSubmitStatus_t CANSubmitPeriodic(FDCAN_HandleTypeDef *hcan, const void *source,
                                       uint32_t key, const CAN_TxFrame_t *frame,
                                       CAN_TxPriority_t priority, uint32_t deadline_ms,
                                       uint32_t period_us, uint32_t *request_id)
{
    return CANSubmitPeriodicPrepared(hcan, source, key, frame, priority, deadline_ms,
                                     period_us, NULL, request_id);
}

static CAN_TxSubmitStatus_t CANSubmitBatchInternal(FDCAN_HandleTypeDef *hcan, const void *source,
                                           const CAN_TxFrame_t *frames, size_t frame_count,
                                           CAN_TxPriority_t priority, uint32_t deadline_ms,
                                           uint32_t *request_id, uint8_t interruptible, uint8_t absolute)
{
    const uint8_t bus_index = CANBusIndexFromHandle(hcan);
    if (request_id == NULL || frames == NULL || frame_count == 0u ||
        frame_count > CAN_TX_MAX_BATCH_FRAMES ||
        priority < CAN_TX_PRIORITY_URGENT || priority > CAN_TX_PRIORITY_BACKGROUND ||
        (!absolute && !CANTxRequestDeadlineValid(deadline_ms)))
        return CAN_TX_SUBMIT_INVALID;
    for (size_t i = 0u; i < frame_count; ++i)
        if (!CANFrameValid(&frames[i]))
            return CAN_TX_SUBMIT_INVALID;
    if (!CANSubmitContextValid())
        return CAN_TX_SUBMIT_CONTEXT;
    if (!CANBusIsActive(bus_index))
        return CAN_TX_SUBMIT_NOT_READY;

    CANTxSchedulerEnsureInitialized(bus_index);
    taskENTER_CRITICAL();
    if (!CANBusIsActive(bus_index))
    {
        taskEXIT_CRITICAL(); // 进入临界区前若已暂停，不能把旧命令放入队列。
        return CAN_TX_SUBMIT_NOT_READY;
    }
    CANTxScheduler_t *scheduler = &can_bus[bus_index].tx_scheduler;
    const uint32_t now = HAL_GetTick();
    if (absolute && !CANTxRequestDeadlineValid(deadline_ms-now))
    {
        // 截止时刻已到或相距过远时拒绝，不能从本次入队重新计算有效期。
        taskEXIT_CRITICAL();
        return CAN_TX_SUBMIT_INVALID;
    }
    CANTxExpireQueue(scheduler, now);
    const int queue_index = CANTxFindFreeQueueSlot(scheduler);
    if (queue_index < 0 || scheduler->queued_count >= scheduler->config.queue_capacity)
    {
        scheduler->stats.rejected++;
        taskEXIT_CRITICAL();
        return CAN_TX_SUBMIT_FULL;
    }
    const int result_index = CANTxAllocateResult(scheduler);
    if (result_index < 0)
    {
        scheduler->stats.rejected++;
        taskEXIT_CRITICAL();
        return CAN_TX_SUBMIT_FULL;
    }

    const uint32_t token = CANTxAllocateRequestId(scheduler, bus_index, (uint8_t)result_index);
    CANTxResultSlot_t *result = &scheduler->results[result_index];
    result->frame_count = (uint8_t)frame_count;
    result->state = CAN_TX_RESULT_QUEUED;

    CANQueuedRequest_t *queued = &scheduler->queue[queue_index];
    memset(queued, 0, sizeof(*queued));
    queued->used = 1u;
    queued->frame_count = (uint8_t)frame_count;
    queued->result_index = (uint8_t)result_index;
    queued->priority = (uint8_t)priority;
    queued->interruptible = interruptible;
    queued->expiry_ms = absolute ? deadline_ms : now + deadline_ms;
    queued->source = source;
    for (size_t i = 0u; i < frame_count; ++i)
        CANTxCopyFrame(&queued->frames[i], &frames[i]);

    scheduler->queued_count++;
    scheduler->stats.submitted++;
    CANTxUpdateQueueStats(scheduler);
    *request_id = token;
    taskEXIT_CRITICAL();
    return CAN_TX_SUBMIT_ACCEPTED;
}

CAN_TxSubmitStatus_t CANSubmitOneShotBatch(FDCAN_HandleTypeDef *hcan, const void *source,
    const CAN_TxFrame_t *frames, size_t frame_count, CAN_TxPriority_t priority,
    uint32_t deadline_ms, uint32_t *request_id)
{
    return CANSubmitBatchInternal(hcan,source,frames,frame_count,priority,deadline_ms,request_id,0u,0u);
}

CAN_TxSubmitStatus_t CANSubmitInterruptibleBatch(FDCAN_HandleTypeDef *hcan, const void *source,
    const CAN_TxFrame_t *frames, size_t frame_count, CAN_TxPriority_t priority,
    uint32_t deadline_ms, uint32_t *request_id)
{
    return CANSubmitBatchInternal(hcan,source,frames,frame_count,priority,deadline_ms,request_id,1u,0u);
}

CAN_TxSubmitStatus_t CANSubmitInterruptibleBatchUntil(FDCAN_HandleTypeDef *hcan, const void *source,
    const CAN_TxFrame_t *frames, size_t frame_count, CAN_TxPriority_t priority,
    uint32_t expiry_ms, uint32_t *request_id)
{
    return CANSubmitBatchInternal(hcan,source,frames,frame_count,priority,expiry_ms,request_id,1u,1u);
}

CAN_TxSubmitStatus_t CANSubmitOneShot(FDCAN_HandleTypeDef *hcan, const void *source,
                                      const CAN_TxFrame_t *frame,
                                      CAN_TxPriority_t priority, uint32_t deadline_ms,
                                      uint32_t *request_id)
{
    return CANSubmitOneShotBatch(hcan, source, frame, 1u, priority, deadline_ms, request_id);
}

static uint8_t CANTxStateTerminal(CAN_TxResultState_t state)
{
    return (uint8_t)(state == CAN_TX_RESULT_COMPLETE || state == CAN_TX_RESULT_FAILED ||
                     state == CAN_TX_RESULT_EXPIRED || state == CAN_TX_RESULT_CANCELLED ||
                     state == CAN_TX_RESULT_SUPERSEDED);
}

static void CANTxReleaseQueueSlot(CANTxScheduler_t *scheduler, uint8_t queue_index)
{
    memset(&scheduler->queue[queue_index], 0, sizeof(scheduler->queue[queue_index]));
    if (scheduler->queued_count > 0u)
        scheduler->queued_count--;
    if (scheduler->active_queue_index == queue_index)
        scheduler->active_queue_index = CAN_TX_INVALID_RESULT_INDEX;
    CANTxUpdateQueueStats(scheduler);
}

// 调用方持有发送临界区；只撤销尚在软件队列的部分，硬件已接收的帧另行处理。
static void CANTxCancelQueued(CANTxScheduler_t *scheduler)
{
    for (uint8_t index = 0u; index < CAN_TX_MAX_QUEUE_ITEMS; ++index)
    {
        if (scheduler->queue[index].used == 0u) continue; // 空槽不影响计数和请求状态。
        CANTxSetTerminal(scheduler, scheduler->queue[index].result_index, CAN_TX_RESULT_CANCELLED); // 告诉请求者剩余部分已取消。
        CANTxReleaseQueueSlot(scheduler, index); // 删除软件副本，不能在重启后继续提交。
    }
}

static void CANTxFailBeforeAcceptance(CANBusState_t *bus, uint8_t queue_index,
                                      uint32_t hal_error)
{
    CANTxScheduler_t *scheduler = &bus->tx_scheduler;
    CANQueuedRequest_t *queued = &scheduler->queue[queue_index];
    CANTxResultSlot_t *result = &scheduler->results[queued->result_index];
    if (hal_error != HAL_OK)
    {
        result->last_hal_error = hal_error;
        scheduler->stats.hal_rejected++;
        bus->tx.tx_error++;
    }
    else
    {
        scheduler->stats.fifo_busy++;
        bus->tx.tx_drop++;
        can_prof_tx_full = bus->tx.tx_drop;
    }

    if (queued->periodic != 0u)
    {
        // 周期目标只允许尝试一次；下一周期会重新提交最新计算值。
        CANTxSetTerminal(scheduler, queued->result_index, CAN_TX_RESULT_FAILED);
        CANTxReleaseQueueSlot(scheduler, queue_index);
        return;
    }

    if (queued->periodic == 0u)
    {
        queued->attempts++;
        if (queued->attempts >= CAN_TX_ATTEMPTS_TOTAL)
        {
            CANTxSetTerminal(scheduler, queued->result_index, CAN_TX_RESULT_FAILED);
            CANTxReleaseQueueSlot(scheduler, queue_index);
            return;
        }
        scheduler->stats.retries++;
    }

    // 尚无帧交给 HAL 时让出选择权，避免低优先级重试挡住新到的紧急请求。
    if (queued->next_frame == 0u)
    {
        queued->active = 0u;
        if (scheduler->active_queue_index == queue_index)
            scheduler->active_queue_index = CAN_TX_INVALID_RESULT_INDEX;
        result->state = CAN_TX_RESULT_QUEUED;
    }
    else
        result->state = CAN_TX_RESULT_ACTIVE;
}

static uint8_t CANTxRequestExpired(const CANQueuedRequest_t *queued, uint32_t now)
{
    return (uint8_t)((uint32_t)(now - queued->expiry_ms) < 0x80000000u);
}

static void CANTxExpireQueue(CANTxScheduler_t *scheduler, uint32_t now)
{
    for (uint8_t i = 0u; i < CAN_TX_MAX_QUEUE_ITEMS; ++i)
    {
        CANQueuedRequest_t *queued = &scheduler->queue[i];
        if (queued->used != 0u && CANTxRequestExpired(queued, now) != 0u)
        {
            CANTxSetTerminal(scheduler, queued->result_index, CAN_TX_RESULT_EXPIRED);
            CANTxReleaseQueueSlot(scheduler, i);
        }
    }
}

static uint8_t CANTxSelectNextRequest(CANBusState_t *bus, uint32_t now)
{
    CANTxScheduler_t *scheduler = &bus->tx_scheduler;
    uint8_t selected = CAN_TX_INVALID_RESULT_INDEX;
    uint32_t selected_remaining = UINT32_MAX;

    for (uint8_t i = 0u; i < CAN_TX_MAX_QUEUE_ITEMS; ++i)
    {
        CANQueuedRequest_t *queued = &scheduler->queue[i];
        if (queued->used == 0u)
            continue;
        if (CANTxRequestExpired(queued, now) != 0u)
            continue;
        if (queued->active != 0u)
            continue;

        const uint32_t remaining = queued->expiry_ms - now;
        if (selected == CAN_TX_INVALID_RESULT_INDEX ||
            queued->priority < scheduler->queue[selected].priority ||
            (queued->priority == scheduler->queue[selected].priority &&
             remaining < selected_remaining))
        {
            selected = i;
            selected_remaining = remaining;
        }
    }

    if (selected != CAN_TX_INVALID_RESULT_INDEX)
    {
        scheduler->queue[selected].active = 1u;
        scheduler->active_queue_index = selected;
        scheduler->results[scheduler->queue[selected].result_index].state = CAN_TX_RESULT_ACTIVE;
    }
    return selected;
}

static uint8_t CANTxHardwareIndex(uint32_t buffer_mask)
{
    if (buffer_mask == 0u || (buffer_mask & (buffer_mask - 1u)) != 0u)
        return CAN_TX_INVALID_RESULT_INDEX;
    for (uint8_t i = 0u; i < CAN_TX_MAX_HW_BUFFERS; ++i)
        if (buffer_mask == (1u << i))
            return i;
    return CAN_TX_INVALID_RESULT_INDEX;
}

static void CANProcessTxSelected(uint8_t selected_bus)
{
    if (__get_IPSR() != 0u || can_service_started == 0u)
        return;

    for (uint8_t bus_index = 0u; bus_index < DEVICE_CAN_CNT; ++bus_index)
    {
        if (selected_bus != CAN_TX_INVALID_RESULT_INDEX && bus_index != selected_bus) continue;
        if (!CANBusIsActive(bus_index)) continue; // 没启用的总线不访问硬件。
        CANBusState_t *bus = &can_bus[bus_index];
        CANTxScheduler_t *scheduler = &bus->tx_scheduler;
        CANTxSchedulerEnsureInitialized(bus_index);
        const uint32_t probe_start = DWT_ProbeStart();

        taskENTER_CRITICAL();
        if (!CANBusIsActive(bus_index))
        {
            taskEXIT_CRITICAL(); // 在获取临界区前发生故障时，本轮不再接触控制器。
            continue;
        }
        uint32_t now = HAL_GetTick();
        CANTxExpireQueue(scheduler, now);
        uint8_t queue_index = scheduler->active_queue_index;
        if (queue_index != CAN_TX_INVALID_RESULT_INDEX &&
            scheduler->queue[queue_index].used == 0u)
            queue_index = CAN_TX_INVALID_RESULT_INDEX;
        if (queue_index != CAN_TX_INVALID_RESULT_INDEX && scheduler->queue[queue_index].interruptible)
        {
            CANQueuedRequest_t *active = &scheduler->queue[queue_index];
            for (uint8_t i=0u; i<CAN_TX_MAX_QUEUE_ITEMS; ++i)
            {
                CANQueuedRequest_t *waiting = &scheduler->queue[i];
                if (waiting->used && !waiting->active && waiting->priority < active->priority)
                {
                    // 已接受的帧不能撤回，放弃余帧并报告失败，使控制帧在下一次服务中优先提交。
                    CANTxSetTerminal(scheduler,active->result_index,CAN_TX_RESULT_FAILED);
                    CANTxReleaseQueueSlot(scheduler,queue_index);
                    queue_index=CAN_TX_INVALID_RESULT_INDEX;
                    break;
                }
            }
        }
        if (queue_index == CAN_TX_INVALID_RESULT_INDEX)
            queue_index = CANTxSelectNextRequest(bus, now);
        if (queue_index == CAN_TX_INVALID_RESULT_INDEX)
        {
            taskEXIT_CRITICAL();
            DWT_ProbeDone(&can_prof_tx, probe_start);
            continue;
        }

        CANQueuedRequest_t *queued = &scheduler->queue[queue_index];
        // 中断可能已将已接受的帧标为失败；丢弃尚未提交的余帧，避免发送不完整的旧消息。
        if (CANTxStateTerminal(scheduler->results[queued->result_index].state))
        {
            CANTxReleaseQueueSlot(scheduler, queue_index);
            taskEXIT_CRITICAL();
            DWT_ProbeDone(&can_prof_tx, probe_start);
            continue;
        }
        if (queued->next_frame >= queued->frame_count)
        {
            CANTxReleaseQueueSlot(scheduler, queue_index);
            taskEXIT_CRITICAL();
            DWT_ProbeDone(&can_prof_tx, probe_start);
            continue;
        }

        // 一次服务只查询并启动一帧，避免总线繁忙时阻塞电机周期。
        if (HAL_FDCAN_GetTxFifoFreeLevel(bus->handle) == 0u)
        {
            CANTxFailBeforeAcceptance(bus, queue_index, HAL_OK);
            taskEXIT_CRITICAL();
            DWT_ProbeDone(&can_prof_tx, probe_start);
            continue;
        }

        CANQueuedFrame_t *frame = &queued->frames[queued->next_frame];
        now = HAL_GetTick();
        if (CANTxRequestExpired(queued, now) != 0u)
        {
            CANTxSetTerminal(scheduler, queued->result_index, CAN_TX_RESULT_EXPIRED);
            CANTxReleaseQueueSlot(scheduler, queue_index);
            taskEXIT_CRITICAL();
            DWT_ProbeDone(&can_prof_tx, probe_start);
            continue;
        }
        // 周期目标可能已经在队列等待；每次交给 HAL 前都按当前许可改写副本。
        if (queued->periodic != 0u && queued->periodic_prepare != NULL)
            queued->periodic_prepare(queued->source, now,
                                     frame->data, frame->length);
        now = HAL_GetTick();
        if (CANTxRequestExpired(queued, now) != 0u)
        {
            CANTxSetTerminal(scheduler, queued->result_index, CAN_TX_RESULT_EXPIRED);
            CANTxReleaseQueueSlot(scheduler, queue_index);
            taskEXIT_CRITICAL();
            DWT_ProbeDone(&can_prof_tx, probe_start);
            continue;
        }
        FDCAN_TxHeaderTypeDef header = {0};
        header.Identifier = frame->identifier;
        header.IdType = frame->id_type == CAN_TX_ID_EXTENDED ? FDCAN_EXTENDED_ID : FDCAN_STANDARD_ID;
        header.TxFrameType = FDCAN_DATA_FRAME;
        header.DataLength = DLC_LookUp_Table[frame->length];
        header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
        header.BitRateSwitch = FDCAN_BRS_OFF;
        header.FDFormat = FDCAN_CLASSIC_CAN;
        header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
        header.MessageMarker = 0u;

        const HAL_StatusTypeDef hal_status = HAL_FDCAN_AddMessageToTxFifoQ(
            bus->handle, &header, (uint8_t *)frame->data);
        if (hal_status != HAL_OK)
        {
            CANTxFailBeforeAcceptance(bus, queue_index, (uint32_t)hal_status);
            taskEXIT_CRITICAL();
            DWT_ProbeDone(&can_prof_tx, probe_start);
            continue;
        }

        // HAL已复制负载到消息RAM；在开放TX中断前记录其FIFO槽归属，避免完成回调抢先到达。
        const uint8_t hardware_index = CANTxHardwareIndex(
            HAL_FDCAN_GetLatestTxFifoQRequestBuffer(bus->handle));
        const uint8_t result_index = queued->result_index;
        CANTxResultSlot_t *result = &scheduler->results[result_index];
        result->accepted_frames++;
        result->pending_hw++;
        queued->next_frame++;
        queued->attempts = 0u;
        bus->tx.tx_ok++;

        if (hardware_index == CAN_TX_INVALID_RESULT_INDEX ||
            scheduler->hw_result[hardware_index] != CAN_TX_INVALID_RESULT_INDEX)
        {
            // HAL已经接受但没有可关联的槽位时绝不重发，避免重复发送同一命令。
            bus->tx_ownership_uncertain = 1u; // 无法确认该硬件帧结束，后续故障不能直接重启。
            if (result->pending_hw > 0u)
                result->pending_hw--;
            CANTxSetTerminal(scheduler, result_index, CAN_TX_RESULT_FAILED);
            result->last_hal_error = (uint32_t)HAL_ERROR;
            bus->tx.tx_error++;
            CANTxReleaseQueueSlot(scheduler, queue_index);
        }
        else
        {
            scheduler->hw_result[hardware_index] = result_index;
            if (queued->next_frame >= queued->frame_count)
            {
                result->state = CAN_TX_RESULT_IN_FLIGHT;
                CANTxReleaseQueueSlot(scheduler, queue_index);
            }
            else
                result->state = CAN_TX_RESULT_ACTIVE;
        }

        taskEXIT_CRITICAL();
        DWT_ProbeDone(&can_prof_tx, probe_start);
    }
}

void CANProcessTx(void)
{
    CANProcessTxSelected(CAN_TX_INVALID_RESULT_INDEX);
}

void CANProcessTxBus(FDCAN_HandleTypeDef *hcan)
{
    if (hcan==NULL || __get_IPSR()!=0u) return;
    uint8_t bus_index=CANBusIndexFromHandle(hcan);
    if (bus_index==CAN_TX_INVALID_RESULT_INDEX) return;
    CANProcessTxSelected(bus_index);
}

uint8_t CANGetTxRequestState(uint32_t request_id, CAN_TxResult_t *result)
{
    if (request_id == 0u || result == NULL || __get_IPSR() != 0u)
        return 0u;
    taskENTER_CRITICAL();
    const int packed_index = CANTxFindResultById(request_id);
    if (packed_index < 0)
    {
        taskEXIT_CRITICAL();
        return 0u;
    }

    const uint8_t bus_index = (uint8_t)((uint32_t)packed_index >> 8u);
    const uint8_t result_index = (uint8_t)packed_index;
    const CANTxResultSlot_t *slot = &can_bus[bus_index].tx_scheduler.results[result_index];
    CAN_TxResult_t candidate = {
        .request_id = slot->request_id,
        .state = slot->state,
        .frame_count = slot->frame_count,
        .accepted_frames = slot->accepted_frames,
        .completed_frames = slot->completed_frames,
        .pending_frames = slot->pending_hw,
        .last_hal_error = slot->last_hal_error,
    };
    *result = candidate;
    taskEXIT_CRITICAL();
    return 1u;
}

uint8_t CANCancelTxRequest(uint32_t request_id)
{
    if (request_id == 0u || !CANSubmitContextValid())
        return 0u;
    taskENTER_CRITICAL();
    const int packed_index = CANTxFindResultById(request_id);
    if (packed_index < 0)
    {
        taskEXIT_CRITICAL();
        return 0u;
    }
    const uint8_t bus_index = (uint8_t)((uint32_t)packed_index >> 8u);
    const uint8_t result_index = (uint8_t)packed_index;
    CANTxScheduler_t *scheduler = &can_bus[bus_index].tx_scheduler;
    CANTxResultSlot_t *result = &scheduler->results[result_index];
    for (uint8_t i = 0u; i < CAN_TX_MAX_QUEUE_ITEMS; ++i)
    {
        CANQueuedRequest_t *queued = &scheduler->queue[i];
        if (queued->used != 0u && queued->result_index == result_index)
        {
            CANTxSetTerminal(scheduler, result_index, CAN_TX_RESULT_CANCELLED);
            CANTxReleaseQueueSlot(scheduler, i);
            taskEXIT_CRITICAL();
            return 1u;
        }
    }
    const uint8_t cancellable = (uint8_t)(result->state == CAN_TX_RESULT_QUEUED ||
                                          result->state == CAN_TX_RESULT_ACTIVE);
    if (cancellable != 0u)
        CANTxSetTerminal(scheduler, result_index, CAN_TX_RESULT_CANCELLED);
    taskEXIT_CRITICAL();
    return cancellable;
}

uint8_t CANGetTxSchedulerStats(FDCAN_HandleTypeDef *hcan,
                               CAN_TxSchedulerStats_t *stats)
{
    const uint8_t bus_index = CANBusIndexFromHandle(hcan);
    if (bus_index == 0xffu || stats == NULL || __get_IPSR() != 0u)
        return 0u;
    CANTxSchedulerEnsureInitialized(bus_index);
    taskENTER_CRITICAL();
    CAN_TxSchedulerStats_t candidate = can_bus[bus_index].tx_scheduler.stats;
    candidate.queued = can_bus[bus_index].tx_scheduler.queued_count;
    candidate.periodic_reserved_permille =
        can_bus[bus_index].tx_scheduler.periodic_reserved_permille;
    *stats = candidate;
    taskEXIT_CRITICAL();
    return 1u;
}

/**
 * @brief 把一帧接收数据压进本总线的环形队列
 * @note  只能在对应总线的 RX 中断里调用(单生产者的前提)。
 *        数据先写完, 再 __DMB 之后发布 head, 保证消费者看到 head 时数据已就绪。
 */
static void CANRxQueuePush(CANBusState_t *bus, CANInstance *ins, uint8_t len, const uint8_t *data)
{
    if (bus == NULL || ins == NULL)
        return;
    const uint32_t timestamp_cycles = DWT_ProbeStart();
    const uint32_t timestamp_ms = HAL_GetTick();
    if (len > 8u)
        len = 8u;

    uint16_t head = bus->rxq_head;
    uint16_t next = (uint16_t)((head + 1u) % CAN_RXQ_DEPTH);
    if (next == bus->rxq_tail)
    {
        /* 队列满: 丢弃新帧 + 计数(不覆盖最旧, 免得和消费者抢 tail) */
        bus->rxq_drop++;
        return;
    }

    CANRxItem_t *it = &bus->rxq[head];
    it->instance = ins;
    it->len = len;
    memcpy(it->data, data, len);
    it->timestamp_ms = timestamp_ms;
    it->timestamp_cycles = timestamp_cycles;
    it->generation = bus->rx_generation; // 保存入队时的总线代际，不改变原始接收时间。

    __DMB();
    bus->rxq_head = next;

    /* 诊断: 记录队列历史最大占用, 用来判断深度(32)够不够 */
    uint16_t used = (uint16_t)((uint16_t)(head - bus->rxq_tail + CAN_RXQ_DEPTH) % CAN_RXQ_DEPTH) + 1u;
    if (used > bus->rxq_peak)
        bus->rxq_peak = used;
}

// 公开调用条件见头文件；只派发接收后不超过 50 ms 的帧，按总线分别保持到达顺序。
void CANProcessRx(void)
{
    CANBusStateEnsureInitialized();
    // 检查并设置 processing 时暂时关普通中断；已有派发执行时直接返回，由另一调用处理队列。
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

    for (uint8_t i = 0; i < DEVICE_CAN_CNT; ++i)
    {
        CANBusState_t *bus = &can_bus[i];
        while (bus->rxq_tail != bus->rxq_head)
        {
            CANRxItem_t *it = &bus->rxq[bus->rxq_tail];
            CANInstance *ins = it->instance;
            if (CANBusIsActive(i) && it->generation == bus->rx_generation &&
                ins != NULL && ins->can_module_callback != NULL &&
                (uint32_t)(HAL_GetTick() - it->timestamp_ms) <= MESSAGE_DEFAULT_MAX_AGE_MS)
            {
                ins->rx_len = it->len;
                ins->rx_timestamp_ms = it->timestamp_ms;
                ins->rx_timestamp_cycles = it->timestamp_cycles;
                memcpy(ins->rx_buff, it->data, it->len);
                ins->can_module_callback(ins);
            }
            bus->rx_dispatch++;

            /* 先把数据取完再推进 tail, 否则生产者可能覆盖正在读的槽 */
            __DMB();
            bus->rxq_tail = (uint16_t)((bus->rxq_tail + 1u) % CAN_RXQ_DEPTH);
        }
    }

    DWT_ProbeDone(&can_prof_rx_dispatch, probe);
    processing = 0u;
}

uint32_t CANGetRxQueueDrop(FDCAN_HandleTypeDef *hcan)
{
    CANBusState_t *bus = CANBusFromHandle(hcan);
    return (bus != NULL) ? bus->rxq_drop : 0u;
}

/* CAN运行问题作为普通诊断提交，由日志核心统一过滤和限速。 */
static void CANLogDiagnostic(CANBusState_t *bus, const char *tag, uint32_t value)
{
    (void)tag;
    (void)value;
    if (bus == NULL)
        return;
    BSP_LOG_EVENT_2(EV_BSP_CAN_BSP_CAN_C_01,
                    BSP_LOG_FIELD_CSTR_LIMIT(tag, 48u),
                    BSP_LOG_FIELD_U32(value));
}

/* ----------------two static function called by CANRegister()-------------------- */

/**
 * @brief 添加过滤器以实现对特定id的报文的接收,会被CANRegister()调用
 *        给CAN添加过滤器后,BxCAN会根据接收到的报文的id进行消息过滤,符合规则的id会被填入FIFO触发中断
 *        对于FDCAN，设置使用特定ID模式过滤。
 *
 * @note f407的bxCAN有28个过滤器,这里将其配置为前14个过滤器给CAN1使用,后14个被CAN2使用
 *       初始化时,奇数id的模块会被分配到FIFO0,偶数id的模块会被分配到FIFO1
 *       注册到CAN1的模块使用过滤器0-13,CAN2使用过滤器14-27
 *       FDCAN的消息RAM是所有FDCAN外设共用的。
 *       H723系列FDCAN过滤器数量完全在CubeMX中自定义，因此先做一次检查，再添加即可。
 *
 * @attention 你不需要完全理解这个函数的作用,因为它主要是用于初始化,在开发过程中不需要关心底层的实现
 *            享受开发的乐趣吧!如果你真的想知道这个函数在干什么,请联系作者或自己查阅资料(请直接查阅官方的reference manual)
 *            FDCAN的教程较少，但是添加FDCAN的人已经发了一篇CSDN讲解了，可以参考一下
 *
 * @param _instance can instance owned by specific module
 */
static uint8_t CANAddFilter(CANInstance *_instance)
{
    if (_instance == NULL || _instance->can_handle == NULL)
        return 0u;
#ifdef FDCAN
    const uint8_t bus_idx = CANBusIndexFromHandle(_instance->can_handle); // 查找本端点所在总线。
    const CAN_TxIdType_t id_type = _instance->identifier_type; // 按类型选择独立过滤器区域。
    if (bus_idx == 0xFFu || (id_type != CAN_TX_ID_STANDARD && id_type != CAN_TX_ID_EXTENDED))
        return 0u; // 未知总线或类型直接失败。
    const uint32_t limit = CANFilterLimit(bus_idx, id_type); // 不能超出板级分配的硬件槽位。
    uint32_t slot = 0u; // 从最低槽位开始查找可复用空间。
    while (slot < limit && (can_filter_used[bus_idx][id_type] & (1u << slot)) != 0u) ++slot;
    if (slot >= limit) return 0u; // 没有可用槽时保留已有端点。
    FDCAN_FilterTypeDef filter = {0}; // 清零全部 HAL 参数。
    filter.FilterIndex = slot; // 本类型过滤器区域内的槽位。
    filter.FilterType = FDCAN_FILTER_DUAL; // 两个 ID 相同，相当于精确匹配。
    filter.FilterConfig = (_instance->tx_id & 1u) ? FDCAN_FILTER_TO_RXFIFO0 : FDCAN_FILTER_TO_RXFIFO1;
    filter.FilterID1 = _instance->rx_id; // 只接收声明的完整 ID。
    filter.FilterID2 = _instance->rx_id; // 第二个匹配值与第一个相同。
    filter.IdType = id_type == CAN_TX_ID_STANDARD ? FDCAN_STANDARD_ID : FDCAN_EXTENDED_ID;
    filter.IsCalibrationMsg = 0u; // 不使用专用校准消息。
    if (HAL_FDCAN_ConfigFilter(_instance->can_handle, &filter) != HAL_OK) return 0u; // 失败不占用槽。
    can_filter_used[bus_idx][id_type] |= 1u << slot; // HAL 接受后才发布槽位占用。
    _instance->filter_index = (uint8_t)slot; // 保存位置以便注销时精确关闭。
#else
    CAN_FilterTypeDef filter = {0};
    uint8_t *filter_idx = NULL;
    if (_instance->can_handle == &hcan1)
        filter_idx = &can1_filter_idx;
    else if (_instance->can_handle == &hcan2)
        filter_idx = &can2_filter_idx;
    if (filter_idx == NULL || *filter_idx >= 28u)
        return 0u;
    filter.FilterMode = CAN_FILTERMODE_IDLIST;
    filter.FilterScale = CAN_FILTERSCALE_16BIT;
    filter.FilterFIFOAssignment = (_instance->tx_id & 1u) ? CAN_RX_FIFO0 : CAN_RX_FIFO1;
    filter.SlaveStartFilterBank = 14u;
    filter.FilterIdLow = (uint16_t)(_instance->rx_id << 5);
    filter.FilterBank = *filter_idx;
    filter.FilterActivation = CAN_FILTER_ENABLE;
    if (HAL_CAN_ConfigFilter(_instance->can_handle, &filter) != HAL_OK)
        return 0u;
    (*filter_idx)++;
#endif
    return 1u;
}

// 配置接收与中断并启动一条总线；任一步失败就报告 0，必要时停回去。
static uint8_t CANBusConfigure(FDCAN_HandleTypeDef *h)
{
    if (h == NULL || h->Instance == NULL || // 未准备的控制器不能交给 HAL 访问寄存器。
        HAL_FDCAN_ConfigRxFifoOverwrite(h, FDCAN_RX_FIFO0, FDCAN_RX_FIFO_OVERWRITE) != HAL_OK ||
        HAL_FDCAN_ConfigRxFifoOverwrite(h, FDCAN_RX_FIFO1, FDCAN_RX_FIFO_OVERWRITE) != HAL_OK ||
        HAL_FDCAN_ConfigGlobalFilter(h, FDCAN_REJECT, FDCAN_REJECT, FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE) != HAL_OK)
        return 0u;
    can_diag_start_ret = (uint32_t)HAL_FDCAN_Start(h);
    can_diag_state = (uint32_t)h->State;
    if (can_diag_start_ret != (uint32_t)HAL_OK)
        return 0u;
    can_diag_act_ret = (uint32_t)HAL_FDCAN_ActivateNotification(
        h, FDCAN_RX_ACTIVE_ITS | FDCAN_TX_ACTIVE_ITS, FDCAN_TX_BUFFER_IT_MASK);
    can_diag_ie = h->Instance->IE;
    can_diag_ile = h->Instance->ILE;
    can_diag_ils = h->Instance->ILS;
    if (can_diag_act_ret != (uint32_t)HAL_OK)
    {
        (void)HAL_FDCAN_Stop(h);
        return 0u;
    }
    return 1u;
}

// 仅停止当前初始化失败的总线，其他总线的端点和过滤器保持可用。
static void CANServiceStop(uint8_t bus_index)
{
#ifdef FDCAN
    if (bus_index >= DEVICE_CAN_CNT) return; // 无效编号不改变任何总线。
    (void)HAL_FDCAN_Stop(can_bus[bus_index].handle); // 只停止本次启动的控制器。
    memset(can_filter_used[bus_index], 0, sizeof(can_filter_used[bus_index])); // 此总线尚无已发布端点。
#else
    if (bus_index == 0u) { (void)HAL_CAN_Stop(&hcan1); can1_filter_idx = 0u; }
    else if (bus_index == 1u) { (void)HAL_CAN_Stop(&hcan2); can2_filter_idx = 14u; }
    else return;
#endif
    can_service_started &= (uint8_t)~(1u << bus_index); // 只撤销本总线的可用标记。
}

// 按端点需要启动一条总线，不触碰尚未使用的控制器。
static uint8_t CANServiceInit(uint8_t bus_index)
{
    CANBusStateEnsureInitialized();
    if (bus_index >= DEVICE_CAN_CNT) return 0u; // 越界编号不能访问总线表。
    if ((can_filter_quarantined & (uint8_t)(1u << bus_index)) != 0u) return 0u; // 过滤器未恢复时禁止重新启动。
    if (CANBusIsActive(bus_index)) return 1u; // 已启动的总线不重复配置。
#ifdef FDCAN
    if (!CANBusConfigure(can_bus[bus_index].handle)) return 0u; // 本总线失败不停止其他总线。
#else
    CAN_HandleTypeDef *handle = bus_index == 0u ? &hcan1 : &hcan2;
    if (HAL_CAN_Start(handle) != HAL_OK ||
        HAL_CAN_ActivateNotification(handle, CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_RX_FIFO1_MSG_PENDING) != HAL_OK)
    {
        CANServiceStop(bus_index);
        return 0u;
    }
#endif
    can_service_started |= (uint8_t)(1u << bus_index); // 完成配置后才允许该总线收发。
    return 1u;
}

uint8_t CANStartBus(FDCAN_HandleTypeDef *hcan)
{
    const uint8_t bus_index = CANBusIndexFromHandle(hcan); // 将硬件句柄映射到固定总线编号。
    if (bus_index >= DEVICE_CAN_CNT) return 0u; // 未知句柄不得启动其他总线。
    return CANServiceInit(bus_index); // 串行启动指定总线或复用现有服务。
}

uint8_t CANBusIsQuarantined(FDCAN_HandleTypeDef *hcan)
{
    const uint8_t index = CANBusIndexFromHandle(hcan); // 根据硬件句柄查询固定总线编号。
    if (index >= DEVICE_CAN_CNT) return 1u; // 未知总线不能被当作可用。
    return (uint8_t)((can_filter_quarantined & (uint8_t)(1u << index)) != 0u);
}

/* ----------------------- two extern callable function -----------------------*/

// 逐条总线执行：切换 CAN_SELFTEST_MODE、注册测试接收者、发送并派发确认、注销、恢复普通模式。
// 重新初始化会清过滤规则，恢复后必须按业务注册表重装。
#define CAN_SELFTEST_FRAMES 8u
/* 自测 ID: 发送 ID 必须等于接收 ID, 否则测的帧会被自己的滤镜挡掉(踩过这个坑) */
#define CAN_SELFTEST_ID 0x123u
#define CAN_SELFTEST_WAIT_MS 20u
#define CAN_SELFTEST_STALL_POLLS 4096u // 连续多次计数不变时，停止依赖 DWT 的等待。
#define CAN_SELFTEST_MAX_POLLS 1000000u // 即使计数异常缓慢，轮询次数也有上限。
// 默认外部回环会发到总线并自行确认接收；此功能专用于测试，平时保持关闭。
#ifndef CAN_SELFTEST_MODE
#define CAN_SELFTEST_MODE FDCAN_MODE_EXTERNAL_LOOPBACK
#endif

volatile uint32_t can_selftest_tx_ok = 0;
volatile uint32_t can_selftest_tx_fail = 0;
volatile uint32_t can_selftest_rx_ok = 0;
volatile uint32_t can_selftest_rx_bad = 0;
volatile uint8_t can_selftest_pass_mask = 0;
volatile uint8_t can_selftest_skip_mask = 0; // 标记因端点资源不足而跳过的总线。
volatile uint8_t can_selftest_done = 0;
volatile uint8_t can_selftest_stage = 0;   /* 调试用: 卡住时能看出走到哪一步 */
volatile uint32_t can_selftest_state_err = 0; /* 自测里模式切换/启动未生效的次数 */
// 自测后保存的寄存器读数，用于检查模式、错误与接收配置。
volatile uint32_t can_selftest_test = 0;
volatile uint32_t can_selftest_psr = 0;
volatile uint32_t can_selftest_ecr = 0;
volatile uint32_t can_selftest_rxf0s = 0;
volatile uint32_t can_selftest_rxgfc = 0;
volatile uint32_t can_selftest_rxf0c = 0;

/* 把一条总线强制切到指定模式并确保真正启动:
   - HAL_FDCAN_Init() 结束后不会清 CCCR.INIT, 必须再调 HAL_FDCAN_Start();
   - HAL 内部 hfdcan->State 很容易与真实硬件状态失配(Start 只在 State==READY 时才动作,
     否则直接返回错误什么都不做), 所以这里先对齐 State, 启动后再回读 CCCR.INIT 校验。 */
uint8_t CANForceMode(FDCAN_HandleTypeDef *h, uint32_t mode, uint8_t accept_all)
{
    const uint8_t bus_index = CANBusIndexFromHandle(h); // 模式切换只接受已知控制器。
    if (bus_index >= DEVICE_CAN_CNT || (can_filter_quarantined & (uint8_t)(1u << bus_index)) != 0u)
        return 0u; // 不允许通过诊断入口绕过总线隔离。
    if (h->State == HAL_FDCAN_STATE_BUSY)
        (void)HAL_FDCAN_Stop(h);
    h->State = HAL_FDCAN_STATE_READY;
    h->Init.Mode = mode;
    if (HAL_FDCAN_Init(h) != HAL_OK)
        return 0u;

    (void)accept_all; /* 预留: 需要"不过滤接收全部"时可在此切换 */
    if (!CANBusConfigure(h))
        return 0u;
    if ((h->Instance->CCCR & FDCAN_CCCR_INIT) != 0u) /* Start 没生效, 再对齐一次 */
    {
        h->State = HAL_FDCAN_STATE_READY;
        (void)HAL_FDCAN_Start(h);
    }
    if ((h->Instance->CCCR & FDCAN_CCCR_INIT) != 0u)
        return 0u;                                   /* 仍停着: 记一次错误 */
    return 1u;
}

static CANInstance *selftest_instance = NULL;
static volatile uint8_t selftest_rx_seen = 0;
static uint8_t selftest_expect[8];
static uint8_t selftest_expect_len = 8;

// 测试实例也由 CANProcessRx() 在任务中派发；检查本次数据并更新计数。
static void CANSelfTestRxCallback(CANInstance *ins)
{
    if (ins->rx_len == selftest_expect_len &&
        memcmp(ins->rx_buff, selftest_expect, selftest_expect_len) == 0)
        can_selftest_rx_ok++;
    else
        can_selftest_rx_bad++;
    selftest_rx_seen = 1;
}

// 回环会临时注册标准 ID；资源不够时不能先重置已有总线。
static uint8_t CANSelfTestEndpointAvailable(uint8_t bus)
{
    if (bus >= DEVICE_CAN_CNT || !CANBusIsActive(bus) || idx >= CAN_MX_REGISTER_CNT)
        return 0u; // 仅已启动总线且有全局端点名额时可进入诊断。
    const FDCAN_HandleTypeDef *handle = can_bus[bus].handle; // 本条总线使用自己的登记表。
    for (uint8_t i = 0u; i < idx; ++i)
        if (can_instance[i]->can_handle == handle && can_instance[i]->identifier_type == CAN_TX_ID_STANDARD &&
            can_instance[i]->rx_id == CAN_SELFTEST_ID)
            return 0u; // 测试 ID 已被业务端点使用时不能覆盖。
    const uint32_t limit = CANFilterLimit(bus, CAN_TX_ID_STANDARD); // 查询本控制器的标准过滤器容量。
    for (uint32_t slot = 0u; slot < limit; ++slot)
        if ((can_filter_used[bus][CAN_TX_ID_STANDARD] & (1u << slot)) == 0u)
            return 1u; // 至少有一个确认空闲的标准槽。
    return 0u; // 标准槽全部占用，保留现有总线状态。
}

// 尝试关闭已注销端点的硬件过滤器；失败时保留占用，避免覆盖仍在使用的槽。
static uint8_t CANReleaseFilter(CANInstance *instance, uint8_t bus_index)
{
    if (instance == NULL || bus_index >= DEVICE_CAN_CNT) return 0u; // 无效实例不修改槽位表。
    const CAN_TxIdType_t id_type = instance->identifier_type; // 标准与扩展过滤器分别管理。
    if (id_type != CAN_TX_ID_STANDARD && id_type != CAN_TX_ID_EXTENDED) return 0u; // 无效类型不能索引表。
    const uint8_t slot = instance->filter_index; // 使用注册时保存的准确槽位。
    if (slot >= CAN_MX_REGISTER_CNT || (can_filter_used[bus_index][id_type] & (1u << slot)) == 0u)
        return 0u; // 未持有槽时不能关闭其他端点。
    FDCAN_FilterTypeDef filter = {0}; // 清零配置，避免携带未使用字段。
    filter.FilterIndex = slot; // 只关闭本端点的过滤器。
    filter.FilterType = FDCAN_FILTER_DUAL; // 与原有精确匹配使用相同类型。
    filter.IdType = id_type == CAN_TX_ID_STANDARD ? FDCAN_STANDARD_ID : FDCAN_EXTENDED_ID;
    filter.FilterConfig = FDCAN_FILTER_DISABLE; // 关闭硬件匹配，避免继续接收旧 ID。
    if (HAL_FDCAN_ConfigFilter(instance->can_handle, &filter) != HAL_OK)
        return 0u; // 硬件未确认关闭时保留占用并上报失败。
    can_filter_used[bus_index][id_type] &= ~(1u << slot); // 硬件确认后才允许复用。
    return 1u;
}

/* 仅在接收任务不并发时注销；清除中断已排队的旧指针后再释放。 */
uint8_t CANUnregisterInstance(CANInstance *ins)
{
    uint8_t i;
    if (ins == NULL)
        return 0u;
    for (i = 0; i < idx; ++i)
    {
        if (can_instance[i] == ins)
            break;
    }
    if (i >= idx)
        return 0u;

    const uint32_t previous_mask = __get_PRIMASK(); // 保存调用前中断屏蔽状态。
    __disable_irq(); // 防止摘除查找表时接收中断继续写入旧实例。
    uint8_t bus_idx = CANBusIndexFromHandle(ins->can_handle); // 定位实例所属总线。
    if (bus_idx != 0xFFu)
    {
        uint16_t slot = (uint16_t)(ins->rx_id & (CAN_RX_LOOKUP_SIZE - 1u)); // 找到接收查找项。
        if (rx_lookup[bus_idx][slot] == ins)
            rx_lookup[bus_idx][slot] = NULL; // 先阻止中断再次取得旧实例。
        CANBusState_t *bus = &can_bus[bus_idx]; // 检查该总线已排队但尚未派发的帧。
        for (uint16_t pending = bus->rxq_tail; pending != bus->rxq_head;
             pending = (uint16_t)((pending + 1u) % CAN_RXQ_DEPTH))
        {
            if (bus->rxq[pending].instance == ins)
                bus->rxq[pending].instance = NULL; // 保留队列顺序，仅撤销旧实例引用。
        }
    }
    for (; i + 1u < idx; ++i)
        can_instance[i] = can_instance[i + 1u]; // 压紧注册表，不改变其他实例。
    can_instance[--idx] = NULL; // 清除末尾的重复位置。
    __set_PRIMASK(previous_mask); // 恢复原来的屏蔽状态后再释放内存。
    const uint8_t filter_closed = CANReleaseFilter(ins, bus_idx); // 只有硬件确认后才回收槽位。
    free(ins); // 接收查找表和队列均已不再引用此实例。
    return filter_closed; // 即使关闭失败，软件实例也已释放。
}

void CANQuarantineBus(FDCAN_HandleTypeDef *handle)
{
    const uint8_t bus_index = CANBusIndexFromHandle(handle); // 只隔离指定控制器。
    if (bus_index >= DEVICE_CAN_CNT) return; // 未知句柄不能影响其他总线。
    const uint32_t saved = __get_PRIMASK(); // 保留调用前的中断状态。
    __disable_irq(); // 先撤销服务状态及待派发的旧反馈。
    can_filter_quarantined |= (uint8_t)(1u << bus_index); // 禁止后续注册隐式重启。
    can_service_started &= (uint8_t)~(1u << bus_index); // 新提交和调度不再使用此总线。
    // 保留单消费者队列指针；派发任务会丢弃隔离总线的旧反馈。
    if (handle->Instance != NULL) handle->Instance->IE = 0u; // 即使停止失败也禁止此控制器继续触发中断。
    __set_PRIMASK(saved); // 恢复原中断状态后再调用 HAL。
    if (handle->Instance != NULL) (void)HAL_FDCAN_Stop(handle); // 尽力停止硬件；失败时软件仍保持隔离。
    taskENTER_CRITICAL(); // 与发送任务对队列和结果的访问互斥。
    CANTxScheduler_t *scheduler = &can_bus[bus_index].tx_scheduler; // 仅处理受影响的调度器。
    CANTxCancelQueued(scheduler); // 撤销尚在软件队列中的旧请求。
    for (uint8_t i = 0u; i < CAN_TX_RESULT_HISTORY; ++i)
        if (scheduler->results[i].used != 0u &&
            (scheduler->results[i].state == CAN_TX_RESULT_QUEUED ||
             scheduler->results[i].state == CAN_TX_RESULT_ACTIVE ||
             scheduler->results[i].state == CAN_TX_RESULT_IN_FLIGHT))
            CANTxSetTerminal(scheduler, i, CAN_TX_RESULT_CANCELLED); // 已交给硬件的结果也撤销，但保留未完成帧计数。
    taskEXIT_CRITICAL(); // 其他总线的队列和结果保持不变。
}

uint8_t CANReapplyFilters(FDCAN_HandleTypeDef *handle)
{
    const uint8_t bus_index = CANBusIndexFromHandle(handle); // 获取待恢复的单条总线。
    if (bus_index >= DEVICE_CAN_CNT || !CANBusIsActive(bus_index)) return 0u; // 无效或隔离中的总线不能重建。
    memset(can_filter_used[bus_index], 0, sizeof(can_filter_used[bus_index])); // 仅重建已经重新初始化的控制器。
    for (uint8_t i = 0u; i < idx; ++i)
        if (can_instance[i]->can_handle == handle && !CANAddFilter(can_instance[i]))
        {
            CANQuarantineBus(handle); // 任何重装失败都阻止本总线继续运行。
            return 0u; // 调用方必须上报失败。
        }
    return 1u;
}

uint8_t CANRunLoopbackSelfTest(void)
{
    CANBusStateEnsureInitialized();
    uint8_t passed = 0;

    can_selftest_tx_ok = 0;
    can_selftest_tx_fail = 0;
    can_selftest_rx_ok = 0;
    can_selftest_rx_bad = 0;
    can_selftest_pass_mask = 0;
    can_selftest_skip_mask = 0u; // 清除上次自测的资源跳过记录。
    can_selftest_done = 0;
    can_selftest_stage = 0;
    can_selftest_test = can_selftest_psr = can_selftest_ecr = 0;
    can_selftest_rxf0s = can_selftest_rxgfc = can_selftest_rxf0c = 0;

    for (uint8_t bus = 0; bus < DEVICE_CAN_CNT; ++bus)
    {
        FDCAN_HandleTypeDef *h = can_bus[bus].handle;
        uint8_t bus_ok = 1;
        uint8_t tx_uncertain = 0u; // 测试帧仍可能在硬件中时禁止重新启用该总线。

        if (!CANSelfTestEndpointAvailable(bus))
        {
            can_selftest_skip_mask |= (uint8_t)(1u << bus); // 保存本条总线没有临时端点资源。
            ++can_selftest_tx_fail; // 诊断未发送，不能被当作通过。
            continue; // 不切换模式、不改变已注册设备或其他总线。
        }

        /* 1. 切到回环模式 */
        can_selftest_stage = (uint8_t)(10u * (bus + 1u) + 1u);
        if (CANForceMode(h, CAN_SELFTEST_MODE, 1u) == 0u)
        {
            can_selftest_state_err++;
            if (!CANForceMode(h, FDCAN_MODE_NORMAL, 0u)) CANQuarantineBus(h); // 无法恢复模式时保持隔离。
            else if (!CANReapplyFilters(h)) can_selftest_state_err++; // 恢复过滤器失败也需报告。
            continue;
        }

        /* 2. 注册测试实例(会加滤镜 + 进 rx_lookup) */
        can_selftest_stage = (uint8_t)(10u * (bus + 1u) + 2u);
        CAN_Init_Config_s test_cfg = {
            .can_handle = h,
            .tx_id = CAN_SELFTEST_ID,
            .rx_id = CAN_SELFTEST_ID,
            .can_module_callback = CANSelfTestRxCallback,
            .id = NULL,
        };
        selftest_instance = CANRegister(&test_cfg);
        if (selftest_instance == NULL)
        {
            can_selftest_tx_fail++;
            if (!CANForceMode(h, FDCAN_MODE_NORMAL, 0u)) CANQuarantineBus(h); // 模式失败不得继续使用。
            else if (!CANReapplyFilters(h)) can_selftest_state_err++; // 单总线过滤器失败单独记录。
            continue;
        }

        /* 3. 发 N 帧并等待回环接收 */
        for (uint8_t n = 0; n < CAN_SELFTEST_FRAMES; ++n)
        {
            for (uint8_t k = 0; k < 8u; ++k)
            {
                uint8_t v = (uint8_t)(n * 16u + k);
                selftest_expect[k] = v;
            }
            selftest_expect_len = 8;
            selftest_rx_seen = 0;
            const CAN_TxFrame_t frame = {
                .identifier = CAN_SELFTEST_ID,
                .identifier_type = CAN_TX_ID_STANDARD,
                .data = selftest_expect,
                .length = 8u,
            };
            uint32_t request_id = 0u;
            if (CANSubmitOneShot(h, selftest_instance, &frame,
                                 CAN_TX_PRIORITY_URGENT, CAN_SELFTEST_WAIT_MS,
                                 &request_id) != CAN_TX_SUBMIT_ACCEPTED)
            {
                can_selftest_tx_fail++;
                bus_ok = 0;
                continue;
            }

            /* 注意: 调度器启动前 TIM23 时间基准尚未正常递增, HAL_GetTick() 不会走,
               所以这里必须用 DWT 周期计数器计时, 否则会死等(TIM23 的 uwTick 停住)。 */
            uint32_t t0 = DWT_ProbeStart();
            uint8_t counted_accept = 0u;
            uint8_t tx_complete = 0u; // 接收回环帧后仍需等待硬件发送完成通知。
            uint32_t previous_cycle = t0; // 上一次 DWT 计数，用于发现时基停止。
            uint32_t stagnant_polls = 0u; // 统计连续未前进的读取次数。
            uint32_t total_polls = 0u; // 限制偶发前进但不可靠的时基造成的等待。
            uint8_t clock_failed = 0u; // 保存本帧是否因时基异常提前结束。
            while (!(selftest_rx_seen && tx_complete) &&
                   DWT_ProbeElapsedUs(t0) < (uint32_t)CAN_SELFTEST_WAIT_MS * 1000u)
            {
                /* 接收改成"中断入队 + 任务派发"之后, 回调不再在中断里执行,
                   这里必须主动消费队列, 否则永远等不到 selftest_rx_seen。 */
                CANProcessTxBus(h); // 只推进本条诊断总线，避免改动其他总线的发送队列。
                CANProcessRx();
                CAN_TxResult_t tx_result;
                memset(&tx_result, 0, sizeof(tx_result));
                if (CANGetTxRequestState(request_id, &tx_result) != 0u &&
                    tx_result.accepted_frames > 0u && counted_accept == 0u)
                {
                    can_selftest_tx_ok++;
                    counted_accept = 1u;
                }
                if (tx_result.state == CAN_TX_RESULT_COMPLETE && tx_result.pending_frames == 0u)
                    tx_complete = 1u; // 收到硬件完成通知才允许安全切换模式。
                if (tx_result.state == CAN_TX_RESULT_FAILED ||
                    tx_result.state == CAN_TX_RESULT_EXPIRED)
                    break;
                const uint32_t cycle = DWT_ProbeStart(); // 每轮读取原始周期数，允许自然回绕。
                if (cycle == previous_cycle) ++stagnant_polls; // 连续相等说明时基可能停止。
                else stagnant_polls = 0u; // 确实前进后重新累计停滞次数。
                previous_cycle = cycle; // 保存本轮读数供下次比较。
                ++total_polls; // 无论时基是否前进都计入总上限。
                if (stagnant_polls >= CAN_SELFTEST_STALL_POLLS || total_polls >= CAN_SELFTEST_MAX_POLLS)
                {
                    clock_failed = 1u; // 时基不可信时不能继续依赖它等待。
                    break; // 留给本总线的恢复路径处理。
                }
            }
            CAN_TxResult_t final_tx = {0}; // 离开等待后重新检查本次请求的最终状态。
            if (!CANGetTxRequestState(request_id, &final_tx))
                tx_uncertain = 1u; // 无法确认请求状态时不能安全重启控制器。
            else if (final_tx.pending_frames != 0u || final_tx.state == CAN_TX_RESULT_IN_FLIGHT)
                tx_uncertain = 1u; // 已交给硬件的帧尚未确认完成或中止。
            else if ((final_tx.state == CAN_TX_RESULT_QUEUED || final_tx.state == CAN_TX_RESULT_ACTIVE) &&
                     !CANCancelTxRequest(request_id))
                tx_uncertain = 1u; // 未送出的请求必须确认从软件队列撤销。
            if (tx_uncertain)
            {
                ++can_selftest_state_err; // 保留发送状态不确定的诊断结果。
                bus_ok = 0u; // 不把该总线视为通过。
                break; // 立即停止产生更多测试帧。
            }
            if (clock_failed)
            {
                ++can_selftest_state_err; // 保留时基异常造成的自测失败。
                bus_ok = 0u; // 本条总线不能报告自测通过。
                break; // 不再发送剩余测试帧，立即恢复正常模式。
            }
            if (!selftest_rx_seen || !tx_complete)
                bus_ok = 0;

            // 保存第一条总线的模式及错误寄存器，供测试结束后读取。
            if (bus == 0u)
            {
                can_selftest_test = h->Instance->TEST;
                can_selftest_psr = h->Instance->PSR;
                can_selftest_ecr = h->Instance->ECR;
                can_selftest_rxf0s = h->Instance->RXF0S;
                can_selftest_rxgfc = h->Instance->GFC;
                can_selftest_rxf0c = h->Instance->RXF0C;
            }
        }

        if (can_selftest_rx_ok > 0u && can_selftest_rx_bad == 0u && bus_ok)
            can_selftest_pass_mask |= (uint8_t)(1u << bus);

        /* 4. 恢复 NORMAL 模式并重装滤镜 */
        if (tx_uncertain) CANQuarantineBus(h); // 先禁止本总线服务，避免重启后重用未确认的硬件发送状态。
        if (!CANUnregisterInstance(selftest_instance))
        {
            ++can_selftest_state_err; // 记录自测过滤器未能确认关闭。
            can_selftest_pass_mask &= (uint8_t)~(1u << bus); // 本次自测不能报告成功。
        }
        selftest_instance = NULL;
        if (tx_uncertain) continue; // 隔离状态交给整车依赖检查，不尝试隐式恢复。
        if (CANForceMode(h, FDCAN_MODE_NORMAL, 0u) == 0u)
        {
            can_selftest_state_err++;
            can_selftest_pass_mask &= (uint8_t)~(1u << bus); // 恢复失败不能算自测成功。
            CANQuarantineBus(h); // 停止使用恢复失败的总线。
            BSP_LOG_EVENT_1(EV_BSP_CAN_BSP_CAN_C_02, BSP_LOG_FIELD_U32((unsigned)(bus + 1)));
        }
        else if (!CANReapplyFilters(h))
        {
            can_selftest_state_err++; // 保留过滤器重装失败。
            can_selftest_pass_mask &= (uint8_t)~(1u << bus); // 失败不能报告该总线通过。
        }
    }

    can_selftest_done = 1;
    for (uint8_t i = 0; i < DEVICE_CAN_CNT; ++i)
    {
        if (can_selftest_pass_mask & (1u << i))
            passed++;
    }
    BSP_LOG_EVENT_6(EV_BSP_CAN_BSP_CAN_C_03, BSP_LOG_FIELD_U32((unsigned)passed), BSP_LOG_FIELD_U32((unsigned)DEVICE_CAN_CNT), BSP_LOG_FIELD_U32((unsigned long)can_selftest_tx_ok), BSP_LOG_FIELD_U32((unsigned long)can_selftest_rx_ok), BSP_LOG_FIELD_U32((unsigned long)can_selftest_rx_bad), BSP_LOG_FIELD_U32((unsigned long)can_selftest_tx_fail));
    return passed;
}

CANInstance *CANRegister(CAN_Init_Config_s *config)
{
    if (config == NULL || config->can_handle == NULL || config->can_module_callback == NULL)
        return NULL;
    if (config->identifier_type != CAN_TX_ID_STANDARD && config->identifier_type != CAN_TX_ID_EXTENDED)
        return NULL; // 无效类型不能映射到硬件过滤器。
    const uint32_t id_limit = config->identifier_type == CAN_TX_ID_STANDARD ? 0x7ffu : 0x1fffffffu;
    if (config->tx_id > id_limit || config->rx_id > id_limit)
        return NULL;
#ifdef FDCAN
    uint8_t bus_idx = CANBusIndexFromHandle(config->can_handle);
    if (bus_idx == 0xFFu)
        return NULL;
    CANTxSchedulerEnsureInitialized(bus_idx);
    uint16_t lookup_idx = (uint16_t)(config->rx_id & (CAN_RX_LOOKUP_SIZE - 1u));
#else
    if (config->can_handle != &hcan1 && config->can_handle != &hcan2)
        return NULL;
#endif
    if (idx >= CAN_MX_REGISTER_CNT)
        return NULL;
    for (uint8_t i = 0u; i < idx; ++i)
    {
        if (can_instance[i] != NULL && can_instance[i]->can_handle == config->can_handle &&
            can_instance[i]->rx_id == config->rx_id &&
            can_instance[i]->identifier_type == config->identifier_type)
            return NULL;
    }

    CANInstance *instance = (CANInstance *)malloc(sizeof(*instance));
    if (instance == NULL)
        return NULL;
    memset(instance, 0, sizeof(*instance));
#ifdef FDCAN
    instance->txconf.Identifier = config->tx_id;
    instance->txconf.IdType = config->identifier_type == CAN_TX_ID_STANDARD ? FDCAN_STANDARD_ID : FDCAN_EXTENDED_ID;
    instance->txconf.TxFrameType = FDCAN_DATA_FRAME;
    instance->txconf.DataLength = FDCAN_DLC_BYTES_8;
    instance->txconf.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    instance->txconf.BitRateSwitch = FDCAN_BRS_OFF;
    instance->txconf.FDFormat = FDCAN_CLASSIC_CAN;
    instance->txconf.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    instance->txconf.MessageMarker = 0u;
#else
    instance->txconf.StdId = config->tx_id;
    instance->txconf.IDE = CAN_ID_STD;
    instance->txconf.RTR = CAN_RTR_DATA;
    instance->txconf.DLC = 0x08u;
#endif
    instance->can_handle = config->can_handle;
    instance->tx_id = config->tx_id;
    instance->rx_id = config->rx_id;
    instance->identifier_type = config->identifier_type; // 收发和派发使用相同的 ID 类型。
    instance->can_module_callback = config->can_module_callback;
    instance->id = config->id;

    // 第一次注册才启动服务；随后添加过滤规则失败时，清理本次分配和本次启动。
    uint8_t started_here = (uint8_t)!CANBusIsActive(bus_idx); // 记录是否由本次注册启动。
    if (started_here && !CANServiceInit(bus_idx))
    {
        free(instance);
        return NULL;
    }
    if (!CANAddFilter(instance))
    {
        if (started_here)
            CANServiceStop(bus_idx); // 首次注册失败只撤销本次启动的总线。
        free(instance);
        return NULL;
    }

    const uint32_t previous_mask = __get_PRIMASK(); // 保存调用前中断状态。
    __disable_irq(); // 防止接收中断在登记表发布到一半时读取它。
    can_instance[idx] = instance; // 先写入完整实例地址。
#ifdef FDCAN
    rx_lookup[bus_idx][lookup_idx] = instance; // 快速查找项和登记表一起发布。
#endif
    ++idx; // 最后扩大中断可遍历的有效范围。
    __set_PRIMASK(previous_mask); // 恢复调用前状态，不在关中断时配置硬件。
    return instance;
}

/* 发送完成相关回调(中断上下文: 只做计数, 不打印不阻塞) */
void HAL_FDCAN_TxFifoEmptyCallback(FDCAN_HandleTypeDef *hfdcan)
{
    if (!can_bus_state_initialized) return; // 早期中断不能重入正在建立的工作区。
    CANBusState_t *bus = CANBusFromHandle(hfdcan);
    if (bus != NULL)
        bus->tx.tx_fifo_empty_events++;
}

void HAL_FDCAN_TxBufferCompleteCallback(FDCAN_HandleTypeDef *hfdcan, uint32_t BufferIndexes)
{
    if (!can_bus_state_initialized) return; // 工作区未发布时不读取发送状态。
    CANBusState_t *bus = CANBusFromHandle(hfdcan);
    if (bus == NULL)
        return;

    CANTxScheduler_t *scheduler = &bus->tx_scheduler;
    for (uint8_t hw_index = 0u; hw_index < CAN_TX_MAX_HW_BUFFERS; ++hw_index)
    {
        if ((BufferIndexes & (1u << hw_index)) == 0u)
            continue;
        const uint8_t result_index = scheduler->hw_result[hw_index];
        scheduler->hw_result[hw_index] = CAN_TX_INVALID_RESULT_INDEX;
        if (result_index >= CAN_TX_RESULT_HISTORY)
            continue;

        CANTxResultSlot_t *result = &scheduler->results[result_index];
        if (result->used == 0u)
            continue;
        if (result->pending_hw > 0u)
            result->pending_hw--;
        result->completed_frames++;
        bus->tx.tx_done++;
        if (!CANTxStateTerminal(result->state) &&
            result->accepted_frames == result->frame_count && result->pending_hw == 0u &&
            result->completed_frames == result->frame_count)
            CANTxSetTerminal(scheduler, result_index, CAN_TX_RESULT_COMPLETE);
    }
}

void HAL_FDCAN_TxBufferAbortCallback(FDCAN_HandleTypeDef *hfdcan, uint32_t BufferIndexes)
{
    if (!can_bus_state_initialized) return; // 工作区未发布时不读取撤销状态。
    CANBusState_t *bus = CANBusFromHandle(hfdcan);
    if (bus == NULL)
        return;

    CANTxScheduler_t *scheduler = &bus->tx_scheduler;
    for (uint8_t hw_index = 0u; hw_index < CAN_TX_MAX_HW_BUFFERS; ++hw_index)
    {
        if ((BufferIndexes & (1u << hw_index)) == 0u)
            continue;
        const uint8_t result_index = scheduler->hw_result[hw_index];
        scheduler->hw_result[hw_index] = CAN_TX_INVALID_RESULT_INDEX;
        if (result_index >= CAN_TX_RESULT_HISTORY)
            continue;

        CANTxResultSlot_t *result = &scheduler->results[result_index];
        if (result->used == 0u)
            continue;
        if (result->pending_hw > 0u)
            result->pending_hw--;
        bus->tx.tx_error++;
        result->last_hal_error = (uint32_t)HAL_ERROR;
        // HAL曾接受该帧；中止后不重发，避免重复执行一次性命令。
        CANTxSetTerminal(scheduler, result_index, CAN_TX_RESULT_FAILED);
    }
}

void CANGetTxStats(FDCAN_HandleTypeDef *hcan, CAN_TxStats_t *stats)
{
    CANBusState_t *bus = CANBusFromHandle(hcan);
    if (bus == NULL || stats == NULL)
        return;
    *stats = bus->tx;
}

void CANGetBusStats(FDCAN_HandleTypeDef *hcan, CAN_BusStats_t *stats)
{
    CANBusState_t *bus = CANBusFromHandle(hcan);
    CAN_Status_t st;
    if (bus == NULL || stats == NULL)
        return;
    CANGetStatus(hcan, &st);
    bus->bus.status = st.status;
    bus->bus.tx_error_count = st.tx_error_count;
    bus->bus.rx_error_count = st.rx_error_count;
    *stats = bus->bus;
}

uint8_t CANGetObservationCounters(uint8_t bus_index, CAN_ObservationCounters_t *counters)
{
    if (bus_index >= DEVICE_CAN_CNT || counters == NULL)
        return 0u;

    CANBusStateEnsureInitialized();
    CANBusState_t *bus = &can_bus[bus_index];
    CAN_ObservationCounters_t candidate;
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    candidate.active = CANBusIsActive(bus_index); // 每条总线分别报告实际启动状态。
    candidate.tx_ok = bus->tx.tx_ok;
    candidate.tx_drop = bus->tx.tx_drop;
    candidate.tx_error = bus->tx.tx_error;
    candidate.tx_done = bus->tx.tx_done;
    candidate.tx_fifo_empty_events = bus->tx.tx_fifo_empty_events;
    candidate.busoff = bus->bus.busoff;
    candidate.busoff_recover = bus->bus.busoff_recover;
    candidate.rx_lost = bus->bus.rx_lost;
    candidate.rx_frames = bus->bus.rx_frames;
    candidate.rx_queue_drop = bus->rxq_drop;
    candidate.rx_queue_peak = bus->rxq_peak;
    candidate.tx_error_count = bus->bus.tx_error_count;
    candidate.rx_error_count = bus->bus.rx_error_count;
    candidate.status = (uint32_t)bus->bus.status;
    if (primask == 0u)
        __enable_irq();
    *counters = candidate;
    return 1u;
}

// 控制器已进入 INIT 时有界丢弃两个硬件 FIFO 的旧帧，不向模块派发。
static uint8_t CANDiscardStoppedRx(FDCAN_HandleTypeDef *handle)
{
    if (handle == NULL || handle->Instance == NULL ||
        (handle->Instance->CCCR & FDCAN_CCCR_INIT) == 0u)
        return 0u; // 不能证明停止接收时，不能认定 FIFO 已清空。
    const uint32_t fifos[] = {FDCAN_RX_FIFO0, FDCAN_RX_FIFO1}; // 两个硬件 FIFO 都要处理。
    for (uint8_t index = 0u; index < 2u; ++index)
    {
        const uint32_t fifo = fifos[index]; // 选择本轮清理的 FIFO。
        for (uint8_t count = 0u; count < 64u; ++count)
        {
            if (HAL_FDCAN_GetRxFifoFillLevel(handle, fifo) == 0u) break; // 已空时转向下一个 FIFO。
            if (handle->State != HAL_FDCAN_STATE_BUSY) return 0u; // HAL 只允许 BUSY 状态读取接收帧。
            FDCAN_RxHeaderTypeDef header = {0}; // 旧帧头只用于满足 HAL 输出要求。
            uint8_t data[64] = {0}; // HAL 可能复制最长 CAN FD 负载，不能只预留 8 字节。
            if (HAL_FDCAN_GetRxMessage(handle, fifo, &header, data) != HAL_OK)
                return 0u; // 读取失败时旧帧可能仍在，禁止重启。
        }
        if (HAL_FDCAN_GetRxFifoFillLevel(handle, fifo) != 0u)
            return 0u; // 超过硬件最大容量仍未清空，不能无限等待或放行。
    }
    return 1u; // 两个 FIFO 均确认为空。
}

/**
 * @brief CAN 总线健康检查 + BusOff 恢复
 *        空总线(没有从机应答)会让 TEC 迅速累计到 256 进入 BusOff, 而 BusOff 后
 *        外设停止收发、TX FIFO 永远不会空 —— 必须由软件把它拉回来。
 *        建议在 100Hz 的 daemon 任务里以 10Hz 调用。
 */
void CANHealthMonitor(void)
{
    CANBusStateEnsureInitialized();
    static uint32_t last_ms = 0;
    uint32_t now = HAL_GetTick();
    if ((uint32_t)(now - last_ms) < 100u)
        return;
    last_ms = now;

#ifdef FDCAN
    for (uint8_t i = 0; i < DEVICE_CAN_CNT; ++i)
    {
        if (!CANBusIsActive(i)) continue; // 未启用的控制器不能执行健康查询或恢复。
        FDCAN_ProtocolStatusTypeDef ps;
        FDCAN_ErrorCountersTypeDef ec;
        CANBusState_t *bus = &can_bus[i];

        if (HAL_FDCAN_GetProtocolStatus(bus->handle, &ps) != HAL_OK)
            continue;
        if (HAL_FDCAN_GetErrorCounters(bus->handle, &ec) == HAL_OK)
        {
            bus->bus.tx_error_count = (uint8_t)ec.TxErrorCnt;
            bus->bus.rx_error_count = (uint8_t)ec.RxErrorCnt;
        }

        /* 两种情况都要救: ① BusOff(错误计数超限) ② CCCR.INIT=1(外设停在初始化状态,
           例如某个流程只 Stop 没 Start)。后者不主动检查的话, 总线会静默失联。 */
        if (ps.BusOff || ((bus->handle->Instance->CCCR & FDCAN_CCCR_INIT) != 0u))
        {
            if (ps.BusOff)
                bus->bus.busoff++;
            taskENTER_CRITICAL(); // 与发送队列提交和推进互斥。
            can_recovering |= (uint8_t)(1u << i); // 先暂停本总线，禁止恢复期间接收新请求。
            ++bus->rx_generation; // 恢复后丢弃故障前已经进入软件队列的反馈。
            CANTxCancelQueued(&bus->tx_scheduler); // 控制器重启前撤销已经排队但未提交的旧帧。
            uint8_t hardware_pending = bus->tx_ownership_uncertain; // 无法关联槽位时同样视为有未确认帧。
            for (uint8_t slot = 0u; slot < CAN_TX_MAX_HW_BUFFERS; ++slot)
                if (bus->tx_scheduler.hw_result[slot] != CAN_TX_INVALID_RESULT_INDEX)
                    hardware_pending = 1u; // 任一已交给 HAL 的槽尚未报告完成或中止。
            taskEXIT_CRITICAL(); // 保留其他总线的队列；已交给硬件的帧仍需单独处置。
            if (hardware_pending != 0u)
            {
                CANQuarantineBus(bus->handle); // 尚不能确认硬件帧已取消，不能重新启动总线。
                continue;
            }
            if (!CANDiscardStoppedRx(bus->handle))
            {
                CANQuarantineBus(bus->handle); // 无法确认旧硬件接收帧已清除时保持隔离。
                continue;
            }
            /* 对齐 HAL 的 State(它可能与真实硬件不一致), 否则 Start 会直接返回错误 */
            if (bus->handle->State == HAL_FDCAN_STATE_BUSY && HAL_FDCAN_Stop(bus->handle) != HAL_OK)
            {
                CANQuarantineBus(bus->handle); // 停止失败后不能重新放行。
                continue;
            }
            bus->handle->State = HAL_FDCAN_STATE_READY;
            if (HAL_FDCAN_Start(bus->handle) != HAL_OK ||
                HAL_FDCAN_ActivateNotification(bus->handle, FDCAN_RX_ACTIVE_ITS | FDCAN_TX_ACTIVE_ITS,
                                               FDCAN_TX_BUFFER_IT_MASK) != HAL_OK)
            {
                CANQuarantineBus(bus->handle); // 重启或中断恢复失败时保持隔离。
                continue;
            }
            taskENTER_CRITICAL(); // 与提交和发送的复核使用同一保护。
            can_recovering &= (uint8_t)~(1u << i); // 控制器启动后允许新请求；整车许可仍未接入。
            taskEXIT_CRITICAL();
            bus->bus.busoff_recover++;
            CANLogDiagnostic(bus, "CAN总线恢复尝试", bus->bus.busoff);
        }

        /* 接收侧的异常在这里限速打印(中断里只计数):
           ① rx_lost: 硬件 RX FIFO 溢出丢帧(总线太忙);
           ② rxq_drop: 软件接收队列满(任务消费不及时, 队列深度不够或任务被阻塞)。
           两者都是"帧真的丢了", 一旦出现就该查, 所以放在这里统一告警。 */
        if (bus->bus.rx_lost != 0u)
            CANLogDiagnostic(bus, "CAN硬件接收溢出", bus->bus.rx_lost);
        if (bus->rxq_drop != 0u)
            CANLogDiagnostic(bus, "CAN接收队列满", bus->rxq_drop);
    }

    /* 接收队列诊断汇总(供 OpenOCD/LCD 读取):
       drop 应恒为 0; peak 用来判断队列深度(32)够不够。 */
    uint32_t drop_sum = 0u;
    uint16_t peak_max = 0u;
    for (uint8_t i = 0; i < DEVICE_CAN_CNT; ++i)
    {
        drop_sum += can_bus[i].rxq_drop;
        if (can_bus[i].rxq_peak > peak_max)
            peak_max = can_bus[i].rxq_peak;
    }
    can_rxq_drop_total = drop_sum;
    can_rxq_peak_max = peak_max;
#endif
}

void CANGetStatus(FDCAN_HandleTypeDef *hcan, CAN_Status_t *status)
{
    if (hcan == NULL || status == NULL)
        return;

    FDCAN_ProtocolStatusTypeDef protocol_status;
    FDCAN_ErrorCountersTypeDef error_counters;

    status->status = CAN_STATUS_UNKNOWN;
    status->rx_error_count = 0;
    status->tx_error_count = 0;

    if (!CANBusIsActive(CANBusIndexFromHandle(hcan)))
        return; // 未初始化的控制器不能读取 HAL 状态，保持 UNKNOWN。

    if (HAL_FDCAN_GetProtocolStatus(hcan, &protocol_status) != HAL_OK)
        return;
    if (HAL_FDCAN_GetErrorCounters(hcan, &error_counters) != HAL_OK)
        return;

    status->rx_error_count = (uint8_t)error_counters.RxErrorCnt;
    status->tx_error_count = (uint8_t)error_counters.TxErrorCnt;

    if (protocol_status.BusOff)
        status->status = CAN_STATUS_BUSOFF;
    else if (protocol_status.ErrorPassive || error_counters.RxErrorCnt > 0 || error_counters.TxErrorCnt > 0)
        status->status = CAN_STATUS_ERROR;
    else
        status->status = CAN_STATUS_OK;
}

/* -----------------------belows are callback definitions--------------------------*/

//对于FDCAN，回调函数和处理方式完全不同，因此直接用两套逻辑处理
#ifdef FDCAN
// 两个 FDCAN FIFO 共用的中断处理：取完所有帧，查出对应实例后只入队，不调用模块解析。
static void FDCANFIFOxCallback(FDCAN_HandleTypeDef *_hfdcan, uint32_t fifox)
{
    FDCAN_RxHeaderTypeDef rxconf = {0}; // 各次中断使用独立帧头，避免嵌套时覆盖。
    uint8_t fdcan_rx_buff[64] = {0}; // HAL 先复制数据再返回帧头，预留 FD 最大长度以便安全丢弃。
    uint32_t probe_start = DWT_ProbeStart();
    CANBusState_t *bus = CANBusFromHandle(_hfdcan);
    uint8_t bus_idx = CANBusIndexFromHandle(_hfdcan);
    if (!CANBusIsActive(bus_idx)) return; // 隔离后不再接收或派发该总线的旧帧。

    while (HAL_FDCAN_GetRxFifoFillLevel(_hfdcan, fifox)) // FIFO不为空,有可能在其他中断时有多帧数据进入
    {
        if (HAL_FDCAN_GetRxMessage(_hfdcan, fifox, &rxconf, fdcan_rx_buff) != HAL_OK)
            break; // 读取失败可能不会移除 FIFO 条目，必须结束本轮中断。
        can_prof_rx_frames++;
        if (bus != NULL)
            bus->bus.rx_frames++;
        uint32_t data_length = (rxconf.DataLength >> 16u) & 0xFu; // 取 HAL 提供的经典帧 DLC。
        if (data_length > 8u || rxconf.FDFormat != FDCAN_CLASSIC_CAN)
            continue; // 非法长度及 FD 帧不能伪装成零长度经典帧。
        if (rxconf.RxFrameType == FDCAN_DATA_FRAME &&
            (rxconf.IdType == FDCAN_STANDARD_ID || rxconf.IdType == FDCAN_EXTENDED_ID))
        {
            const CAN_TxIdType_t id_type = rxconf.IdType == FDCAN_STANDARD_ID ?
                CAN_TX_ID_STANDARD : CAN_TX_ID_EXTENDED; // 硬件标记转换为框架中的类型。
        	/* 常见路径: 用 rx_id 低 9 位查表, O(1) 找到归属实例 */
        	CANInstance *target = NULL;
        	if (bus_idx != 0xFFu)
        	{
        		CANInstance *cand = rx_lookup[bus_idx][rxconf.Identifier & (CAN_RX_LOOKUP_SIZE - 1u)];
                if (cand != NULL && cand->rx_id == rxconf.Identifier && cand->identifier_type == id_type)
        			target = cand;
        	}
        	if (target == NULL)
        	{
        		/* 回退路径: 低位别名冲突时退回线性查找(极少发生) */
        		for (size_t i = 0; i < idx; ++i)
        		{
                    if (_hfdcan == can_instance[i]->can_handle && rxconf.Identifier == can_instance[i]->rx_id &&
                    can_instance[i]->identifier_type == id_type)
        			{
        				target = can_instance[i];
        				break;
        			}
        		}
        	}

        	if (target != NULL && target->can_module_callback != NULL)
        	{
        		/* 只入队, 不在中断里解析(BSPCAN-02):
        		   解析(含滤波/多圈累加)延后到 CANProcessRx() 在任务上下文逐帧执行,
        		   既让中断最短, 又消除了"中断写 measure / 任务读 measure"的数据竞争。 */
                CANRxQueuePush(bus, target, (uint8_t)data_length, fdcan_rx_buff);
        	}
        	/* 注意: 这里不能 return! 必须把 FIFO 里的帧全部取完, 否则 FIFO 满就会丢帧 */
        }
    }
    DWT_ProbeDone(&can_prof_rx_isr, probe_start);
}


void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    if (!can_bus_state_initialized) return; // 工作区未发布时不读取实例表或队列。
	/* 检查Rx FIFO 0中是否有消息丢失 */
	if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_MESSAGE_LOST) != 0)
	{
		CANBusState_t *bus = CANBusFromHandle(hfdcan);
		if (bus != NULL)
		{
			bus->bus.rx_lost++;
			/* 这里只计数, 不在中断里打日志(写 RTT 是长耗时操作);
			   实际打印由 CANHealthMonitor() 在任务上下文限速完成。 */
		}
	}
	/* 检查是否有新消息写入Rx FIFO 0或到达一定阈值 */
	if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE)||(RxFifo0ITs & FDCAN_IT_RX_FIFO0_FULL)||(RxFifo0ITs & FDCAN_IT_RX_FIFO0_WATERMARK))
	{
		FDCANFIFOxCallback(hfdcan, FDCAN_RX_FIFO0); // 调用我们自己写的函数来处理消息
	}
}
void HAL_FDCAN_RxFifo1Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo1ITs)
{
    if (!can_bus_state_initialized) return; // 工作区未发布时不读取实例表或队列。
	/* 检查Rx FIFO 1中是否有消息丢失 */
	if ((RxFifo1ITs & FDCAN_IT_RX_FIFO1_MESSAGE_LOST) != 0)
	{
		CANBusState_t *bus = CANBusFromHandle(hfdcan);
		if (bus != NULL)
		{
			bus->bus.rx_lost++;
			/* 同上: 只计数, 打印交给 CANHealthMonitor() */
		}
	}
	/* 检查是否有新消息写入Rx FIFO 1或到达一定阈值 */
	if ((RxFifo1ITs & FDCAN_IT_RX_FIFO1_NEW_MESSAGE)||(RxFifo1ITs & FDCAN_IT_RX_FIFO1_FULL)||(RxFifo1ITs & FDCAN_IT_RX_FIFO1_WATERMARK))
	{
		FDCANFIFOxCallback(hfdcan, FDCAN_RX_FIFO1); // 调用我们自己写的函数来处理消息
	}
}


#else


// 保留的 BXCAN 路径：收到消息后逐项匹配，在中断里直接调用模块回调；与当前 FDCAN 队列路径不同。
static void CANFIFOxCallback(CAN_HandleTypeDef *_hcan, uint32_t fifox)
{
   static CAN_RxHeaderTypeDef rxconf; // 同上
   uint8_t can_rx_buff[8];
   while (HAL_CAN_GetRxFifoFillLevel(_hcan, fifox)) // FIFO不为空,有可能在其他中断时有多帧数据进入
   {
       HAL_CAN_GetRxMessage(_hcan, fifox, &rxconf, can_rx_buff); // 从FIFO中获取数据
       for (size_t i = 0; i < idx; ++i)
       { // 两者相等说明这是要找的实例
           if (_hcan == can_instance[i]->can_handle && rxconf.StdId == can_instance[i]->rx_id)
           {
               if (can_instance[i]->can_module_callback != NULL) // 回调函数不为空就调用
               {
                   can_instance[i]->rx_len = rxconf.DLC;                      // 保存接收到的数据长度
                   memcpy(can_instance[i]->rx_buff, can_rx_buff, rxconf.DLC); // 消息拷贝到对应实例
                   can_instance[i]->can_module_callback(can_instance[i]);     // 触发回调进行数据解析和处理
               }
               return;
           }
       }
   }
}

// 下面两个 BXCAN 回调在对应 FIFO 有待取消息时调用，不必等到 FIFO 满。
// 下面的函数会调用CANFIFOxCallback()来进一步处理来自特定CAN设备的消息

/** @brief BXCAN FIFO0 有待取消息时进入，继续交给共用处理函数。
* @param[in] hcan 来源 CAN 句柄。
 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
   CANFIFOxCallback(hcan, CAN_RX_FIFO0); // 调用我们自己写的函数来处理消息
}

/** @brief BXCAN FIFO1 有待取消息时进入，继续交给共用处理函数。
* @param[in] hcan 来源 CAN 句柄。
 */
void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
   CANFIFOxCallback(hcan, CAN_RX_FIFO1); // 调用我们自己写的函数来处理消息
}


#endif
