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

// 注册表用于 ID 查找、过滤规则重装及注销；中断不直接运行模块解析。
static CANInstance *can_instance[CAN_MX_REGISTER_CNT] = {NULL};
static uint8_t idx;
static uint8_t can_service_started;
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
} CANRxItem_t;

/* 接收中断只保留"有新帧"和"丢帧诊断"两种:
   FULL / WATERMARK 与 NEW_MESSAGE 触发的是同一个处理函数, 全开等于同一事件多次进中断 */
#define FDCAN_RX_ACTIVE_ITS                                            \
    (FDCAN_IT_RX_FIFO0_NEW_MESSAGE | FDCAN_IT_RX_FIFO0_MESSAGE_LOST |  \
     FDCAN_IT_RX_FIFO1_NEW_MESSAGE | FDCAN_IT_RX_FIFO1_MESSAGE_LOST)
/* 发送完成中断要按 TX 缓冲位掩码使能; 必须与 fdcan.c 的 TxFifoQueueElmtsNbr(16) 一致 */
#define FDCAN_TX_BUFFER_IT_MASK 0xFFFFu

typedef struct
{
    FDCAN_HandleTypeDef *handle;
    CAN_TxStats_t tx;
    CAN_BusStats_t bus;
    LogRateLimit_t log_rl; /* 限速日志状态(见 bsp_log.h) */
    LogRateLimit_t log_rl_rx; /* 接收侧日志单独限速, 免得被发送侧告警压掉 */

    /* ---- 接收队列(SPSC): 中断入队, 任务用 CANProcessRx() 消费 ---- */
    CANRxItem_t rxq[CAN_RXQ_DEPTH];
    volatile uint16_t rxq_head;    /* 生产者(中断)写 */
    volatile uint16_t rxq_tail;    /* 消费者(任务)写 */
    volatile uint32_t rxq_drop;    /* 队列满丢弃帧数 */
    volatile uint16_t rxq_peak;    /* 历史最大占用(诊断: 判断深度够不够) */
    volatile uint32_t rx_dispatch; /* 已派发给回调的帧数 */
} CANBusState_t;

static CANBusState_t can_bus[DEVICE_CAN_CNT] = {
    {.handle = &hfdcan1}, {.handle = &hfdcan2}, {.handle = &hfdcan3}};
static CANInstance *rx_lookup[DEVICE_CAN_CNT][CAN_RX_LOOKUP_SIZE];
/* 每条总线已用的标准滤镜下标(自测重装滤镜时需要整体重置) */
static uint8_t can_filter_idx[DEVICE_CAN_CNT] = {0};

/* 诊断: 最近一次总线配置的结果(HAL 返回码/状态/中断寄存器), 用于排查"中断不触发" */
volatile uint32_t can_diag_start_ret = 0;
volatile uint32_t can_diag_state = 0;
volatile uint32_t can_diag_act_ret = 0;
volatile uint32_t can_diag_ie = 0;
volatile uint32_t can_diag_ile = 0;
volatile uint32_t can_diag_ils = 0;

/* 每条总线允许的标准滤镜数量(来自 CubeMX 配置的 StdFiltersNbr) */
static uint32_t CANFilterLimit(uint8_t bus_idx)
{
    switch (bus_idx)
    {
    case 0:
        return hfdcan1.Init.StdFiltersNbr;
    case 1:
        return hfdcan2.Init.StdFiltersNbr;
    case 2:
        return hfdcan3.Init.StdFiltersNbr;
    default:
        return 0;
    }
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
    for (uint8_t i = 0; i < DEVICE_CAN_CNT; ++i)
    {
        if (can_bus[i].handle == _handle)
            return i;
    }
    return 0xFFu;
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
    it->timestamp_ms = HAL_GetTick();

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
            if (ins != NULL && ins->can_module_callback != NULL &&
                (uint32_t)(HAL_GetTick() - it->timestamp_ms) <= MESSAGE_DEFAULT_MAX_AGE_MS)
            {
                ins->rx_len = it->len;
                ins->rx_timestamp_ms = it->timestamp_ms;
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

/* 限速日志: 同一类消息最多 1 条/秒, 其余只计数(避免日志把 CPU 和 RTT 缓冲吃光)
   限速逻辑统一用 bsp_log 的 LogRateLimitAllow(), 与其它模块保持一致 */
static void CANLogRateLimitedEx(CANBusState_t *bus, LogRateLimit_t *rl, const char *tag, uint32_t value)
{
    if (bus == NULL || rl == NULL)
        return;
    if (LogRateLimitAllow(rl, 1000u))
    {
        LOGWARNING("[bsp_can] %s (值 %lu, 累计 %lu 次, 期间限速 %lu 条)",
                   tag, (unsigned long)value,
                   (unsigned long)rl->total, (unsigned long)rl->dropped);
    }
}

/* 默认用总线的"发送侧"限速状态(保持既有调用点不变) */
static void CANLogRateLimited(CANBusState_t *bus, const char *tag, uint32_t value)
{
    if (bus == NULL)
        return;
    CANLogRateLimitedEx(bus, &bus->log_rl, tag, value);
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
    if (_instance == NULL || _instance->can_handle == NULL || _instance->rx_id > 0x7ffu)
        return 0u;
#ifdef FDCAN
    uint8_t bus_idx = CANBusIndexFromHandle(_instance->can_handle);
    if (bus_idx == 0xFFu || can_filter_idx[bus_idx] >= CANFilterLimit(bus_idx))
        return 0u;
    FDCAN_FilterTypeDef filter = {0};
    filter.FilterIndex = can_filter_idx[bus_idx];
    filter.FilterType = FDCAN_FILTER_DUAL;
    filter.FilterConfig = (_instance->tx_id & 1u) ? FDCAN_FILTER_TO_RXFIFO0 : FDCAN_FILTER_TO_RXFIFO1;
    filter.FilterID1 = _instance->rx_id;
    filter.FilterID2 = _instance->rx_id;
    filter.IdType = FDCAN_STANDARD_ID;
    filter.IsCalibrationMsg = 0u;
    if (HAL_FDCAN_ConfigFilter(_instance->can_handle, &filter) != HAL_OK)
        return 0u;
    can_filter_idx[bus_idx]++;
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
    if (h == NULL ||
        HAL_FDCAN_ConfigRxFifoOverwrite(h, FDCAN_RX_FIFO0, FDCAN_RX_FIFO_OVERWRITE) != HAL_OK ||
        HAL_FDCAN_ConfigRxFifoOverwrite(h, FDCAN_RX_FIFO1, FDCAN_RX_FIFO_OVERWRITE) != HAL_OK ||
        HAL_FDCAN_ConfigGlobalFilter(h, FDCAN_REJECT, FDCAN_REJECT, FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE) != HAL_OK)
        return 0u;
    can_diag_start_ret = (uint32_t)HAL_FDCAN_Start(h);
    can_diag_state = (uint32_t)h->State;
    if (can_diag_start_ret != (uint32_t)HAL_OK)
        return 0u;
    can_diag_act_ret = (uint32_t)HAL_FDCAN_ActivateNotification(h, FDCAN_RX_ACTIVE_ITS, 0u);
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

// 启动某条总线失败时统一停止已启动部分，重置过滤规则编号。
static void CANServiceStop(void)
{
#ifdef FDCAN
    for (uint8_t i = 0; i < DEVICE_CAN_CNT; ++i)
        (void)HAL_FDCAN_Stop(can_bus[i].handle);
    memset(can_filter_idx, 0, sizeof(can_filter_idx));
#else
    (void)HAL_CAN_Stop(&hcan1);
    (void)HAL_CAN_Stop(&hcan2);
    can1_filter_idx = 0u;
    can2_filter_idx = 14u;
#endif
    can_service_started = 0u;
}

// 依次启动各总线，一条失败便停止本轮全部服务，不留下部分启动状态。
static uint8_t CANServiceInit(void)
{
#ifdef FDCAN
    for (uint8_t i = 0; i < DEVICE_CAN_CNT; ++i)
    {
        if (!CANBusConfigure(can_bus[i].handle))
        {
            CANServiceStop();
            return 0u;
        }
    }
#else
    if (HAL_CAN_Start(&hcan1) != HAL_OK ||
        HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_RX_FIFO1_MSG_PENDING) != HAL_OK ||
        HAL_CAN_Start(&hcan2) != HAL_OK ||
        HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_RX_FIFO1_MSG_PENDING) != HAL_OK)
    {
        CANServiceStop();
        return 0u;
    }
#endif
    can_service_started = 1u;
    return 1u;
}

/* ----------------------- two extern callable function -----------------------*/

// 逐条总线执行：切换 CAN_SELFTEST_MODE、注册测试接收者、发送并派发确认、注销、恢复普通模式。
// 重新初始化会清过滤规则，恢复后必须按业务注册表重装。
#define CAN_SELFTEST_FRAMES 8u
/* 自测 ID: 发送 ID 必须等于接收 ID, 否则测的帧会被自己的滤镜挡掉(踩过这个坑) */
#define CAN_SELFTEST_ID 0x123u
#define CAN_SELFTEST_WAIT_MS 20u
// 默认外部回环会发到总线并自行确认接收；此功能专用于测试，平时保持关闭。
#ifndef CAN_SELFTEST_MODE
#define CAN_SELFTEST_MODE FDCAN_MODE_EXTERNAL_LOOPBACK
#endif

volatile uint32_t can_selftest_tx_ok = 0;
volatile uint32_t can_selftest_tx_fail = 0;
volatile uint32_t can_selftest_rx_ok = 0;
volatile uint32_t can_selftest_rx_bad = 0;
volatile uint8_t can_selftest_pass_mask = 0;
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
    if (h == NULL)
        return 0u;
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

/* 注销一个实例: 从 can_instance[] 与 rx_lookup 里摘掉并释放内存 */
void CANUnregisterInstance(CANInstance *ins)
{
    uint8_t i;
    if (ins == NULL)
        return;
    for (i = 0; i < idx; ++i)
    {
        if (can_instance[i] == ins)
            break;
    }
    if (i >= idx)
        return;

    uint8_t bus_idx = CANBusIndexFromHandle(ins->can_handle);
    if (bus_idx != 0xFFu)
    {
        uint16_t slot = (uint16_t)(ins->rx_id & (CAN_RX_LOOKUP_SIZE - 1u));
        if (rx_lookup[bus_idx][slot] == ins)
            rx_lookup[bus_idx][slot] = NULL;
    }
    free(ins);
    for (; i + 1u < idx; ++i)
        can_instance[i] = can_instance[i + 1u];
    can_instance[--idx] = NULL;
}

/* 重装所有已注册实例的滤镜(重新 Init 之后必须调用) */
void CANReapplyFilters(void)
{
    for (uint8_t i = 0; i < DEVICE_CAN_CNT; ++i)
        can_filter_idx[i] = 0;
    for (uint8_t i = 0; i < idx; ++i)
        CANAddFilter(can_instance[i]);
}

uint8_t CANRunLoopbackSelfTest(void)
{
    uint8_t passed = 0;

    can_selftest_tx_ok = 0;
    can_selftest_tx_fail = 0;
    can_selftest_rx_ok = 0;
    can_selftest_rx_bad = 0;
    can_selftest_pass_mask = 0;
    can_selftest_done = 0;
    can_selftest_stage = 0;
    can_selftest_test = can_selftest_psr = can_selftest_ecr = 0;
    can_selftest_rxf0s = can_selftest_rxgfc = can_selftest_rxf0c = 0;

    for (uint8_t bus = 0; bus < DEVICE_CAN_CNT; ++bus)
    {
        FDCAN_HandleTypeDef *h = can_bus[bus].handle;
        uint8_t bus_ok = 1;

        /* 1. 切到回环模式 */
        can_selftest_stage = (uint8_t)(10u * (bus + 1u) + 1u);
        if (CANForceMode(h, CAN_SELFTEST_MODE, 1u) == 0u)
        {
            can_selftest_state_err++;
            (void)CANForceMode(h, FDCAN_MODE_NORMAL, 0u);
            CANReapplyFilters();
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

        /* 3. 发 N 帧并等待回环接收 */
        for (uint8_t n = 0; n < CAN_SELFTEST_FRAMES; ++n)
        {
            for (uint8_t k = 0; k < 8u; ++k)
            {
                uint8_t v = (uint8_t)(n * 16u + k);
                selftest_expect[k] = v;
                selftest_instance->tx_buff[k] = v;
            }
            selftest_expect_len = 8;
            selftest_rx_seen = 0;

            if (CANTransmit(selftest_instance, 0.0f) != 1u)
            {
                can_selftest_tx_fail++;
                bus_ok = 0;
                continue;
            }
            can_selftest_tx_ok++;

            /* 注意: 调度器启动前 TIM23 时间基准尚未正常递增, HAL_GetTick() 不会走,
               所以这里必须用 DWT 周期计数器计时, 否则会死等(TIM23 的 uwTick 停住)。 */
            uint32_t t0 = DWT_ProbeStart();
            while (!selftest_rx_seen &&
                   DWT_ProbeElapsedUs(t0) < (uint32_t)CAN_SELFTEST_WAIT_MS * 1000u)
            {
                /* 接收改成"中断入队 + 任务派发"之后, 回调不再在中断里执行,
                   这里必须主动消费队列, 否则永远等不到 selftest_rx_seen。 */
                CANProcessRx();
            }
            if (!selftest_rx_seen)
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
        CANUnregisterInstance(selftest_instance);
        selftest_instance = NULL;
        if (CANForceMode(h, FDCAN_MODE_NORMAL, 0u) == 0u)
        {
            can_selftest_state_err++;
            LOGERROR("[bsp_can] 自测后恢复 NORMAL 模式失败 (bus %u)", (unsigned)(bus + 1));
        }
        CANReapplyFilters();
    }

    can_selftest_done = 1;
    for (uint8_t i = 0; i < DEVICE_CAN_CNT; ++i)
    {
        if (can_selftest_pass_mask & (1u << i))
            passed++;
    }
    LOGINFO("[bsp_can] 内部回环自测: 通过 %u/%u 条总线, tx_ok=%lu rx_ok=%lu rx_bad=%lu tx_fail=%lu",
             (unsigned)passed, (unsigned)DEVICE_CAN_CNT,
             (unsigned long)can_selftest_tx_ok, (unsigned long)can_selftest_rx_ok,
             (unsigned long)can_selftest_rx_bad, (unsigned long)can_selftest_tx_fail);
    return passed;
}

CANInstance *CANRegister(CAN_Init_Config_s *config)
{
    if (config == NULL || config->can_handle == NULL ||
        config->can_module_callback == NULL || config->tx_id > 0x7ffu || config->rx_id > 0x7ffu)
        return NULL;
#ifdef FDCAN
    uint8_t bus_idx = CANBusIndexFromHandle(config->can_handle);
    if (bus_idx == 0xFFu)
        return NULL;
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
            can_instance[i]->rx_id == config->rx_id)
            return NULL;
    }

    CANInstance *instance = (CANInstance *)malloc(sizeof(*instance));
    if (instance == NULL)
        return NULL;
    memset(instance, 0, sizeof(*instance));
#ifdef FDCAN
    instance->txconf.Identifier = config->tx_id;
    instance->txconf.IdType = FDCAN_STANDARD_ID;
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
    instance->can_module_callback = config->can_module_callback;
    instance->id = config->id;

    // 第一次注册才启动服务；随后添加过滤规则失败时，清理本次分配和本次启动。
    uint8_t started_here = (uint8_t)!can_service_started;
    if (started_here && !CANServiceInit())
    {
        free(instance);
        return NULL;
    }
    if (!CANAddFilter(instance))
    {
        if (started_here)
            CANServiceStop();
        free(instance);
        return NULL;
    }

    can_instance[idx++] = instance;
#ifdef FDCAN
    rx_lookup[bus_idx][lookup_idx] = instance;
#endif
    return instance;
}

/* @todo 目前似乎封装过度,应该添加一个指向tx_buff的指针,tx_buff不应该由CAN instance保存 */
/* 如果让CANinstance保存txbuff,会增加一次复制的开销 */
uint8_t CANTransmit(CANInstance *_instance, float timeout)
{
    (void)timeout; /* 保留参数以免改动所有调用者; 现在是非阻塞语义, 不再等待 */
    uint32_t probe_start = DWT_ProbeStart();
    CANBusState_t *bus = CANBusFromHandle(_instance->can_handle);

#ifdef FDCAN
    if (HAL_FDCAN_GetTxFifoFreeLevel(_instance->can_handle) == 0u)
    {
        // 发送空间满就立即返回，避免等待拖延整轮控制；模块自行决定下一轮是否再次发送。
        if (bus != NULL)
        {
            bus->tx.tx_drop++;
            can_prof_tx_full = bus->tx.tx_drop;
            CANLogRateLimited(bus, "TX FIFO 满, 丢帧(下一周期重发)", bus->tx.tx_drop);
        }
        DWT_ProbeDone(&can_prof_tx, probe_start);
        return 0;
    }
    if (HAL_FDCAN_AddMessageToTxFifoQ(_instance->can_handle, &_instance->txconf, _instance->tx_buff) != HAL_OK)
    {
        if (bus != NULL)
        {
            bus->tx.tx_error++;
            CANLogRateLimited(bus, "AddMessageToTxFifoQ 失败", bus->tx.tx_error);
        }
        DWT_ProbeDone(&can_prof_tx, probe_start);
        return 0;
    }
#else
    if (HAL_CAN_GetTxMailboxesFreeLevel(_instance->can_handle) == 0u)
    {
        if (bus != NULL)
            bus->tx.tx_drop++;
        DWT_ProbeDone(&can_prof_tx, probe_start);
        return 0;
    }
    if (HAL_CAN_AddTxMessage(_instance->can_handle, &_instance->txconf, _instance->tx_buff, &_instance->tx_mailbox) != HAL_OK)
    {
        if (bus != NULL)
            bus->tx.tx_error++;
        DWT_ProbeDone(&can_prof_tx, probe_start);
        return 0;
    }
#endif

    if (bus != NULL)
        bus->tx.tx_ok++;
    DWT_ProbeDone(&can_prof_tx, probe_start);
    return 1; // 已成功交给硬件
}

/* 发送完成相关回调(中断上下文: 只做计数, 不打印不阻塞) */
void HAL_FDCAN_TxFifoEmptyCallback(FDCAN_HandleTypeDef *hfdcan)
{
    CANBusState_t *bus = CANBusFromHandle(hfdcan);
    if (bus != NULL)
        bus->tx.tx_fifo_empty_events++;
}

void HAL_FDCAN_TxBufferCompleteCallback(FDCAN_HandleTypeDef *hfdcan, uint32_t BufferIndexes)
{
    (void)BufferIndexes; /* TXBTO 是累计标志, 用 popcount 统计会重复累加, 这里只统计事件次数 */
    CANBusState_t *bus = CANBusFromHandle(hfdcan);
    if (bus != NULL)
        bus->tx.tx_done++;
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

/**
 * @brief CAN 总线健康检查 + BusOff 恢复
 *        空总线(没有从机应答)会让 TEC 迅速累计到 256 进入 BusOff, 而 BusOff 后
 *        外设停止收发、TX FIFO 永远不会空 —— 必须由软件把它拉回来。
 *        建议在 100Hz 的 daemon 任务里以 10Hz 调用。
 */
void CANHealthMonitor(void)
{
    static uint32_t last_ms = 0;
    uint32_t now = HAL_GetTick();
    if ((uint32_t)(now - last_ms) < 100u)
        return;
    last_ms = now;

#ifdef FDCAN
    for (uint8_t i = 0; i < DEVICE_CAN_CNT; ++i)
    {
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
            /* 对齐 HAL 的 State(它可能与真实硬件不一致), 否则 Start 会直接返回错误 */
            if (bus->handle->State == HAL_FDCAN_STATE_BUSY)
                (void)HAL_FDCAN_Stop(bus->handle);
            bus->handle->State = HAL_FDCAN_STATE_READY;
            (void)HAL_FDCAN_Start(bus->handle);  /* 清 INIT, 重新上线 */
            HAL_FDCAN_ActivateNotification(bus->handle, FDCAN_RX_ACTIVE_ITS, 0);
            bus->bus.busoff_recover++;
            CANLogRateLimited(bus, "BusOff/INIT 检测到并尝试恢复", bus->bus.busoff);
        }

        /* 接收侧的异常在这里限速打印(中断里只计数):
           ① rx_lost: 硬件 RX FIFO 溢出丢帧(总线太忙);
           ② rxq_drop: 软件接收队列满(任务消费不及时, 队列深度不够或任务被阻塞)。
           两者都是"帧真的丢了", 一旦出现就该查, 所以放在这里统一告警。 */
        if (bus->bus.rx_lost != 0u)
            CANLogRateLimitedEx(bus, &bus->log_rl_rx, "RX FIFO 溢出丢帧", bus->bus.rx_lost);
        if (bus->rxq_drop != 0u)
            CANLogRateLimitedEx(bus, &bus->log_rl_rx, "接收队列满丢帧(任务消费不及时)", bus->rxq_drop);
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

void CANSetDLC(CANInstance *_instance, uint8_t length)
{
    // 长度 0 改成 1，超过 8 改成 8；记录错误并继续，不把整个控制任务留在等待中。
    static uint32_t dlc_err_cnt = 0;
    static uint32_t dlc_log_ms = 0;

    if (_instance == NULL)
        return;

    if (length > 8u || length == 0u)
    {
        dlc_err_cnt++;
        uint32_t now = HAL_GetTick();
        if ((uint32_t)(now - dlc_log_ms) >= 1000u)
        {
            dlc_log_ms = now;
            LOGWARNING("[bsp_can] DLC 参数非法(%u), 已钳位为 1..8; 累计 %lu 次(检查调用方或野指针)",
                       (unsigned)length, (unsigned long)dlc_err_cnt);
        }
        if (length > 8u)
            length = 8u;
        else
            length = 1u;
    }

    _instance->txconf.DataLength = DLC_LookUp_Table[length];
}

/* -----------------------belows are callback definitions--------------------------*/

//对于FDCAN，回调函数和处理方式完全不同，因此直接用两套逻辑处理
#ifdef FDCAN
// 两个 FDCAN FIFO 共用的中断处理：取完所有帧，查出对应实例后只入队，不调用模块解析。
static void FDCANFIFOxCallback(FDCAN_HandleTypeDef *_hfdcan, uint32_t fifox)
{
    static FDCAN_RxHeaderTypeDef rxconf; // 同上
	static uint16_t DataLength = 0;
    static uint8_t fdcan_rx_buff[8];
    uint32_t probe_start = DWT_ProbeStart();
    CANBusState_t *bus = CANBusFromHandle(_hfdcan);
    uint8_t bus_idx = CANBusIndexFromHandle(_hfdcan);

    while (HAL_FDCAN_GetRxFifoFillLevel(_hfdcan, fifox)) // FIFO不为空,有可能在其他中断时有多帧数据进入
    {
        HAL_FDCAN_GetRxMessage(_hfdcan, fifox, &rxconf, fdcan_rx_buff); // 从FIFO中获取数据
        can_prof_rx_frames++;
        if (bus != NULL)
            bus->bus.rx_frames++;
		//解析数据长度，@Todo 此处在用新版本重新生成后可能得修改，DataLength可能不需要右移，具体情况具体看	！
		if(((rxconf.DataLength >> 16) & 0xF)>=0 && ((rxconf.DataLength >> 16) & 0xF)<=8)
		{
			DataLength=(rxconf.DataLength >> 16) & 0xF; // 保存接收到的数据长度
		}
		else
		{
			DataLength=0;
		}
        if(rxconf.RxFrameType==FDCAN_DATA_FRAME && rxconf.IdType==FDCAN_STANDARD_ID)
        {
        	/* 常见路径: 用 rx_id 低 9 位查表, O(1) 找到归属实例 */
        	CANInstance *target = NULL;
        	if (bus_idx != 0xFFu)
        	{
        		CANInstance *cand = rx_lookup[bus_idx][rxconf.Identifier & (CAN_RX_LOOKUP_SIZE - 1u)];
        		if (cand != NULL && cand->rx_id == rxconf.Identifier)
        			target = cand;
        	}
        	if (target == NULL)
        	{
        		/* 回退路径: 低位别名冲突时退回线性查找(极少发生) */
        		for (size_t i = 0; i < idx; ++i)
        		{
        			if (_hfdcan == can_instance[i]->can_handle && rxconf.Identifier == can_instance[i]->rx_id)
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
        		CANRxQueuePush(bus, target, (uint8_t)DataLength, fdcan_rx_buff);
        	}
        	/* 注意: 这里不能 return! 必须把 FIFO 里的帧全部取完, 否则 FIFO 满就会丢帧 */
        }
    }
    DWT_ProbeDone(&can_prof_rx_isr, probe_start);
}


void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
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
