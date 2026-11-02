/** @file bsp_can.h
* @brief H723 经典 CAN 实例注册、发送、接收派发和总线状态查询。
* 接收中断只入队，模块解析由任务调用 CANProcessRx() 完成；每帧最多 8 字节。
 */
#ifndef BSP_CAN_H
#define BSP_CAN_H

//在此选择CAN类型，两者只能选择一个！！！
#define FDCAN //G系列和H7系列使用FDCAN
//#define BXCAN //F系列使用BxCAN

//CAN类型宏定义检查，有错误停止编译
#if !defined(FDCAN) && !defined(BXCAN)
    #error "Neither FDCAN nor BXCAN is defined. Please define one of them."
#elif defined(FDCAN) && defined(BXCAN)
    #error "Both FDCAN and BXCAN are defined. Please define only one."
#endif


#include <stdint.h>
#include <stddef.h>
#ifdef FDCAN
#include "fdcan.h"
#define hcan1  hfdcan1
#define hcan2  hfdcan2
#define hcan3  hfdcan3

#define CAN_MX_REGISTER_CNT 16     // 这个数量取决于CAN总线的负载
#define MX_CAN_FILTER_CNT (3 * 14) // 最多可以使用的CAN过滤器数量,目前远不会用到这么多
#define DEVICE_CAN_CNT 3           //H723VG有3个FDCAN

#endif

// 默认关闭回环自测。实际模式由 bsp_can.c 的 CAN_SELFTEST_MODE 决定，默认外部回环，
// 会短暂切换总线并发出测试帧；不能把它当作不影响外部设备的普通状态查询。
#ifndef CAN_SELFTEST_LOOPBACK
#define CAN_SELFTEST_LOOPBACK 0   /* 置 1 = 启动后在 daemon 任务里跑一次 FDCAN 回环自测 */
#endif
#ifdef BXCAN
#include "can.h"
// 最多能够支持的CAN设备数
#define CAN_MX_REGISTER_CNT 16     // 这个数量取决于CAN总线的负载
#define MX_CAN_FILTER_CNT (2 * 14) // 最多可以使用的CAN过滤器数量,目前远不会用到这么多
#define DEVICE_CAN_CNT 2           // 根据板子设定,F407IG有CAN1,CAN2,因此为2;F334只有一个,则设为1
// 如果只有1个CAN,还需要把bsp_can.c中所有的hcan2变量改为hcan1(别担心,主要是总线和FIFO的负载均衡,不影响功能)
#endif

// 定义查找表
static const uint32_t DLC_LookUp_Table[9] = {
    FDCAN_DLC_BYTES_0,
    FDCAN_DLC_BYTES_1,  
    FDCAN_DLC_BYTES_2,  
    FDCAN_DLC_BYTES_3,
    FDCAN_DLC_BYTES_4,
    FDCAN_DLC_BYTES_5,
    FDCAN_DLC_BYTES_6,
    FDCAN_DLC_BYTES_7,
    FDCAN_DLC_BYTES_8
};



/* can instance typedef, every module registered to CAN should have this variable */
// 保持结构体及数组元素按自然边界存放，HAL 会把其中数据当作 32 位读取；
// 不要在外层套用 pack(1)，否则某些数组元素地址可能无法按 32 位访问。
typedef struct _
{
#ifdef FDCAN
    FDCAN_HandleTypeDef  *can_handle; // can句柄
    FDCAN_TxHeaderTypeDef txconf;    // CAN报文发送配置
#else
    CAN_HandleTypeDef *can_handle; // can句柄
    CAN_TxHeaderTypeDef txconf;    // CAN报文发送配置
#endif
    uint32_t tx_id;                // 发送id
    uint32_t tx_mailbox;           // CAN消息填入的邮箱号
    uint8_t tx_buff[8];            // 模块编码暂存区；新发送接口提交时复制数据，不保留此指针
    uint8_t rx_buff[8];            // 接收缓存,最大消息长度为8
    uint32_t rx_id;                // 接收id
    uint8_t rx_len;                // 接收长度,可能为0-8
    uint32_t rx_timestamp_ms; // 帧进入接收队列时的 HAL 毫秒时间，用于新鲜度检查。
    uint32_t rx_timestamp_cycles; // 同一时刻的 DWT 周期计数，供需要亚毫秒精度的解析器计算相邻帧间隔。
    // 接收的回调函数,用于解析接收到的数据
    void (*can_module_callback)(struct _ *); // callback needs an instance to tell among registered ones
    void *id;                                // 使用can外设的模块指针(即id指向的模块拥有此can实例,是父子关系)
} CANInstance;

// 编译时检查实例大小和两个字段是否为 4 字节的倍数，避免 HAL 的 32 位访问出错。
_Static_assert((sizeof(CANInstance) % 4u) == 0u, "CANInstance 大小必须是 4 的倍数");
_Static_assert((__builtin_offsetof(CANInstance, txconf) % 4u) == 0u, "txconf 必须 4 字节对齐");
_Static_assert((__builtin_offsetof(CANInstance, tx_buff) % 4u) == 0u, "tx_buff 必须 4 字节对齐");

/* CAN实例初始化结构体,将此结构体指针传入注册函数 */
typedef struct
{
#ifdef FDCAN
    FDCAN_HandleTypeDef  *can_handle;           // can句柄
#else
    CAN_HandleTypeDef *can_handle;              // can句柄
#endif
    uint32_t tx_id;                             // 发送id
    uint32_t rx_id;                             // 接收id
    void (*can_module_callback)(CANInstance *); // 处理接收数据的回调函数
    void *id;                                   // 拥有can实例的模块地址,用于区分不同的模块(如果有需要的话),如果不需要可以不传入
} CAN_Init_Config_s;

typedef enum
{
    CAN_STATUS_OK = 0,
    CAN_STATUS_ERROR,
    CAN_STATUS_BUSOFF,
    CAN_STATUS_UNKNOWN,
} CAN_Status_e;

typedef struct
{
    CAN_Status_e status;
    uint8_t rx_error_count;
    uint8_t tx_error_count;
} CAN_Status_t;

/* CAN 发送统计(每条总线一份): 用于观察"丢帧换实时性"的实际代价 */
typedef struct
{
    uint32_t tx_ok;               /* 成功写入硬件 TX FIFO 的帧数 */
    uint32_t tx_drop;             /* 发送空间满而被丢弃的帧数，是否再次发送由调用方决定。 */
    uint32_t tx_error;            /* HAL 返回错误的次数 */
    uint32_t tx_done;             /* 发送完成回调次数，一次通知可能包含多帧。 */
    uint32_t tx_fifo_empty_events;/* TX FIFO 变空事件次数 */
} CAN_TxStats_t;

#include "can_tx.h"

typedef struct
{
    uint32_t submitted;
    uint32_t coalesced;
    uint32_t rejected;
    uint32_t bandwidth_rejected;
    uint32_t retries;
    uint32_t fifo_busy;
    uint32_t hal_rejected;
    uint32_t completed;
    uint32_t failed;
    uint32_t expired;
    uint32_t cancelled;
    uint32_t superseded;
    uint16_t queue_peak;
    uint16_t queued;
    uint16_t periodic_reserved_permille;
} CAN_TxSchedulerStats_t;

/* CAN 总线健康统计 */
typedef struct
{
    uint32_t busoff;              /* 检测到 BusOff 的次数 */
    uint32_t busoff_recover;      /* 尝试恢复的次数，不代表每次都成功。 */
    uint32_t rx_lost;             /* MESSAGE_LOST 事件次数 */
    uint32_t rx_frames;           /* 收到的帧数 */
    uint8_t tx_error_count;       /* TEC */
    uint8_t rx_error_count;       /* REC */
    CAN_Status_e status;          /* 当前状态 */
} CAN_BusStats_t;

/* 观测记录需要的纯软件计数；不触发 HAL 查询或总线恢复。 */
typedef struct
{
    uint32_t active;
    uint32_t tx_ok;
    uint32_t tx_drop;
    uint32_t tx_error;
    uint32_t tx_done;
    uint32_t tx_fifo_empty_events;
    uint32_t busoff;
    uint32_t busoff_recover;
    uint32_t rx_lost;
    uint32_t rx_frames;
    uint32_t rx_queue_drop;
    uint32_t rx_queue_peak;
    uint32_t tx_error_count;
    uint32_t rx_error_count;
    uint32_t status;
} CAN_ObservationCounters_t;

/** @brief 注册标准 CAN ID，按需启动服务并添加接收过滤规则。
* @param[in] config 有效总线、非空回调及 0～0x7ff 的收发 ID；id 指向的模块须长期有效。
* @return 成功返回长期有效实例，参数、重复接收 ID、容量、分配或 HAL 配置失败返回 NULL。
* @note 启动时注册；回调由派发任务运行，同一总线相同接收 ID 不允许重复注册。
 */
CANInstance *CANRegister(CAN_Init_Config_s *config);

/** @brief 清零固定 CAN 工作区并装入总线句柄，不启动硬件或发送报文。
* @note 由 BSPInit 在创建任务前调用一次；普通主机测试可由首次 CAN API 自动触发同一初始化。
 */
void CANInitWorkspace(void);

/** @brief 配置一条 FDCAN 总线的发送容量和周期利用率预算。
* @param[in] hcan 已支持的 FDCAN 句柄；必须在首次 CANRegister() 前调用。
* @param[in] config 每总线独立配置；队列容量 1～16、利用率 1～1000‰、速率大于零。
* @return 调度状态；配置失败不改变已有配置。
* @note 静态默认值为 CAN1 16项/700‰、CAN2 12项/650‰、CAN3 8项/500‰，标称速率均为1Mbit/s。
 */
CAN_TxSubmitStatus_t CANConfigureTxBus(FDCAN_HandleTypeDef *hcan,
                                       const CAN_TxBusConfig_t *config);

/** @brief 登记或更新周期最新目标；同来源和键的未启动旧目标会被替换。
* @param[in] hcan 已启动服务的总线句柄。
* @param[in] source 长期稳定的流来源标识；同一流更新时必须保持相同指针。
* @param[in] key 来源内部的流键。
* @param[in] frame 单帧经典 CAN 数据，提交时复制 0～8 字节。
* @param[in] priority 数值越小越先处理，范围 0～3。
* @param[in] deadline_ms 从本次成功提交起的期限，单位 ms，范围 1～INT32_MAX，边界到达即过期。
* @param[in] period_us 周期流的最小更新周期，单位 us，用于保守带宽核算，须大于零。
* @param[out] request_id 非空；仅成功／合并时写入非零编号，失败保持原值。
* @return 接收、合并、容量、带宽、上下文或参数状态；成功不代表 HAL 已接受。
* @note 普通任务上下文调用。负载在返回后可立即复用；周期帧遇忙或 HAL 拒绝时本次即失败，等待下一周期的新目标，不重放旧值。
 */
CAN_TxSubmitStatus_t CANSubmitPeriodic(FDCAN_HandleTypeDef *hcan, const void *source,
                                       uint32_t key, const CAN_TxFrame_t *frame,
                                       CAN_TxPriority_t priority, uint32_t deadline_ms,
                                       uint32_t period_us, uint32_t *request_id);

/** @brief 提交带发送前准备回调的周期目标；旧周期流已有准备回调时，提交失败会保留该回调。
 * @param[in] hcan 已启动服务的总线句柄。
 * @param[in] source 长期有效的来源标识，也会传给 prepare 作为回调上下文。
 * @param[in] key 来源内的周期流编号。
 * @param[in] frame 经典 CAN 数据帧；提交时复制负载。
 * @param[in] priority 调度优先级。
 * @param[in] deadline_ms 从提交时刻起的最长期限。
 * @param[in] period_us 周期流的带宽预算周期。
 * @param[in] prepare 可为 NULL；非空时在实际交给 HAL 前调用，修改队列自有负载副本。
 * @param[out] request_id 非空；仅接受或合并成功时写入请求编号。
 * @return 与 CANSubmitPeriodic 相同；参数错误、容量不足或带宽超限时拒绝。
 * @note prepare 在短临界区内运行，必须有界，不得阻塞、分配内存或再次提交 CAN 请求。
 *       prepare 收到的 source 指针及其指向数据须在周期流排队期间保持有效。
 *       成功替换会同时更新队列帧及回调；提交失败时旧帧和旧回调保持不变。
 */
CAN_TxSubmitStatus_t CANSubmitPeriodicPrepared(FDCAN_HandleTypeDef *hcan, const void *source,
                                       uint32_t key, const CAN_TxFrame_t *frame,
                                       CAN_TxPriority_t priority, uint32_t deadline_ms,
                                       uint32_t period_us, CAN_TxPeriodicPrepareFn_t prepare,
                                       uint32_t *request_id);

/** @brief 提交一条有界一次性命令。
* @param[in] hcan 已启动服务的总线句柄。
* @param[in] source 可选长期来源标识，仅供诊断关联。
* @param[in] frame 经典 CAN 数据帧；提交时复制负载。
* @param[in] priority 数值越小越先处理，范围 0～3。
* @param[in] deadline_ms 从成功入队起算的总期限，单位 ms，范围 1～INT32_MAX。
* @param[out] request_id 非空；成功时写非零编号，失败保持原值。
* @return 调度器提交结果；队列接收不等于 HAL 接受或对端确认。
* @note 未被 HAL 接受的帧可重试初次尝试后的最多3次；HAL接受后的失败不重试。
 */
CAN_TxSubmitStatus_t CANSubmitOneShot(FDCAN_HandleTypeDef *hcan, const void *source,
                                      const CAN_TxFrame_t *frame,
                                      CAN_TxPriority_t priority, uint32_t deadline_ms,
                                      uint32_t *request_id);

/** @brief 原子复制一组按批内顺序调度的一次性帧，容量不足时整组拒绝。
* @param[in] hcan 已启动服务的总线句柄。
* @param[in] source 可选长期来源标识。
* @param[in] frames 连续可读的帧描述数组，1～8项；每项负载在返回时复制。
* @param[in] frame_count 组内帧数，不得超过8。
* @param[in] priority 数值越小越先处理，范围 0～3。
* @param[in] deadline_ms 从成功入队起算的总期限，单位 ms，范围 1～INT32_MAX。
* @param[out] request_id 非空；失败保持原值。
* @return 调度器提交结果；组内帧依次尝试提交，完成／失败通过状态查询报告。
* @note 一组提交期间不会插入其他软件请求，但其他总线节点仍可参与仲裁；已被 HAL 接受的帧不会由软件重试。
 */
CAN_TxSubmitStatus_t CANSubmitOneShotBatch(FDCAN_HandleTypeDef *hcan, const void *source,
                                           const CAN_TxFrame_t *frames, size_t frame_count,
                                           CAN_TxPriority_t priority, uint32_t deadline_ms,
                                           uint32_t *request_id);

/** @brief 提交允许被更高优先级请求中断的一组帧。
 * @param[in] hcan,source,frames,frame_count,priority,deadline_ms 与CANSubmitOneShotBatch相同。
 * @param[out] request_id 成功返回编号，失败保持原值。
 * @return 队列提交状态；被中断的组最终记为FAILED，不自动重发。
 * @note 已交给HAL的帧无法撤回；接收方须在期限内收到完整组并检查校验后才能使用。
 */
CAN_TxSubmitStatus_t CANSubmitInterruptibleBatch(FDCAN_HandleTypeDef *hcan, const void *source,
    const CAN_TxFrame_t *frames, size_t frame_count, CAN_TxPriority_t priority,
    uint32_t deadline_ms, uint32_t *request_id);

/** @brief 提交可中断帧组，使用来源数据的本机绝对截止时刻。
 * @param[in] hcan,source,frames,frame_count,priority 与可中断批量提交相同。
 * @param[in] expiry_ms 本机HAL毫秒计数的截止时刻，支持32位回绕。
 * @param[out] request_id 成功时写入；失败保持原值。
 * @return 提交结果；到达期限或剩余时间大于INT32_MAX时拒绝。
 * @note 队列内读取当前时间并检查截止时刻，等待或抢占不会延长数据期限。
 */
CAN_TxSubmitStatus_t CANSubmitInterruptibleBatchUntil(FDCAN_HandleTypeDef *hcan, const void *source,
    const CAN_TxFrame_t *frames, size_t frame_count, CAN_TxPriority_t priority,
    uint32_t expiry_ms, uint32_t *request_id);

/** @brief 在1ms级普通任务中推进每条总线最多一帧，按优先级与期限调度。
* @note 只在 MotorControlTask() 或专用自测任务调用；不等待 FIFO 空闲，每条总线每次至多尝试一帧。
 */
void CANProcessTx(void);

/** @brief 只对指定总线额外推进至多一帧。
 * @param[in] hcan 已支持的FDCAN句柄；NULL或未知句柄直接返回。
 * @note 仅普通任务串行调用；不等待FIFO，优先级、期限和失败处理与CANProcessTx相同。
 */
void CANProcessTxBus(FDCAN_HandleTypeDef *hcan);

/** @brief 查询请求入队、HAL接受及完成状态；读取不推进队列。
* @param[in] request_id 非零请求编号。
* @param[out] result 非空；找到时整体复制，参数错误或记录已回收时保持原值。
* @return 1 表示找到请求，0 表示参数错误、ISR 上下文或记录已回收。
* @note 限普通任务查询；HAL 接受与传输完成由不同状态表示。
 */
uint8_t CANGetTxRequestState(uint32_t request_id, CAN_TxResult_t *result);

/** @brief 取消尚未全部交给 HAL 的请求；已接受帧不能撤回。
* @param[in] request_id 非零请求编号。
* @return 1 表示排队剩余部分已取消，0 表示请求不存在或已全部交给 HAL。
 */
uint8_t CANCancelTxRequest(uint32_t request_id);

/** @brief 复制指定总线调度统计；查询不推进发送。
* @param[in] hcan 已支持的 FDCAN 句柄。
* @param[out] stats 输出对象；参数错误时保持原值。
* @return 1 表示复制成功，0 表示参数无效或处于 ISR 上下文。
 */
uint8_t CANGetTxSchedulerStats(FDCAN_HandleTypeDef *hcan,
                               CAN_TxSchedulerStats_t *stats);

/** @brief 估算经典 CAN 帧含最坏位填充和帧间隔的位数。
* @param[in] identifier_type 标准或扩展标识符类型。
* @param[in] data_length 数据长度，0～8字节。
* @return 合法帧的保守位数；参数无效返回0。
 */
uint16_t CANClassicFrameWorstCaseBits(CAN_TxIdType_t identifier_type,
                                      uint8_t data_length);

/** @brief 读取 CAN 协议及错误计数。
* @param[in] hcan 有效句柄；NULL 时不处理。
* @param[out] status NULL 时不处理；先设 UNKNOWN、计数 0，HAL 查询失败则保留这些值。
 */
void CANGetStatus(FDCAN_HandleTypeDef *hcan, CAN_Status_t *status);

/** @brief 复制指定总线的累计发送统计。
* @param[in] hcan 已知总线句柄。
* @param[out] stats 输出对象，NULL 或句柄未知时保持原值。
 */
void CANGetTxStats(FDCAN_HandleTypeDef *hcan, CAN_TxStats_t *stats);

/** @brief 查询当前错误计数，并复制指定总线的累计状态。
* @param[in] hcan 已知总线句柄。
* @param[out] stats 输出对象，NULL 或句柄未知时保持原值。
 */
void CANGetBusStats(FDCAN_HandleTypeDef *hcan, CAN_BusStats_t *stats);

/** @brief 复制指定总线的纯软件计数，不读取寄存器、不执行恢复。
* @param[in] bus_index 总线编号，范围 0～DEVICE_CAN_CNT-1。
* @param[out] counters 输出对象；成功时整体更新，参数错误时保持原值。
* @return 1 表示复制成功，0 表示总线编号或输出指针无效。
* @note 普通任务上下文调用；复制期间短暂屏蔽中断。
 */
uint8_t CANGetObservationCounters(uint8_t bus_index, CAN_ObservationCounters_t *counters);

/* ---------------- 接收队列(ISR 入队 / 任务派发, BSPCAN-02) ----------------
   接收中断现在只把帧压进每条总线各自的环形队列, 由 CANProcessRx() 在任务上下文
   逐帧调用模块回调。必须遵守:
     1) CANProcessRx() 全工程只能有一个调用者(否则 tail 会被多任务竞争),
        当前固定由 MotorControlTask() 在算控制之前调用;
     2) 回环类自测的等待循环里也要调用它, 否则回调永远不会跑;
     3) 队列满时丢弃新帧, 用 CANGetRxQueueDrop() 观察丢帧数。 */
void CANProcessRx(void);

/** @brief 读取接收队列满时丢弃新帧的次数。
* @param[in] hcan 已知总线句柄。
* @return 累计次数，句柄未知返回 0。
 */
uint32_t CANGetRxQueueDrop(FDCAN_HandleTypeDef *hcan);

/* 接收队列诊断(由 CANHealthMonitor 每 100ms 汇总): drop 应恒为 0,
   peak 是队列历史最大占用, 用来判断深度(32)够不够 */
extern volatile uint32_t can_rxq_drop_total;
extern volatile uint16_t can_rxq_peak_max;

/** @brief 自测用：重新初始化一条总线到指定模式并尝试启动，会打断原有收发。
* @param[in,out] h 有效句柄，NULL 返回 0。
* @param[in] mode HAL 定义的 FDCAN 模式。
* @param[in] accept_all 当前预留且忽略，不会切成接收全部 ID。
* @return 1 为配置后已离开初始化状态，0 为失败；后续仍须重装原过滤规则。
 */
uint8_t CANForceMode(FDCAN_HandleTypeDef *h, uint32_t mode, uint8_t accept_all);
/** @brief 按当前注册表重装过滤规则；仅供切换模式后使用，不报告单项失败。 */
void CANReapplyFilters(void);

/** @brief 移出注册表及接收查找表并释放实例，仅供自测收尾。
* @param[in] ins 待释放实例；NULL 或不在表中时不处理，成功后不可再使用。
* @note 须先确保接收队列及回调不再引用它，函数不处理已经排队的旧指针。
 */
void CANUnregisterInstance(CANInstance *ins);

/**
 * @brief CAN 总线健康检查与 BusOff 恢复; 建议以 10Hz 在任务上下文调用
 *        (恢复动作要动多个寄存器, 不能放在中断里)
 */
void CANHealthMonitor(void);

/** @brief 在三条总线上依次切换回环、测试、恢复并重装过滤规则，供专门自测使用。
* @return 通过的总线数，范围 0～3；默认外部回环会发送测试帧。
 */
uint8_t CANRunLoopbackSelfTest(void);

/* 自测结果(供 Live Watch / OpenOCD 读取) */
extern volatile uint32_t can_selftest_tx_ok;    /* 成功交给硬件的测试帧数 */
extern volatile uint32_t can_selftest_tx_fail;  /* 发送被拒绝的帧数 */
extern volatile uint32_t can_selftest_rx_ok;    /* 接收回调里数据完全正确的帧数 */
extern volatile uint32_t can_selftest_rx_bad;   /* 收到但数据不对的帧数 */
extern volatile uint8_t can_selftest_pass_mask; /* bit0/1/2 = CAN1/2/3 通过 */
extern volatile uint8_t can_selftest_done;      /* 1 = 已跑过 */

#endif
