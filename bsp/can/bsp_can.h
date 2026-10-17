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
    uint8_t tx_buff[8];            // 发送缓存,发送消息长度可以通过CANSetDLC()设定,最大为8
    uint8_t rx_buff[8];            // 接收缓存,最大消息长度为8
    uint32_t rx_id;                // 接收id
    uint8_t rx_len;                // 接收长度,可能为0-8
    uint32_t rx_timestamp_ms; // 当前派发帧真正入队的 HAL 毫秒时间。
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

/** @brief 注册标准 CAN ID，按需启动服务并添加接收过滤规则。
* @param[in] config 有效总线、非空回调及 0～0x7ff 的收发 ID；id 指向的模块须长期有效。
* @return 成功返回长期有效实例，参数、重复接收 ID、容量、分配或 HAL 配置失败返回 NULL。
* @note 启动时注册；回调由派发任务运行，同一总线相同接收 ID 不允许重复注册。
 */
CANInstance *CANRegister(CAN_Init_Config_s *config);

/** @brief 设置下一帧数据字节数，默认 8。
* @param[in,out] _instance CAN 实例，NULL 时不处理。
* @param[in] length 1～8 字节；传 0 改为 1，超过 8 改为 8，并记录日志计数。
 */
void CANSetDLC(CANInstance *_instance, uint8_t length);

/** @brief 把实例 tx_buff 中的数据提交给 CAN 硬件，发送空间满时直接返回。
* @param[in] _instance 有效实例，不能为 NULL；先填写发送数组及字节数。
* @param[in] timeout 保留的旧参数，当前忽略，不作为等待时间。
* @return 1 为已交给硬件，0 为队列满或 HAL 拒绝；不表示对端已收到，也不自动重试。
 */
uint8_t CANTransmit(CANInstance *_instance,float timeout);

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
