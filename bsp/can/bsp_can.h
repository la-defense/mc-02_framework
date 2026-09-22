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

/* ---------------- FDCAN 内部回环自测(2026-09) ----------------
   置 1 则启动时自动跑一次自测: 把每条总线临时切到"内部回环"模式, 自己发自己收,
   验证 "软件发送 -> 硬件 -> 接收中断 -> 滤镜匹配 -> 回调解析" 整条链路。
   特点: 不驱动总线, 所以总线上挂着其他设备也安全; 无需外部 ACK, 不会累积错误计数。
   测完会自动恢复 NORMAL 模式并把已注册实例的滤镜重新加回去。
   结果见下面的 can_selftest_* 变量, 也可以用 Live Watch / OpenOCD 读。 */
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
/* !! 这里绝对不能再加 #pragma pack(1) !!
   -O2 下 HAL 的 FDCAN_CopyMessageToRAM() 会把 4 次字节读合并成一次 32 位读
   ("pTxData[3]<<24 | pTxData[2]<<16 | pTxData[1]<<8 | pTxData[0]" 等价于一条 ldr),
   而 pack(1) 会把 CANInstance 压成 1 字节对齐: dji_motor.c 里的
   static CANInstance sender_assignment[9] 每个元素 77 字节, 于是
   sender_assignment[1]/[2]/... 的地址落在非 4 字节边界, tx_buff 也是 →
   一次 32 位读直接 UsageFault(UNALIGNED) → HardFault。
   实测: -Og 逐字节拷贝不炸, -O2 一上电就 HardFault(复位循环)。
   自然对齐后 sizeof 从 77 → 80, 多出的 3 字节填充完全可以接受。 */
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
    // 接收的回调函数,用于解析接收到的数据
    void (*can_module_callback)(struct _ *); // callback needs an instance to tell among registered ones
    void *id;                                // 使用can外设的模块指针(即id指向的模块拥有此can实例,是父子关系)
} CANInstance;

/* 编译期兜底: HAL 会把 tx_buff / &txconf 当 32 位数据访问, 必须 4 字节对齐。
   谁将来再加回 pack(1), 这里会直接编译不过。 */
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
    uint32_t tx_drop;             /* FIFO 满被丢弃的帧数(下一周期自然重发) */
    uint32_t tx_error;            /* HAL 返回错误的次数 */
    uint32_t tx_done;             /* 发送完成中断统计到的帧数 */
    uint32_t tx_fifo_empty_events;/* TX FIFO 变空事件次数 */
} CAN_TxStats_t;

/* CAN 总线健康统计 */
typedef struct
{
    uint32_t busoff;              /* 检测到 BusOff 的次数 */
    uint32_t busoff_recover;      /* 执行恢复(Stop+Start)的次数 */
    uint32_t rx_lost;             /* MESSAGE_LOST 事件次数 */
    uint32_t rx_frames;           /* 收到的帧数 */
    uint8_t tx_error_count;       /* TEC */
    uint8_t rx_error_count;       /* REC */
    CAN_Status_e status;          /* 当前状态 */
} CAN_BusStats_t;

/**
 * @brief Register a module to CAN service,remember to call this before using a CAN device
 *        注册(初始化)一个can实例,需要传入初始化配置的指针.
 * @param config init config
 * @return CANInstance* can instance owned by module
 */
CANInstance *CANRegister(CAN_Init_Config_s *config);

/**
 * @brief 修改CAN发送报文的数据帧长度;注意最大长度为8,在没有进行修改的时候,默认长度为8
 *
 * @param _instance 要修改长度的can实例
 * @param length    设定长度
 */
void CANSetDLC(CANInstance *_instance, uint8_t length);

/**
 * @brief transmit mesg through CAN device,通过can实例发送消息
 *        发送前需要向CAN实例的tx_buff写入发送数据
 * 
 * @attention 超时时间不应该超过调用此函数的任务的周期,否则会导致任务阻塞
 * 
 * @param timeout 超时时间,单位为ms;后续改为us,获得更精确的控制
 * @param _instance* can instance owned by module
 */
uint8_t CANTransmit(CANInstance *_instance,float timeout);

/**
 * @brief 获取FDCAN总线状态(OK/ERROR/BUSOFF)
 */
void CANGetStatus(FDCAN_HandleTypeDef *hcan, CAN_Status_t *status);

/**
 * @brief 读取某条总线的发送统计(丢帧/完成计数)
 */
void CANGetTxStats(FDCAN_HandleTypeDef *hcan, CAN_TxStats_t *stats);

/**
 * @brief 读取某条总线的健康统计(BusOff 次数/恢复次数/错误计数)
 */
void CANGetBusStats(FDCAN_HandleTypeDef *hcan, CAN_BusStats_t *stats);

/**
 * @brief CAN 总线健康检查与 BusOff 恢复; 建议以 10Hz 在任务上下文调用
 *        (恢复动作要动多个寄存器, 不能放在中断里)
 */
void CANHealthMonitor(void);

/* 手动运行一次内部回环自测(会短暂中断该总线收发), 返回通过的总线数(0~3) */
uint8_t CANRunLoopbackSelfTest(void);

/* 自测结果(供 Live Watch / OpenOCD 读取) */
extern volatile uint32_t can_selftest_tx_ok;    /* 成功交给硬件的测试帧数 */
extern volatile uint32_t can_selftest_tx_fail;  /* 发送被拒绝的帧数 */
extern volatile uint32_t can_selftest_rx_ok;    /* 接收回调里数据完全正确的帧数 */
extern volatile uint32_t can_selftest_rx_bad;   /* 收到但数据不对的帧数 */
extern volatile uint8_t can_selftest_pass_mask; /* bit0/1/2 = CAN1/2/3 通过 */
extern volatile uint8_t can_selftest_done;      /* 1 = 已跑过 */

#endif
