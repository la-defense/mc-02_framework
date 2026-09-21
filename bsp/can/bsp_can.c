#include "bsp_can.h"
#include "main.h"
#include "memory.h"
#include "stdlib.h"
#include "bsp_dwt.h"
#include "bsp_log.h"
#include "cmsis_os.h"

/* can instance ptrs storage, used for recv callback */
// 在CAN产生接收中断会遍历数组,选出hcan和rxid与发生中断的实例相同的那个,调用其回调函数
// @todo: 后续为每个CAN总线单独添加一个can_instance指针数组,提高回调查找的性能
static CANInstance *can_instance[CAN_MX_REGISTER_CNT] = {NULL};
static uint8_t idx; // 全局CAN实例索引,每次有新的模块注册会自增

/* ---------------- 总线状态与接收查表(2026-09 CAN 重构) ----------------
   - 每条总线一份统计: 发送丢帧/完成、BusOff、错误计数;
   - rx_lookup[总线][rx_id & 0x1FF] 做 O(1) 派发, 命中后再校验完整 rx_id;
     512×4B×3 总线 = 6KB RAM(用完整 11 位做直表要 24KB, 太贵)。 */
#define CAN_RX_LOOKUP_SIZE 0x200u

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
    uint32_t log_ms;   /* 限速日志时间戳 */
    uint32_t log_cnt;  /* 被限速掉的日志条数 */
} CANBusState_t;

static CANBusState_t can_bus[DEVICE_CAN_CNT] = {
    {.handle = &hfdcan1}, {.handle = &hfdcan2}, {.handle = &hfdcan3}};
static CANInstance *rx_lookup[DEVICE_CAN_CNT][CAN_RX_LOOKUP_SIZE];

/* ---------------- 性能探针(见 docs/学习笔记/03) ---------------- */
volatile DWT_Probe_t can_prof_tx = {0};
volatile uint32_t can_prof_tx_spins = 0;   /* 保留: 自旋已删除, 恒为 0 */
volatile uint32_t can_prof_tx_full = 0;    /* 保留: 兼容旧观察, 等于 tx_drop 累计 */
volatile DWT_Probe_t can_prof_rx_isr = {0};
volatile uint32_t can_prof_rx_frames = 0;

static CANBusState_t *CANBusFromHandle(FDCAN_HandleTypeDef *_handle)
{
    for (uint8_t i = 0; i < DEVICE_CAN_CNT; ++i)
    {
        if (can_bus[i].handle == _handle)
            return &can_bus[i];
    }
    return NULL;
}

static uint8_t CANBusIndexFromHandle(FDCAN_HandleTypeDef *_handle)
{
    for (uint8_t i = 0; i < DEVICE_CAN_CNT; ++i)
    {
        if (can_bus[i].handle == _handle)
            return i;
    }
    return 0xFFu;
}

/* 限速日志: 同一类消息最多 1 条/秒, 其余只计数(避免日志把 CPU 和 RTT 缓冲吃光) */
static void CANLogRateLimited(CANBusState_t *bus, const char *tag, uint32_t value)
{
    uint32_t now = HAL_GetTick();
    if (bus == NULL)
        return;
    if ((uint32_t)(now - bus->log_ms) >= 1000u)
    {
        bus->log_ms = now;
        LOGWARNING("[bsp_can] %s (%lu, 本秒内被限速 %lu 条)",
                   tag, (unsigned long)value, (unsigned long)bus->log_cnt);
        bus->log_cnt = 0;
    }
    else
    {
        bus->log_cnt++;
    }
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
static void CANAddFilter(CANInstance *_instance)
{

#ifdef FDCAN
	static uint8_t can1_filter_idx = 0, can2_filter_idx = 0 , can3_filter_idx = 0;
	//检查是否超出过滤器设定数量上限
	if(can1_filter_idx > hfdcan1.Init.StdFiltersNbr || can2_filter_idx>hfdcan2.Init.StdFiltersNbr || can3_filter_idx > hfdcan3.Init.StdFiltersNbr)
	{
		while(1)
		{
			//报错
		}
	}
	uint8_t *filter_idx_p;

	if(_instance->can_handle==&hfdcan1)
	{
		filter_idx_p=&can1_filter_idx;
	}
	else if(_instance->can_handle==&hfdcan2)
	{
		filter_idx_p=&can2_filter_idx;
	}
	else if(_instance->can_handle==&hfdcan3)
	{
		filter_idx_p=&can3_filter_idx;
	}
	else
	{
		while(1)
		{
			//报错
		}
	}

	FDCAN_FilterTypeDef fdcan_filter_conf;
	fdcan_filter_conf.FilterIndex=(*filter_idx_p)++;
	//使用单个ID模式
	fdcan_filter_conf.FilterType=FDCAN_FILTER_DUAL;
	fdcan_filter_conf.FilterConfig=(_instance->tx_id & 1) ? FDCAN_FILTER_TO_RXFIFO0 : FDCAN_FILTER_TO_RXFIFO1;//奇数id的模块会被分配到FIFO0,偶数id的模块会被分配到FIFO1
	fdcan_filter_conf.FilterID1=_instance->rx_id;
	fdcan_filter_conf.FilterID2=_instance->rx_id;
	fdcan_filter_conf.IdType=FDCAN_STANDARD_ID;
	fdcan_filter_conf.IsCalibrationMsg=0;
	//fdcan_filter_conf.RxBufferIndex=0;

	HAL_FDCAN_ConfigFilter(_instance->can_handle, &fdcan_filter_conf);

#else
	CAN_FilterTypeDef can_filter_conf;
	static uint8_t can1_filter_idx = 0, can2_filter_idx = 14; // 0-13给can1用,14-27给can2用

	can_filter_conf.FilterMode = CAN_FILTERMODE_IDLIST;                                                       // 使用id list模式,即只有将rxid添加到过滤器中才会接收到,其他报文会被过滤
	can_filter_conf.FilterScale = CAN_FILTERSCALE_16BIT;                                                      // 使用16位id模式,即只有低16位有效
	can_filter_conf.FilterFIFOAssignment = (_instance->tx_id & 1) ? CAN_RX_FIFO0 : CAN_RX_FIFO1;              // 奇数id的模块会被分配到FIFO0,偶数id的模块会被分配到FIFO1
	can_filter_conf.SlaveStartFilterBank = 14;                                                                // 从第14个过滤器开始配置从机过滤器(在STM32的BxCAN控制器中CAN2是CAN1的从机)
	can_filter_conf.FilterIdLow = _instance->rx_id << 5;                                                      // 过滤器寄存器的低16位,因为使用STDID,所以只有低11位有效,高5位要填0
	can_filter_conf.FilterBank = _instance->can_handle == &hcan1 ? (can1_filter_idx++) : (can2_filter_idx++); // 根据can_handle判断是CAN1还是CAN2,然后自增
	can_filter_conf.FilterActivation = CAN_FILTER_ENABLE;                                                     // 启用过滤器

	HAL_CAN_ConfigFilter(_instance->can_handle, &can_filter_conf);
#endif

}

/**
 * @brief 在第一个CAN实例初始化的时候会自动调用此函数,启动CAN服务
 *
 * @note 此函数会启动CAN1并开启中断
 *       FDCAN的情况下，我们采用FIFO接收方式（而不是buffer），FIFO和buffer还有queue接收方式请自行查阅H723手册
 *       FDCAN比bxCAN多了一个全局过滤器，这里配置为全部拒绝，只接受指定ID。
 *       
 */
void CANServiceInit()
{
#ifdef FDCAN
	//HAL_FDCAN_ConfigClockCalibration()
	HAL_FDCAN_ConfigRxFifoOverwrite(&hfdcan1,FDCAN_RX_FIFO0,FDCAN_RX_FIFO_OVERWRITE);
	HAL_FDCAN_ConfigRxFifoOverwrite(&hfdcan1,FDCAN_RX_FIFO1,FDCAN_RX_FIFO_OVERWRITE);
	HAL_FDCAN_ConfigGlobalFilter(&hfdcan1, FDCAN_REJECT, FDCAN_REJECT, FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE);//全局过滤器设置
	HAL_FDCAN_Start(&hfdcan1);
	HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_RX_ACTIVE_ITS, 0);
	/* TX 完成/队列空中断默认不开: 实测这两个中断每秒会进 2 万多次(远多于实际帧数),
	   而它们能提供的信息(tx_done)与 tx_ok 基本重复, 不划算。
	   需要统计时把下面两行的注释打开即可(回调已经实现)。 */
	// HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_TX_FIFO_EMPTY, 0);
	// HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_TX_COMPLETE, FDCAN_TX_BUFFER_IT_MASK);

	HAL_FDCAN_ConfigRxFifoOverwrite(&hfdcan2,FDCAN_RX_FIFO0,FDCAN_RX_FIFO_OVERWRITE);
	HAL_FDCAN_ConfigRxFifoOverwrite(&hfdcan2,FDCAN_RX_FIFO1,FDCAN_RX_FIFO_OVERWRITE);
	HAL_FDCAN_ConfigGlobalFilter(&hfdcan2, FDCAN_REJECT, FDCAN_REJECT, FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE);
	HAL_FDCAN_Start(&hfdcan2);
	HAL_FDCAN_ActivateNotification(&hfdcan2, FDCAN_RX_ACTIVE_ITS, 0);
	// HAL_FDCAN_ActivateNotification(&hfdcan2, FDCAN_IT_TX_FIFO_EMPTY, 0);
	// HAL_FDCAN_ActivateNotification(&hfdcan2, FDCAN_IT_TX_COMPLETE, FDCAN_TX_BUFFER_IT_MASK);


	HAL_FDCAN_ConfigRxFifoOverwrite(&hfdcan3,FDCAN_RX_FIFO0,FDCAN_RX_FIFO_OVERWRITE);
	HAL_FDCAN_ConfigRxFifoOverwrite(&hfdcan3,FDCAN_RX_FIFO1,FDCAN_RX_FIFO_OVERWRITE);
	HAL_FDCAN_ConfigGlobalFilter(&hfdcan3, FDCAN_REJECT, FDCAN_REJECT, FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE);
	HAL_FDCAN_Start(&hfdcan3);
	HAL_FDCAN_ActivateNotification(&hfdcan3, FDCAN_RX_ACTIVE_ITS, 0);
	// HAL_FDCAN_ActivateNotification(&hfdcan3, FDCAN_IT_TX_FIFO_EMPTY, 0);
	// HAL_FDCAN_ActivateNotification(&hfdcan3, FDCAN_IT_TX_COMPLETE, FDCAN_TX_BUFFER_IT_MASK);


#else
	HAL_CAN_Start(&hcan1);
	HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
	HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO1_MSG_PENDING);
	HAL_CAN_Start(&hcan2);
	HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);
	HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO1_MSG_PENDING);
#endif

}

/* ----------------------- two extern callable function -----------------------*/

CANInstance *CANRegister(CAN_Init_Config_s *config)
{
    if (!idx)
    {
        CANServiceInit(); // 第一次注册,先进行硬件初始化
        LOGINFO("[bsp_can] CAN Service Init");
    }
    if (idx >= CAN_MX_REGISTER_CNT) // 超过最大实例数
    {
        while (1)
        {
        	/* 这类错误属于"启动期配置错误", 保持不继续注册, 但限速打印 + 让出 CPU,
        	   避免刷爆日志/把整机拖死(排查完应修正注册数量) */
        	LOGERROR("[bsp_can] CAN instance exceeded MAX num (%d), 请减少注册数量或分摊总线", CAN_MX_REGISTER_CNT);
        	osDelay(1000);
        }
    }
    for (size_t i = 0; i < idx; i++)
    { // 重复注册 | id重复
        if (can_instance[i]->rx_id == config->rx_id && can_instance[i]->can_handle == config->can_handle)
        {
            while (1)
            {
            	LOGERROR("[bsp_can] CAN rx_id 冲突: rx_id=%lu 已在同一条总线上注册, 请检查设备 ID 配置",
            	         (unsigned long)config->rx_id);
            	osDelay(1000);
            }
        }
    }

    CANInstance *instance = (CANInstance *)malloc(sizeof(CANInstance)); // 分配空间
    memset(instance, 0, sizeof(CANInstance));                           // 分配的空间未必是0,所以要先清空
    // 进行发送报文的配置
#ifdef FDCAN
    instance->txconf.Identifier = config->tx_id; 				// 发送id
    instance->txconf.IdType = FDCAN_STANDARD_ID;  				// 使用标准id,扩展id则使用CAN_ID_EXT(目前没有需求)
    instance->txconf.TxFrameType = FDCAN_DATA_FRAME,    		// 发送数据帧
    instance->txconf.DataLength = FDCAN_DLC_BYTES_8,    		// 数据长度为8字节
	instance->txconf.ErrorStateIndicator = FDCAN_ESI_ACTIVE,	// 兼容CAN2.0,错误状态指示器设为主动
	instance->txconf.BitRateSwitch = FDCAN_BRS_OFF,         	// 兼容CAN2.0禁用位速率切换
	instance->txconf.FDFormat = FDCAN_CLASSIC_CAN,          	// 使用经典CAN格式
	instance->txconf.TxEventFifoControl = FDCAN_NO_TX_EVENTS,	// 不需要，禁用事件FIFO
	instance->txconf.MessageMarker = 0;                     	// 不使用消息标记
#else
    instance->txconf.StdId = config->tx_id; // 发送id
    instance->txconf.IDE = CAN_ID_STD;      // 使用标准id,扩展id则使用CAN_ID_EXT(目前没有需求)
    instance->txconf.RTR = CAN_RTR_DATA;    // 发送数据帧
    instance->txconf.DLC = 0x08;            // 默认发送长度为8
#endif
    // 设置回调函数和接收发送id
    instance->can_handle = config->can_handle;
    instance->tx_id = config->tx_id; // 好像没用,可以删掉
    instance->rx_id = config->rx_id;
    instance->can_module_callback = config->can_module_callback;
    instance->id = config->id;

    CANAddFilter(instance);         // 添加CAN过滤器规则
    can_instance[idx++] = instance; // 将实例保存到can_instance中

    /* 建立 O(1) 接收派发表: rx_lookup[总线][rx_id & 0x1FF] */
    uint8_t bus_idx = CANBusIndexFromHandle(config->can_handle);
    if (bus_idx != 0xFFu)
        rx_lookup[bus_idx][config->rx_id & (CAN_RX_LOOKUP_SIZE - 1u)] = instance;

    return instance; // 返回can实例指针
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
        /* 硬件 FIFO 满: 直接丢帧, 绝不等待。
           丢一帧的代价是"这个电机再执行 1ms 上一帧指令"(可忽略);
           等待的代价是整个 motor 任务超预算、所有电机一起延迟。 */
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

        if (ps.BusOff)
        {
            bus->bus.busoff++;
            HAL_FDCAN_Stop(bus->handle);   /* 置 CCCR.INIT, 复位错误状态 */
            HAL_FDCAN_Start(bus->handle);  /* 清 INIT, 重新上线 */
            /* Stop 之后中断使能会丢失, 重新打开 */
            HAL_FDCAN_ActivateNotification(bus->handle, FDCAN_RX_ACTIVE_ITS, 0);
            bus->bus.busoff_recover++;
            CANLogRateLimited(bus, "BusOff 检测到并尝试恢复", bus->bus.busoff);
        }
    }
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
    /* 原来的实现在参数错误时会 while(1) 死循环(整机卡死)。
       这里改成: 钳位到合法范围 + 计数 + 限速告警, 系统继续运行 */
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
/**
 * @brief 此函数会被下面两个函数调用,用于处理FIFO0和FIFO1溢出中断(说明收到了新的数据)
 *        所有的实例都会被遍历,找到can_handle和rx_id相等的实例时,调用该实例的回调函数
 *
 * @param _fdhcan
 * @param fifox passed to HAL_CAN_GetRxMessage() to get mesg from a specific fifo
 */
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
        		target->rx_len = (uint8_t)DataLength;                     // 保存接收到的数据长度
        		memcpy(target->rx_buff, fdcan_rx_buff, target->rx_len);    // 消息拷贝到对应实例
        		target->can_module_callback(target);                      // 触发回调解析
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
			CANLogRateLimited(bus, "RxFIFO0 溢出丢帧", bus->bus.rx_lost);
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
			CANLogRateLimited(bus, "RxFIFO1 溢出丢帧", bus->bus.rx_lost);
		}
	}
	/* 检查是否有新消息写入Rx FIFO 1或到达一定阈值 */
	if ((RxFifo1ITs & FDCAN_IT_RX_FIFO1_NEW_MESSAGE)||(RxFifo1ITs & FDCAN_IT_RX_FIFO1_FULL)||(RxFifo1ITs & FDCAN_IT_RX_FIFO1_WATERMARK))
	{
		FDCANFIFOxCallback(hfdcan, FDCAN_RX_FIFO1); // 调用我们自己写的函数来处理消息
	}
}


#else


/**
* @brief 此函数会被下面两个函数调用,用于处理FIFO0和FIFO1溢出中断(说明收到了新的数据)
*        所有的实例都会被遍历,找到can_handle和rx_id相等的实例时,调用该实例的回调函数
*
* @param _hcan
* @param fifox passed to HAL_CAN_GetRxMessage() to get mesg from a specific fifo
*/
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

/**
* @brief 注意,STM32的两个CAN设备共享两个FIFO
* 下面两个函数是HAL库中的回调函数,他们被HAL声明为__weak,这里对他们进行重载(重写)
* 当FIFO0或FIFO1溢出时会调用这两个函数
*/
// 下面的函数会调用CANFIFOxCallback()来进一步处理来自特定CAN设备的消息

/**
* @brief rx fifo callback. Once FIFO_0 is full,this func would be called
*
* @param hcan CAN handle indicate which device the oddest mesg in FIFO_0 comes from
*/
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
   CANFIFOxCallback(hcan, CAN_RX_FIFO0); // 调用我们自己写的函数来处理消息
}

/**
* @brief rx fifo callback. Once FIFO_1 is full,this func would be called
*
* @param hcan CAN handle indicate which device the oddest mesg in FIFO_1 comes from
*/
void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
   CANFIFOxCallback(hcan, CAN_RX_FIFO1); // 调用我们自己写的函数来处理消息
}


#endif
