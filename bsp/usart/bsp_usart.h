/** @file bsp_usart.h
* @brief UART注册和复制队列；MotorControlTask派发接收，专用UART任务推进发送与通知。
* @note 先配置 UART、接收 DMA 及对应中断；每个硬件句柄只注册一次。
 */
#ifndef BSP_RC_H
#define BSP_RC_H

#include <stdint.h>
#include <stddef.h>
#include "usart_rx.h"
#include "main.h"

#define DEVICE_USART_CNT 8     // MC02串口数量8个，其中
#define USART_RXBUFF_LIMIT 256 // DMA数组容量，注册容量受uint8_t限制为1～255字节。

#include "usart_tx.h"

// 串口实例结构体,每个module都要包含一个实例.
// 每个硬件句柄独占注册；业务状态通过context传回，接收数组只供HAL使用。
typedef struct
{
    uint8_t recv_buff[USART_RXBUFF_LIMIT]; // 仅供HAL/DMA使用，业务只能读回调收到的不可变块。
    uint8_t recv_buff_size;                // 单次接收容量1～255字节，不等于协议帧长。
    UART_HandleTypeDef *usart_handle;      // 实例对应的usart_handle
    usart_module_callback module_callback; // 解析收到的数据的回调函数
    void *context;                        // 调用者长期保存的上下文，不由BSP释放。
    volatile uint32_t rx_sequence;        // BSP内部连续性计数，不得由业务改写。
    volatile uint32_t rx_generation;      // 错误/重启后隔离此前已排队的数据，仅供BSP使用。
    USARTTxGuard_t tx_guard;              // 注册时固定，实际启动前检查，context复用本实例的长期对象。
    USARTTxGate_t tx_gate;                // 可NULL；发送任务调用，暂缓不改变期限及队首。
    USARTTxObserver_t tx_observer;        // 可NULL；任务中先观察终态，再通知本请求。
} USARTInstance;

/* usart 初始化配置结构体 */
typedef struct
{
    uint8_t recv_buff_size;                // 单次接收容量1～255字节。
    UART_HandleTypeDef *usart_handle;      // 实例对应的usart_handle
    usart_module_callback module_callback; // 解析收到的数据的回调函数
    void *context;                         // 原样传回回调，非空时须长期保存。
    USARTTxGuard_t tx_guard;                // 可NULL；每次实际启动前调用，禁止等待。
    USARTTxGate_t tx_gate;                 // 可NULL，规则及存储条件见usart_tx.h。
    USARTTxObserver_t tx_observer;         // 可NULL，与接收共用长期context。
} USART_Init_Config_s;

/** @brief 注册 UART、分配接收队列并启动 DMA 接收。
* @param[in] init_config 有效句柄、非空模块回调及 1～255 字节接收容量，配置会复制。
* @return 成功返回长期有效对象；参数、重复注册、容量或启动失败时返回 NULL。
* @note 启动阶段串行注册，派发任务开始前完成。配置复制，context对象由调用者长期保存。
* 启动失败后若无法确认DMA中止，保留隔离对象及池空间，避免释放硬件仍借用的数组。
 */
USARTInstance *USARTRegister(USART_Init_Config_s *init_config);

/** @brief 显式重新准备DMA，打断接收连续性并隔离此前已排队的块。
* @param[in] _instance 已准备 UART 与接收 DMA 的实例；NULL 或信息不完整时失败。
* @return 1 为启动成功，0 为参数不符或 HAL 拒绝；不会替调用方中止旧接收。
* @note 仅任务上下文对已注册实例串行调用。HAL拒绝时请求下轮派发中有限重试；
* 正常RxEvent后的DMA重装使用内部入口，不打断协议连续性。
 */
uint8_t USARTServiceInit(USARTInstance *_instance);


/** @brief 非阻塞复制提交，队列满立即拒绝；仅普通任务使用。
 * @param[in] instance 已注册UART，禁止与直接HAL发送混用。
 * @param[in] request 提交期间有效的完整请求，内容被复制。
 * @param[out] id 成功交付非零编号；失败保持原值，不通知complete。
 * @return ACCEPTED或参数、忙、未就绪、隔离、后端错误。
 * @note 每路4条，包含在途和待通知项；调用方源数据成功后即可释放。
 */
USARTSendStatus_t USARTSubmitTx(USARTInstance *instance, const USARTTxRequest_t *request, uint32_t *id);
/** @brief 请求取消指定编号；仅普通任务使用，实际通知仍由发送任务执行。
 * @param[in] instance 已注册UART。
 * @param[in] id 成功提交得到的非零编号。
 * @return ACCEPTED表示找到请求并处理取消；未找到、隔离或参数错误不改变其他事务。
 * @note 已形成终态或正在通知的请求保留既有结果；以最终通知判断是否取消。
 * 在途字节无法撤回，必须确认停止后才回收内部缓冲。
 */
USARTSendStatus_t USARTCancelTx(USARTInstance *instance, uint32_t id);
/** @brief 复制只读发送统计，不推进队列；任务可调用。
 * @param[in] instance 已注册UART。
 * @param[out] output 完整副本；失败保持原值，last_event仅保留最近通知。
 * @return ACCEPTED或BAD_ARGUMENT。
 */
USARTSendStatus_t USARTGetTxState(const USARTInstance *instance, USARTTxState_t *output);
/** @brief 启动阶段登记发送任务是否创建成功；首次提交前调用，不用于运行期恢复。
 * @param[in] ready 1为任务已创建，0为创建失败；有在途事务后不得修改。
 */
void USARTTxSetServiceReady(uint8_t ready);
/** @brief 调度器启动前创建唯一发送任务，普通启动上下文串行调用。
 * @return 1为已创建（重复调用不重复创建），0为上下文或资源错误；失败可在启动前重试。
 * @note 512个StackType_t、Normal优先级，约1ms推进；创建失败不注销接收对象。
 */
uint8_t USARTTxTaskInit(void);
/** @brief 仅专用发送任务调用；各路最多通知一条、启动一次，全局最多停止一次。
 * @note 禁止递归。回调和HAL等待均在临界区外；不承诺实际调度或线端耗时。
 */
void USARTProcessTx(void);

/**
 * @brief 按接收顺序派发入口时已排队的块，并执行有限错误恢复。
 * @note 全工程只有一个任务调用者（MotorControlTask），递归调用直接返回。
 * 每路四槽可用三槽，满时丢新块；每轮每路最多三块，新到块留到下轮。
 * 超过50ms或错误/重启前的块被丢弃。回调直接借用队列存储，返回前保持只读有效。
 * 不承诺物理耗时或派发延迟；协议自己承担拼帧、校验及来源时间检查。
 */
void USARTProcessRx(void);

#endif
