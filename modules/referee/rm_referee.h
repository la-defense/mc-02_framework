/**
 * @file rm_referee.h
 * @brief 声明裁判帧接收、复制读取和串口发送接口。
 * 先注册唯一UART，再由任意任务复制读取；UI不参与接收初始化。
 * @author kidneygood (you@domain.com)
 * @version 0.1
 * @date 2022-11-18
 * @copyright Copyright (c) 2022
 */
#ifndef RM_REFEREE_H
#define RM_REFEREE_H

#include <stdint.h>
#include "usart.h"
#include "referee_snapshot.h"
#include "usart_tx.h"

#define REFEREE_LINK_TIMEOUT_MS 300u

/** @brief 按命令发送频率设置的期限；0x0002、0x0206、0x0207、0x0301为事件记录，不使用期限。 */
typedef struct {
    uint32_t timeout_10hz_ms; /**< 0x0201、0x0202、0x0208的有效期限，1～INT32_MAX ms。 */
    uint32_t timeout_3hz_ms;  /**< 0x0003、0x0204、0x0209、0x020A的有效期限，1～INT32_MAX ms。 */
    uint32_t timeout_1hz_ms;  /**< 1Hz命令的有效期限，1～INT32_MAX ms。 */
    uint32_t timeout_map_ms;  /**< 0x0303最近内容的有效期限，1～INT32_MAX ms。 */
} RefereeConfig_t;

/** @brief 初始化或快照读取的结果。 */
typedef enum {
    REFEREE_OK = 0,
    REFEREE_ERROR_ARGUMENT,
    REFEREE_ERROR_CONFIG,
    REFEREE_ERROR_ALREADY_INITIALIZED,
    REFEREE_ERROR_NOT_INITIALIZED,
    REFEREE_ERROR_RESOURCE
} RefereeStatus_t;

/** @brief 裁判串口提交结果；ACCEPTED只表示复制入队。 */
typedef enum {
    REFEREE_SEND_ACCEPTED = 0, REFEREE_SEND_BAD_ARGUMENT,
    REFEREE_SEND_NOT_INITIALIZED, REFEREE_SEND_BUSY,
    REFEREE_SEND_ERROR, REFEREE_SEND_TIMEOUT
} RefereeSendStatus_t;

/**
 * @brief 注册唯一裁判UART并启动跨块帧解析。
 * @param[in] referee_usart_handle 已初始化且长期有效的UART句柄，不能为NULL。
 * @param[in] config 可为NULL；非NULL时初始化复制，四个期限必须在1～INT32_MAX ms。
 * @return REFEREE_OK或具体参数、配置、重复初始化及资源错误；失败不覆盖已运行实例。
 * @note 默认期限为10Hz组300ms、3Hz组1000ms、1Hz组及0x0303为3000ms。UI可在接收成功后单独绑定。
 *       候选帧最长128字节，首字节RxEvent时间向后50ms内必须完成组帧，边界允许。
 */
RefereeStatus_t RefereeInit(UART_HandleTypeDef *referee_usart_handle, const RefereeConfig_t *config);

/**
 * @brief 一次复制全部命令数据、各自期限状态和链路诊断。
 * @param[out] output 接收完整副本；NULL返回参数错误。未初始化时清零整个可写输出。
 * @return REFEREE_OK、REFEREE_ERROR_ARGUMENT或REFEREE_ERROR_NOT_INITIALIZED。
 * @note 可从任务调用；复制期间接收发布暂时停止。查询不消费数据、不续期，周期数据必须检查state。
 *       事件命令只报告最近记录和年龄，不表示事件仍有效，也不会重放动作。
 */
RefereeStatus_t RefereeGetSnapshot(RefereeSnapshot_t *output);

/** @brief 将完整帧复制入队，无等待，不代表对端已收到。
 * @param[in] send 至少tx_len字节，成功后原数据可立即复用。
 * @param[in] tx_len 字节数1～128。
 * @param[in] complete 非NULL，专用发送任务通知一次最终结果，不得等待。
 * @param[in] guard 可NULL，启动前检查身份，不得等待。
 * @param[in] context 保存至完成通知结束的调用方对象。
 * @param[out] id 入队成功的非零编号；拒绝时保持原值。
 * @return ACCEPTED或参数、未初始化、队列满、后端错误；不自动重发。
 * @note 先成功RefereeInit，普通任务调用；总期限100ms，达到即到期。
 */
RefereeSendStatus_t RefereeSend(const uint8_t *send, uint16_t tx_len,
    USARTTxCallback_t complete, USARTTxGuard_t guard, void *context, uint32_t *id);
/** @brief 请求取消本模块提交的编号；实际停止和通知由发送任务执行。
 * @param[in] id 成功提交的非零编号。
 * @return 与USARTCancelTx相同；不能撤回已经发出的字节。
 */
USARTSendStatus_t RefereeCancelSend(uint32_t id);
/** @brief 复制裁判UART的最近完成结果和累计失败数，不推进发送。
 * @param[out] output 非NULL；failed含错误、超时、取消和隔离，last_event.id为0表示尚无通知。
 * @return ACCEPTED、NOT_READY或BAD_ARGUMENT；失败保持原输出。
 * @note 普通任务调用；接收数据与发送诊断独立保存。
 */
USARTSendStatus_t RefereeGetTxState(USARTTxState_t *output);
/** @brief 用0x0201本组期限检查机器人身份，普通任务可调用。
 * @param[in] robot_id 需要匹配的线上机器人编号。
 * @return 1为已收到、仍新鲜且编号相同，否则0；不会续期或消费数据。
 */
uint8_t RefereeIdentityMatches(uint16_t robot_id);
#endif
