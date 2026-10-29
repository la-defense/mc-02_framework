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
#include "referee_tx.h"

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

/**
 * @brief 复制最近一条合法机器人交互内容及其接收信息。
 * @param[out] output 非空时接收副本；未初始化时清零，空指针返回参数错误。
 * @return REFEREE_OK、REFEREE_ERROR_ARGUMENT或REFEREE_ERROR_NOT_INITIALIZED。
 * @note 普通任务调用，查询不消费记录或刷新时间；received为0表示从未接收。
 *       内容ID为0x0200～0x02FF，长度为0～112字节。后来的合法记录会覆盖旧记录，
 *       业务自行判断来源年龄及是否允许使用；数据不携带服务器生成时间或对端确认。
 *       接收检查合法编号、同队及新鲜本机接收目标，尚未按比赛阶段限制角色收发方向。
 */
RefereeStatus_t RefereeGetInteraction(RefereeInteractionRecord_t *output);

/** @brief 用0x0201本组期限检查机器人身份，普通任务可调用。
 * @param[in] robot_id 需要匹配的线上机器人编号。
 * @return 1为已收到、仍新鲜且编号相同，否则0；不会续期或消费数据。
 */
uint8_t RefereeIdentityMatches(uint16_t robot_id);
#endif
