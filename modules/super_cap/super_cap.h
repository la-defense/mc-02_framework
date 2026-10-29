/*
* @Descripttion: 超级电容 CAN 收发；反馈按高字节在前组合原始整数，比例和设备单位尚待确认。
 * @version:
 * @Author: Chenfu
 * @Date: 2022-12-02 21:32:47
 * @LastEditTime: 2022-12-05 15:25:46
 */
#ifndef SUPER_CAP_H
#define SUPER_CAP_H

#include "bsp_can.h"
#include <stddef.h>

typedef struct
{
    uint16_t vol; // 原始电压整数，比例及单位待设备协议确认。
    uint16_t current; // 原始电流整数，尚未换算为 A。
    uint16_t power; // 原始功率整数，尚未换算为 W。
} SuperCap_Msg_s;
/** @brief 唯一实例由模块持有，调用方不能自行释放或修改。 */
typedef struct SuperCapInstance SuperCapInstance;

/** @brief 最近帧是否在调用方给定的期限内。 */
typedef enum {
    SUPERCAP_FEEDBACK_UNSEEN = 0,
    SUPERCAP_FEEDBACK_FRESH,
    SUPERCAP_FEEDBACK_EXPIRED
} SuperCapFeedbackState_t;
typedef enum { SUPERCAP_OK = 0, SUPERCAP_ERROR_ARGUMENT } SuperCapStatus_t;
typedef struct {
    SuperCap_Msg_s value; /**< 未经设备协议换算的原始整数。 */
    uint32_t source_rx_ms; /**< CAN队列接收到该帧时的本机毫秒数，0有效。 */
    uint32_t age_ms; /**< 与调用方提供的当前时间之差。 */
    SuperCapFeedbackState_t state;
} SuperCapFeedback_t;

/* 超级电容初始化配置 */
typedef struct
{
    CAN_Init_Config_s can_config;
} SuperCap_Init_Config_s;

/** @brief 注册唯一超级电容对象及其 CAN 接收回调。
* @param[in] supercap_config 非空配置，CAN句柄必须长期有效；内部复制配置并填写对象地址与回调。
* @return 成功返回长期有效对象；参数无效、重复初始化或分配、CAN注册失败返回NULL。
* @note 须在接收派发任务运行前串行初始化；成功对象由模块长期保存，无注销入口。
 */
SuperCapInstance *SuperCapInit(const SuperCap_Init_Config_s *supercap_config);

/** @brief 提交八字节控制内容，CAN调度器在提交期间复制。
* @param[in] instance 初始化成功的长期有效对象，不能为 NULL。
* @param[in] data 至少可读length字节；具体字段和比例由实际电容协议决定。
* @param[in] length 缓冲长度，必须恰为8字节。
* @return CAN提交结果；接受仅表示本机已复制，不代表设备收到或执行。
* @note 仅普通任务调用；中断等不允许提交的上下文由CAN调度器返回相应错误。
 */
CAN_TxSubmitStatus_t SuperCapSend(SuperCapInstance *instance, const uint8_t *data, size_t length);

/** @brief 复制最近一次有效反馈并按调用方期限检查新鲜度。
* @param[in] instance 初始化成功的实例，不能为空。
* @param[in] now_ms 当前本机毫秒时间，与CAN接收时钟同源；允许回绕。
* @param[in] max_age_ms 允许的最大年龄，范围1～INT32_MAX毫秒，边界允许。
* @param[out] output 非空时接收副本；失败保留原内容。
* @return SUPERCAP_OK或SUPERCAP_ERROR_ARGUMENT。
* @note 普通任务调用；无反馈及过期值不得用作电容供电许可，查询不续期。
*       当前32位时间差无法识别反馈停更超过完整计数周期（约49.7天），
*       因此FRESH仅用于诊断，不能单独作为功率授权。
 */
SuperCapStatus_t SuperCapGet(const SuperCapInstance *instance, uint32_t now_ms,
                            uint32_t max_age_ms, SuperCapFeedback_t *output);

#endif // !SUPER_CAP_H
