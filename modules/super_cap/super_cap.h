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

#pragma pack(1)
typedef struct
{
    uint16_t vol; // 原始电压整数，比例及单位待设备协议确认。
    uint16_t current; // 原始电流整数，尚未换算为 A。
    uint16_t power; // 原始功率整数，尚未换算为 W。
} SuperCap_Msg_s;
#pragma pack()

/* 超级电容实例 */
typedef struct
{
    CANInstance *can_ins; // CAN实例
    SuperCap_Msg_s cap_msg; // 超级电容信息
} SuperCapInstance;

/* 超级电容初始化配置 */
typedef struct
{
    CAN_Init_Config_s can_config;
} SuperCap_Init_Config_s;

/** @brief 注册唯一超级电容对象及其 CAN 接收回调。
* @param[in] supercap_config 有效配置，内部复制 CAN 配置并填写对象地址与回调。
* @return 成功返回长期有效对象，NULL、重复初始化或分配、CAN 注册失败返回 NULL。
 */
SuperCapInstance *SuperCapInit(SuperCap_Init_Config_s *supercap_config);

/** @brief 复制八字节控制数据到 CAN 实例并尝试发送，不报告发送结果。
* @param[in] instance 初始化成功的有效对象，不能为 NULL。
* @param[in] data 至少可读八字节；具体字段和比例由实际电容协议决定，本接口不换算。
 */
void SuperCapSend(SuperCapInstance *instance, uint8_t *data);

#endif // !SUPER_CAP_H
