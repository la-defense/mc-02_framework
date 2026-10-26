/** @file servo_motor.h
* @brief 保留的总线舵机与 PWM 舵机接口；两种类型的 angle 参数含义不同，不能统一按度使用。
*/
#ifndef SERVO_MOTOR_H
#define SERVO_MOTOR_H

#include "main.h"
#include "tim.h"
#include <stdint.h>
#include "bsp_pwm.h"
#include "bsp_usart.h"

#define SERVO_MOTOR_CNT 7
#define Servo_Frame_First 0x55
#define Servo_Frame_Second 0x55
#define Servo_MAX_BUFF 10
#define SERVO_MOVE_CMD 0x03
#define SERVO_UNLOAD_CMD 0x14
#define SERVO_POS_READ_CMD 0x15
typedef enum
{
    Servo_None_Type = 0,
    Bus_Servo = 1,
    PWM_Servo = 2,
}ServoType_e;


/* 用于初始化不同舵机的结构体,各类舵机通用 */
typedef struct
{
    PWM_Init_Config_s pwm_init_config;
    ServoType_e servo_type;
    UART_HandleTypeDef *_handle;
    uint8_t servo_id;
}Servo_Init_Config_s;
typedef struct
{   
    uint8_t servo_id;
    float angle;
    uint16_t recv_angle;
    PWMInstance *pwm_instance;
    USARTInstance *usart_instance;
    ServoType_e servo_type;
    USARTTxEvent_t last_tx;
    uint32_t tx_failures;
}ServoInstance;

/** @brief 在启动阶段注册 UART 或 PWM 舵机并保存长期实例。
 * @param[in] Servo_Init_Config 非空配置；servo_id 范围为0～6且不得重复，所选底层外设须先配置。
 * @return 成功返回长期有效实例；参数、ID、内存或底层注册失败返回NULL，配置及已有实例保持不变。
 * @note 仅限调度器启动前串行注册；实例无运行期注销接口。总线帧仍保留固定设备编号，实际协议对应关系未在此修复。
*/
ServoInstance *ServoInit(const Servo_Init_Config_s *Servo_Init_Config);
/** @brief 更新PWM占空比或复制总线命令入队。
 * @param[in,out] servo 已初始化实例，NULL返回参数错误；须保存到发送完成。
 * @param[in] angle PWM为0～1占空比，总线为0～65535有限原始位置数值。
 * @return 本机接受或明确提交错误；总线接受不表示传输完成，last_tx记录最终事件。
 * @note 仅普通任务调用；总期限50ms。总线固定设备编号仍未修复，真实比例与动作尚未验收。
 */
USARTSendStatus_t ServoSetAngle(ServoInstance *servo, float angle);
/** @brief 复制本实例最近的总线发送结果；查询不推进发送。
 * @param[in] servo 长期有效实例，不能为NULL。
 * @param[out] event 非NULL，id为0表示尚未收到通知。
 * @param[out] failures 非NULL，失败通知累计次数。
 * @return ACCEPTED或BAD_ARGUMENT；失败保留输出。
 */
USARTSendStatus_t ServoGetTxState(const ServoInstance *servo, USARTTxEvent_t *event, uint32_t *failures);
#endif
