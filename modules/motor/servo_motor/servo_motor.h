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
}ServoInstance;

/** @brief 按类型注册 UART 或 PWM，并保存到本地实例表。
 * @param[in] Servo_Init_Config 有效配置；servo_id 须从 0 连续编号且小于 7，底层外设须先配置。
 * @return 分配对象；当前未检查容量、内存或底层注册失败，不提供可靠的失败返回。
 * @note 总线接收只支持当前单 UART 方式；公共命令帧内设备编号固定，尚未按 servo_id 改写发送编号。
 */
ServoInstance *ServoInit(Servo_Init_Config_s *Servo_Init_Config);
/** @brief 按类型更新舵机输出，接口不返回发送结果。
 * @param[in,out] servo 已注册且所需底层实例有效的对象，不能为 NULL。
 * @param[in] angle 总线类型按可转为 uint16_t 的 0～65535 原始位置写入，实际角度比例须查设备协议；
 *                  PWM 类型直接作为 0～1 占空比，不做角度换算。
 * @note 总线发送共用数组，DMA 完成前不可再次修改；接入设备前还须核对当前固定命令帧。
 */
void ServoSetAngle(ServoInstance *servo, float angle);
#endif // SERVO_MOTOR_H
