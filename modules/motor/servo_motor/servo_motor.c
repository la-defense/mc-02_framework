/** @file servo_motor.c
* @brief 总线固定命令及PWM选择；长期实例通过ServoSetAngle复制提交，结果由任务通知。
*/
#include "servo_motor.h"
#include <stdlib.h>
#include <string.h>
#include <math.h>
uint8_t servo_angle_read[6]={0x55 ,0x55 ,0x04, 0x15 ,0x01 ,0x01 ,};
uint8_t servo_angle_write[16]={0x55 ,0x55, 0x08, 0x03, 0x01 ,0xF4 ,0x01 ,0x01 ,0x20 ,0x03,0x55 ,0x55 ,0x04, 0x15 ,0x01 ,0x01 ,};
uint8_t servo_unload[6]={0x55,0x55,0x04,0x14,0x01,0x01};

// ID表用于拒绝重复注册，并让稀疏编号保存在对应槽位。
static ServoInstance *servo_motor_instance[SERVO_MOTOR_CNT];
static void DecodeServo(void *context, const USART_RxBlock_t *block);
ServoInstance *ServoInit(const Servo_Init_Config_s *config)
{
    if (config == NULL || config->servo_id >= SERVO_MOTOR_CNT ||
        (config->servo_type != Bus_Servo && config->servo_type != PWM_Servo))
        return NULL;

    if ((config->servo_type == Bus_Servo && config->_handle == NULL) ||
        (config->servo_type == PWM_Servo && config->pwm_init_config.htim == NULL))
        return NULL;

    // 同一ID只能对应一个实例；先检查再申请，失败不会占用槽位。
    if (servo_motor_instance[config->servo_id] != NULL)
        return NULL;

    ServoInstance *servo = (ServoInstance *)malloc(sizeof(*servo));
    if (servo == NULL)
        return NULL;
    memset(servo, 0, sizeof(*servo));
    servo->servo_id = config->servo_id;
    servo->servo_type = config->servo_type;

    switch (config->servo_type)
    {
    case Bus_Servo:
    {
        USART_Init_Config_s usart_config = {
            .context = servo,
            .module_callback = DecodeServo,
            .recv_buff_size = Servo_MAX_BUFF,
            .usart_handle = config->_handle};
        servo->usart_instance = USARTRegister(&usart_config);
        if (servo->usart_instance == NULL)
            goto registration_failed;
        break;
    }
    case PWM_Servo:
    {
        // 底层注册接口接收可写配置；使用副本避免改动调用者保存的数据。
        PWM_Init_Config_s pwm_config = config->pwm_init_config;
        servo->pwm_instance = PWMRegister(&pwm_config);
        if (servo->pwm_instance == NULL)
            goto registration_failed;
        break;
    }
    default:
        free(servo);
        return NULL;
    }

    // 资源均就绪后才公开对象，接收回调从此可安全使用长期上下文。
    servo_motor_instance[servo->servo_id] = servo;
    return servo;

registration_failed:
    free(servo);
    return NULL;
}
//@todo PWM舵机的角度设置需要根据相应定时器PWM等参数进行计算(是否需要规范定时器PWM的初始化参数，以便于计算)
static void ServoTxCompleted(void *context,const USARTTxEvent_t *event)
{
    ServoInstance *servo=context;
    uint32_t saved=__get_PRIMASK();__disable_irq();servo->last_tx=*event;
    if(event->result!=USART_TX_COMPLETE)++servo->tx_failures;
    __set_PRIMASK(saved);
}
USARTSendStatus_t ServoSetAngle(ServoInstance *servo, float angle)
{
    if(servo==NULL || !isfinite(angle))return USART_SEND_BAD_ARGUMENT;
    if(servo->servo_type==PWM_Servo) {
        if(angle<0.0f || angle>1.0f || servo->pwm_instance==NULL)return USART_SEND_BAD_ARGUMENT;
        servo->angle=angle;PWMSetDutyRatio(servo->pwm_instance,angle);return USART_SEND_ACCEPTED;
    }
    if(servo->servo_type!=Bus_Servo || angle<0.0f || angle>65535.0f)return USART_SEND_BAD_ARGUMENT;
    // 保留旧帧和固定设备编号；MOTOR-06只覆盖注册和发送缓冲，设备协议仍待单独核实。
    uint8_t frame[16];memcpy(frame,servo_angle_write,sizeof(frame));
    uint16_t value=(uint16_t)angle;frame[8]=(uint8_t)value;frame[9]=(uint8_t)(value>>8u);
    USARTTxRequest_t request={frame,sizeof(frame),USART_TRANSFER_DMA,50u,ServoTxCompleted,NULL,servo};
    uint32_t id;return USARTSubmitTx(servo->usart_instance,&request,&id);
}
USARTSendStatus_t ServoGetTxState(const ServoInstance *servo, USARTTxEvent_t *event, uint32_t *failures)
{
    if (servo == NULL || event == NULL || failures == NULL) return USART_SEND_BAD_ARGUMENT;
    uint32_t saved = __get_PRIMASK(); __disable_irq();
    *event = servo->last_tx; *failures = servo->tx_failures;
    __set_PRIMASK(saved); return USART_SEND_ACCEPTED;
}

//@todo 只读取了角度 还有电压，动作是否完成等 且只支持一个串口
static void DecodeServo(void *context, const USART_RxBlock_t *block)
{
    // 这里只迁移已有查询响应，短块不读取位置字节；完整设备协议接入另行验证。
    ServoInstance *servo = context;
    if (servo == NULL || block->data == NULL || block->length < 8u) return;
    const uint8_t *data = block->data;
    if (data[0] == Servo_Frame_First && data[1] == Servo_Frame_Second && data[3] == 21u)
        servo->recv_angle = (uint16_t)((uint16_t)data[7] << 8u | data[6]);
}
