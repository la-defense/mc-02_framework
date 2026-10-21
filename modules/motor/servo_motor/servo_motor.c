/** @file servo_motor.c
* @brief 总线固定命令及PWM选择；长期实例通过ServoSetAngle复制提交，结果由任务通知。
*/
#include "servo_motor.h"
#include "stdlib.h"
#include "memory.h"
#include <math.h>
#include "bsp_log.h"
uint8_t servo_angle_read[6]={0x55 ,0x55 ,0x04, 0x15 ,0x01 ,0x01 ,};
uint8_t servo_angle_write[16]={0x55 ,0x55, 0x08, 0x03, 0x01 ,0xF4 ,0x01 ,0x01 ,0x20 ,0x03,0x55 ,0x55 ,0x04, 0x15 ,0x01 ,0x01 ,};
uint8_t servo_unload[6]={0x55,0x55,0x04,0x14,0x01,0x01};

// 本地表保留现有注册方式；接收回调只访问注册时传入的舵机对象。
static ServoInstance *servo_motor_instance[SERVO_MOTOR_CNT];
static uint8_t servo_idx = 0; // register servo_idx,是该文件的全局舵机索引,在注册时使用
static void DecodeServo(void *context, const USART_RxBlock_t *block);
// 通过此函数注册一个舵机
ServoInstance *ServoInit(Servo_Init_Config_s *Servo_Init_Config)
{
    ServoInstance *servo = (ServoInstance *)malloc(sizeof(ServoInstance));
    memset(servo, 0, sizeof(ServoInstance));
    USART_Init_Config_s config = {0};
    config.context = servo;
    servo->servo_type = Servo_Init_Config->servo_type;
    // 总线舵机注册串口接收，PWM 舵机注册定时器；未知类型只记录错误。
    switch (Servo_Init_Config->servo_type)
    {
    case Bus_Servo:
        config.module_callback = DecodeServo;
        config.recv_buff_size = Servo_MAX_BUFF;
        config.usart_handle = Servo_Init_Config->_handle;
        servo->usart_instance = USARTRegister(&config);
        break;
    case PWM_Servo:
        servo->pwm_instance = PWMRegister(&Servo_Init_Config->pwm_init_config);
        break;
    default:
        LOGERROR("Servo type error");
        break;
    }
    servo->servo_id = Servo_Init_Config->servo_id;
    servo_idx++;
    servo_motor_instance[servo->servo_id] = servo;

    return servo;
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
    // 保留旧帧和固定设备编号；本次只修复异步数组寿命，协议接入仍由MOTOR-06跟踪。
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
