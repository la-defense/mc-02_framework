/** @file ht_numeric_test.c
 * @brief 通过生产HT04登记的接收回调验证五点平均、独立历史及失败不续期。
 * @note CAN、任务和时钟只在内存中模拟，初始化不会向真实电机发送命令。
 */
#include "HT04.h"
#include "test_check.h"
#include "bsp_dwt.h"
#include "cmsis_os.h"
#include "../../bsp/os/platform_sync.h"
#include "../../bsp/dwt/platform_probe.h"
#include <math.h>
#include <string.h>

static PlatformCanRxPort_t *ports[2]; // 保存测试中注册的长期端点。
static DaemonInstance daemons[2];
static unsigned registered,daemon_count,reloads,clock_reads;
static uint32_t test_tick;

uint8_t PlatformCanBusIsBound(PlatformCanBus_t bus)
{
    return (uint8_t)(bus == PLATFORM_CAN_BUS_1 || bus == PLATFORM_CAN_BUS_2);
}
uint8_t PlatformCanRxRegister(PlatformCanRxPort_t *port, const PlatformCanEndpoint_t *endpoint,
                              PlatformCanRxCallback_t callback, void *context)
{
    TEST_CHECK(port != NULL && endpoint != NULL && registered < 2u);
    port->backend = port;
    port->callback = callback;
    port->context = context;
    ports[registered++] = port;
    return 1u;
}
static CAN_TxSubmitStatus_t fake_tx_submit(const CAN_TxFrame_t *frame, uint32_t *request_id)
{
    TEST_CHECK(frame != NULL && request_id != NULL && frame->length <= 8u);
    *request_id = 1u;
    return CAN_TX_SUBMIT_ACCEPTED;
}
CAN_TxSubmitStatus_t PlatformCanSubmitPeriodic(PlatformCanBus_t bus,const void *source,uint32_t key,
    const CAN_TxFrame_t *frame,CAN_TxPriority_t priority,uint32_t deadline_ms,uint32_t period_us,
    CAN_TxPeriodicPrepareFn_t prepare,uint32_t *request_id)
{(void)bus;(void)source;(void)key;(void)priority;(void)deadline_ms;(void)period_us;(void)prepare;return fake_tx_submit(frame,request_id);}
CAN_TxSubmitStatus_t PlatformCanSubmitOneShot(PlatformCanBus_t bus,const void *source,
    const CAN_TxFrame_t *frame,CAN_TxPriority_t priority,uint32_t deadline_ms,uint32_t *request_id)
{(void)bus;(void)source;(void)priority;(void)deadline_ms;return fake_tx_submit(frame,request_id);}
DaemonInstance *DaemonRegister(const Daemon_Init_Config_s *config)
{(void)config;TEST_CHECK(daemon_count<2);return &daemons[daemon_count++];}
void DaemonReload(DaemonInstance *instance){(void)instance;++reloads;}
DaemonStatus_t DaemonReportAt(DaemonInstance *instance,uint32_t received_ms,uint8_t flags)
{(void)instance;(void)received_ms;if((flags&DAEMON_REPORT_TRANSPORT_VALID)!=0u)++reloads;return DAEMON_OK;}
uint8_t DaemonUnregister(DaemonInstance *instance){(void)instance;return 1u;}
uint8_t DaemonIsOnline(DaemonInstance *instance){(void)instance;return 1u;}
uint64_t DWT_GetTimeline_us(void){return 0;}
float DWT_GetDeltaT(uint32_t *counter){++clock_reads;++*counter;return .001f;}
void DWT_Delay(float seconds){(void)seconds;}
void PlatformHtLegacyDelayMs(uint32_t milliseconds) { (void)milliseconds; } // 不在主机实际等待。
uint32_t HAL_GetTick(void){return test_tick;}
void PlatformTaskCriticalEnter(void) {} // 本测试串行交错两台电机的反馈。
void PlatformTaskCriticalExit(void) {}
uint32_t PlatformTimeNowMs(void) { return HAL_GetTick(); }
uint64_t PlatformTimeNowUs(void) { return DWT_GetTimeline_us(); }
float PlatformProbeDeltaSeconds(uint32_t *counter) { return DWT_GetDeltaT(counter); }
void *osThreadCreate(const osThreadDef_t *definition,void *argument)
{(void)definition;(void)argument;return NULL;}
void osDelay(uint32_t milliseconds){(void)milliseconds;}
char *__itoa(int value,char *output,int base)
{TEST_CHECK(value>=0 && value<=3 && base==10);output[0]=(char)('0'+value);output[1]=0;return output;}

// 只构造协议反馈并调用登记的生产回调，不触发控制任务。
static void receive(unsigned index,uint16_t speed)
{
    PlatformCanRxFrame_t frame = {.length = 6u, .received_ms = test_tick};
    uint8_t *bytes=frame.data;
    bytes[1]=0x80;bytes[2]=0;
    bytes[3]=(uint8_t)(speed>>4);bytes[4]=(uint8_t)((speed&15u)<<4);
    ports[index]->callback(ports[index]->context, &frame);
}

/** @brief 交错喂入两组速度，再损坏历史检查整帧保持及修复后恢复。 */
int main(void)
{
    Motor_Init_Config_s config={0};
    config.motor_type=HT04;
    config.can_init_config.can_bus=PLATFORM_CAN_BUS_1; // 两个测试实例使用第一条总线。
    config.output_axis=(Motor_Output_Axis_Config_s){1.0f,MOTOR_AXIS_DIRECTION_NORMAL};
    HTMotorInstance *a=HTMotorInit(&config),*b=HTMotorInit(&config);
    TEST_CHECK(a!=NULL && b!=NULL);
    double history[2][5]={{0}};
    const uint16_t raw[2]={4095,0};
    // 每个实例的窗口只包含自己的数据，启动仍保留5项零填充的权重。
    for(unsigned n=0;n<7;++n)
        for(unsigned index=0;index<2;++index)
        {
            for(size_t j=0;j<4;++j)history[index][j]=history[index][j+1];
            history[index][4]=(double)raw[index]*90.0/4095.0-45.0-HT_SPEED_BIAS;
            receive(index,raw[index]);
            double expected=0;for(size_t j=0;j<5;++j)expected+=history[index][j]/5.0;
            HTMotorInstance *motor=index==0?a:b;
            TEST_CHECK(fabs(motor->measure.speed_rads-expected)<2e-5);
        }
    TEST_CHECK(reloads==14 && clock_reads==14);
    const uint32_t previous_sequence = a->feedback_sequence;
    const uint32_t previous_time = a->feedback_timestamp_ms;
    PlatformCanRxFrame_t short_frame = {.length = 5u, .received_ms = 70u};
    ports[0]->callback(ports[0]->context, &short_frame); // 少于六字节时不得刷新来源。
    TEST_CHECK(a->feedback_sequence == previous_sequence && a->feedback_timestamp_ms == previous_time);
    TEST_CHECK(reloads == 14u && clock_reads == 14u);
    a->measure.speed_buff[0]=NAN;
    HTMotor_Measure_t before=a->measure;
    receive(0,2048);
    TEST_CHECK(memcmp(&a->measure,&before,sizeof(before))==0);
    TEST_CHECK(reloads==14 && clock_reads==14);
    memset(a->measure.speed_buff,0,sizeof(a->measure.speed_buff));
    test_tick = 77u; // 交付帧中保存的原始接收时间。
    receive(0,4095);
    TEST_CHECK(reloads==15 && clock_reads==15 && a->measure.feed_cnt==8);
    TEST_CHECK(a->feedback_timestamp_ms == 77u);
    TEST_CHECK(fabsf(a->measure.speed_rads-(45.0f-HT_SPEED_BIAS)/5)<2e-5f);
    return 0;
}
