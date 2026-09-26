#include "motor_task.h"
#include "LK9025.h"
#include "HT04.h"
#include "dji_motor.h"
#include "step_motor.h"
#include "servo_motor.h"
#include "xmmotor.h"
#include "bsp_can.h"
#include "bsp_usart.h"
#include "bsp_usb.h"

/* ---------------- 性能探针(2026-09, CPU 占用排查) ---------------- */
volatile DWT_Probe_t motor_prof_all = {0};
volatile DWT_Probe_t motor_prof_dji = {0};
volatile DWT_Probe_t motor_prof_lk = {0};

void MotorControlTask()
{
    // static uint8_t cnt = 0; 设定不同电机的任务频率
    // if(cnt%5==0) //200hz
    // if(cnt%10==0) //100hz
    uint32_t probe_all = DWT_ProbeStart();
    uint32_t probe_seg;

    /* 先把本周期到达的 CAN 接收帧派发掉(解码/滤波/多圈累加), 再算控制:
       这样拿到的仍是最新反馈, 端到端延迟和"在中断里直接解析"相同。
       注意: 这里是全工程唯一的接收队列消费者, 不要在别处再调用 CANProcessRx()。 */
    CANProcessRx();
    /* 串口与 VCP 同理: 中断只入队, 解析(裁判帧/视觉帧)在这里逐帧做。
       三个消费者都放在 1kHz 任务里, 解析延迟 ≤1ms。 */
    USARTProcessRx();
    VCPProcessRx();

    probe_seg = DWT_ProbeStart();
    DJIMotorControl();
    DWT_ProbeDone(&motor_prof_dji, probe_seg);

    /* 如果有对应的电机则取消注释,可以加入条件编译或者register对应的idx判断是否注册了电机 */
    probe_seg = DWT_ProbeStart();
    LKMotorControl();
    DWT_ProbeDone(&motor_prof_lk, probe_seg);

    // XMMotorControl();

    // legacy support
    // 由于ht04电机的反馈方式为接收到一帧消息后立刻回传,以此方式连续发送可能导致总线拥塞
    // 为了保证高频率控制,HTMotor中提供了以任务方式启动控制的接口,可通过宏定义切换
    // HTMotorControl();
    // 将所有的CAN设备集中在一处发送,最高反馈频率仅能达到500Hz,为了更好的控制效果,应使用新的HTMotorControlInit()接口

    // StepMotorControl();
    DWT_ProbeDone(&motor_prof_all, probe_all);
}
