/** @file motor_task.c
* @brief 统一接收派发顺序和电机周期计算，提供各段耗时记录。
*/
#include "motor_task.h"
#include "bsp_dwt.h"
#include "LK9025.h"
#include "HT04.h"
#include "dji_motor.h"
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

    // 先按顺序解码已排队的反馈，再使用这些测量值计算；不要另建并发派发调用者。
    CANProcessRx();
    // UART 和 USB 也在这里运行模块解析，避免将协议计算放在中断里；排队过时的数据由 BSP 跳过。
    USARTProcessRx();
    VCPProcessRx();

    probe_seg = DWT_ProbeStart();
#if !defined(MC02_PROFILE_BENCH_SAFE)
    DJIMotorControl();
    DWT_ProbeDone(&motor_prof_dji, probe_seg);

    // 当前 LKMotorControl() 已启用；无注册对象时它不会提交发送。
    probe_seg = DWT_ProbeStart();
    LKMotorControl();
    DWT_ProbeDone(&motor_prof_lk, probe_seg);
// 裸板配置仅做通信派发，跳过电机控制和指令发送。
#else
    (void)probe_seg;
#endif

    // XMMotorControl();

    // legacy support
    // 由于ht04电机的反馈方式为接收到一帧消息后立刻回传,以此方式连续发送可能导致总线拥塞
    // 为了保证高频率控制,HTMotor中提供了以任务方式启动控制的接口,可通过宏定义切换
    // HTMotorControl();
    // 将所有的CAN设备集中在一处发送,最高反馈频率仅能达到500Hz,为了更好的控制效果,应使用新的HTMotorControlInit()接口

    // StepMotorControl();
    DWT_ProbeDone(&motor_prof_all, probe_all);
}
