/* 注意该文件应只用于任务初始化,只能被robot.c包含*/
#pragma once

#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

#include "robot.h"
#include "ins_task.h"
#include "motor_task.h"
#include "referee_task.h"
#include "master_process.h"
#include "daemon.h"
#include "HT04.h"
#include "buzzer.h"
#include "task_monitor.h"
#include "lcd_task.h"

#include "bsp_log.h"

osThreadId insTaskHandle;
osThreadId robotTaskHandle;
osThreadId motorTaskHandle;
osThreadId daemonTaskHandle;
osThreadId uiTaskHandle;
osThreadId lcdTaskHandle;

void StartINSTASK(void const *argument);
void StartMOTORTASK(void const *argument);
void StartDAEMONTASK(void const *argument);
void StartROBOTTASK(void const *argument);
void StartUITASK(void const *argument);

/**
 * @brief 初始化机器人任务,所有持续运行的任务都在这里初始化
 *
 */
void OSTaskInit()
{
    osThreadDef(instask, StartINSTASK, osPriorityAboveNormal, 0, 1024);
    insTaskHandle = osThreadCreate(osThread(instask), NULL); // 由于是阻塞读取传感器,为姿态解算设置较高优先级,确保以1khz的频率执行
    // // 后续修改为读取传感器数据准备好的中断处理,

    osThreadDef(motortask, StartMOTORTASK, osPriorityNormal, 0, 256);
    motorTaskHandle = osThreadCreate(osThread(motortask), NULL);

    osThreadDef(daemontask, StartDAEMONTASK, osPriorityNormal, 0, 128);
    daemonTaskHandle = osThreadCreate(osThread(daemontask), NULL);

    osThreadDef(robottask, StartROBOTTASK, osPriorityNormal, 0, 1024);
    robotTaskHandle = osThreadCreate(osThread(robottask), NULL);

    osThreadDef(uitask, StartUITASK, osPriorityNormal, 0, 512);
    uiTaskHandle = osThreadCreate(osThread(uitask), NULL);

    osThreadDef(lcdtask, StartLCDTASK, osPriorityLow, 0, 1024);
    lcdTaskHandle = osThreadCreate(osThread(lcdtask), NULL);

    HTMotorControlInit(); // 没有注册HT电机则不会执行
}

__attribute__((noreturn)) void StartINSTASK(void const *argument)
{
    static float ins_start;
    static float ins_dt;
    INS_Init(); // 确保BMI088被正确初始化.
    LOGINFO("[freeRTOS] INS Task Start");
    for (;;)
    {
        // 1kHz
        TaskMonitorFeed(TASK_MONITOR_INS);
        /* 用轻量周期计数代替 DWT_GetTimeline_ms(): 后者内部有 64bit 除法, 1kHz 下也不便宜 */
        uint32_t ins_probe = DWT_ProbeStart();
        INS_Task();
        ins_dt = (float)DWT_ProbeElapsedUs(ins_probe);
        if (ins_dt > 1000.0f)
        {
            /* 原实现把 &ins_dt(指针)当 %f 打印, RTT 也不支持浮点 → 日志一直是坏的;
               现在改成整数微秒 + 限速(否则 1kHz 超时会刷爆 RTT 缓冲) */
            static LogRateLimit_t rl_ins_overrun = {0};
            if (LogRateLimitAllow(&rl_ins_overrun, 1000u))
            {
                ins_start = DWT_GetTimeline_ms(); /* 保留: 便于 Live Watch 看最近一次时刻 */
                LOGERROR("[freeRTOS] INS Task 超时: %luus (累计 %lu 次, 期间限速 %lu 条)",
                         (unsigned long)(uint32_t)ins_dt,
                         (unsigned long)rl_ins_overrun.total, (unsigned long)rl_ins_overrun.dropped);
            }
        }
        // 视觉上行数据由 INS_Task 内的 VisionUpdateTx() 填充,
        // VisionSend() 统一在 RobotCMDTask(200Hz) 中调用, 避免多任务并发 DMA 发送
        osDelay(1);
    }
}

__attribute__((noreturn)) void StartMOTORTASK(void const *argument)
{
    static float motor_dt;
    static float motor_start;
    LOGINFO("[freeRTOS] MOTOR Task Start");
    for (;;)
    {
        TaskMonitorFeed(TASK_MONITOR_MOTOR);
        uint32_t motor_probe = DWT_ProbeStart();
        MotorControlTask();
        motor_dt = (float)DWT_ProbeElapsedUs(motor_probe);
        if (motor_dt > 1000.0f)
        {
            static LogRateLimit_t rl_motor_overrun = {0};
            if (LogRateLimitAllow(&rl_motor_overrun, 1000u))
            {
                motor_start = DWT_GetTimeline_ms();
                LOGERROR("[freeRTOS] MOTOR Task 超时: %luus (累计 %lu 次, 期间限速 %lu 条)",
                         (unsigned long)(uint32_t)motor_dt,
                         (unsigned long)rl_motor_overrun.total, (unsigned long)rl_motor_overrun.dropped);
            }
        }
        osDelay(1);
    }
}

__attribute__((noreturn)) void StartDAEMONTASK(void const *argument)
{
    static float daemon_dt;
    static float daemon_start;
    BuzzerInit();
    LOGINFO("[freeRTOS] Daemon Task Start");
#if CAN_SELFTEST_LOOPBACK
    /* FDCAN 回环自测(默认关闭): 必须在所有设备注册完成之后跑(否则 CANRegister 会重新
       配置总线把自测配置覆盖掉), 放在任务上下文里也便于用 HAL_GetTick 做超时 */
    CANRunLoopbackSelfTest();
#endif
    for (;;)
    {
        // 100Hz
        TaskMonitorFeed(TASK_MONITOR_DAEMON);
        uint32_t daemon_probe = DWT_ProbeStart();
        DaemonTask();
        BuzzerTask();
        CANHealthMonitor(); /* 10Hz: 检测 BusOff 并恢复(空总线/异常时防止 CAN 永久死掉) */
        TaskMonitorTick();
        daemon_dt = (float)DWT_ProbeElapsedUs(daemon_probe);
        if (daemon_dt > 10000.0f)
        {
            static LogRateLimit_t rl_daemon_overrun = {0};
            if (LogRateLimitAllow(&rl_daemon_overrun, 1000u))
            {
                daemon_start = DWT_GetTimeline_ms();
                LOGERROR("[freeRTOS] Daemon Task 超时: %luus (累计 %lu 次, 期间限速 %lu 条)",
                         (unsigned long)(uint32_t)daemon_dt,
                         (unsigned long)rl_daemon_overrun.total, (unsigned long)rl_daemon_overrun.dropped);
            }
        }
        osDelay(10);
    }
}

__attribute__((noreturn)) void StartROBOTTASK(void const *argument)
{
    static float robot_dt;
    static float robot_start;
    LOGINFO("[freeRTOS] ROBOT core Task Start");
    // 200Hz-500Hz,若有额外的控制任务如平衡步兵可能需要提升至1kHz
    for (;;)
    {
        TaskMonitorFeed(TASK_MONITOR_ROBOT);
        uint32_t robot_probe = DWT_ProbeStart();
        RobotTask();
        robot_dt = (float)DWT_ProbeElapsedUs(robot_probe);
        if (robot_dt > 5000.0f)
        {
            static LogRateLimit_t rl_robot_overrun = {0};
            if (LogRateLimitAllow(&rl_robot_overrun, 1000u))
            {
                robot_start = DWT_GetTimeline_ms();
                LOGERROR("[freeRTOS] ROBOT core Task 超时: %luus (累计 %lu 次, 期间限速 %lu 条)",
                         (unsigned long)(uint32_t)robot_dt,
                         (unsigned long)rl_robot_overrun.total, (unsigned long)rl_robot_overrun.dropped);
            }
        }
        osDelay(5);
    }
}

__attribute__((noreturn)) void StartUITASK(void const *argument)
{
    LOGINFO("[freeRTOS] UI Task Start");
    MyUIInit();
    LOGINFO("[freeRTOS] UI Init Done, communication with ref has established");
    for (;;)
    {
        // 每给裁判系统发送一包数据会挂起一次,详见UITask函数的refereeSend()
        UITask();
        osDelay(1); // 即使没有任何UI需要刷新,也挂起一次,防止卡在UITask中无法切换
    }
}
