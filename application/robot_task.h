/** @file robot_task.h
* @brief 任务创建及常驻任务实现，仅由 robot.c 包含；重复包含到其他编译单元会重复定义。
 */
/* 注意该文件应只用于任务初始化,只能被robot.c包含*/
#pragma once

#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

#include "robot.h"
#include "ins_task.h"
#include "motor_task.h"
#include "dji_motor.h"
#include "referee_task.h"
#include "master_process.h"
#include "daemon.h"
#include "HT04.h"
#include "buzzer.h"
#include "task_monitor.h"
#include "lcd_task.h"
#include "bsp_crash.h"
#include "bsp_safety.h"
#include "imu_heater.h"
#include "init_diagnostic.h"
#include "robot_safety.h"

#include "bsp_log.h"
#include "bsp_dwt.h"
#include "bsp_spi.h"
#include "bsp_adc.h"

osThreadId insTaskHandle;
osThreadId robotTaskHandle;
osThreadId motorTaskHandle;
osThreadId daemonTaskHandle;
osThreadId uiTaskHandle;
osThreadId lcdTaskHandle;

/** @brief 初始化 INS 后约每 1 ms 处理采样和姿态；失败时只维持诊断循环。
* @param[in] argument 未使用的 RTOS 任务参数，可以为 NULL；仅用作任务入口。
 */
void StartINSTASK(void const *argument);
/** @brief 约每 1 ms 派发通信并运行电机计算。
* @param[in] argument 未使用的 RTOS 任务参数，可以为 NULL；仅用作任务入口。
 */
void StartMOTORTASK(void const *argument);
/** @brief 约每 10 ms 检查设备和任务，全部正常时才喂狗。
* @param[in] argument 未使用的 RTOS 任务参数，可以为 NULL；仅用作任务入口。
 */
void StartDAEMONTASK(void const *argument);
/** @brief 约每 5 ms 运行整车控制和诊断。
* @param[in] argument 未使用的 RTOS 任务参数，可以为 NULL；仅用作任务入口。
 */
void StartROBOTTASK(void const *argument);
#if MC02_ENABLE_REFEREE_UI
/** @brief 等待新鲜裁判机器人编号并周期运行 UI；每次循环至少等待 1 ms。
* @param[in] argument 未使用的 RTOS 任务参数，可以为 NULL；仅用作任务入口。
 */
void StartUITASK(void const *argument);
#endif

/** @brief 创建 INS、电机、Daemon、整车、LCD 及当前配置所需 UI 任务。
* @return 1 为全部创建成功，0 为至少一个失败，并记录失败任务编号；不删除已创建的任务。
* @note 由 RobotInit() 在调度器启动前调用一次。
 */
uint8_t OSTaskInit(void)
{
    osThreadDef(instask, StartINSTASK, osPriorityAboveNormal, 0, 1024);
    insTaskHandle = osThreadCreate(osThread(instask), NULL); // 由于是阻塞读取传感器,为姿态解算设置较高优先级,确保以1khz的频率执行
    // // 后续修改为读取传感器数据准备好的中断处理,

    // 电机任务还会派发接收并打印日志，栈容量包含回调和格式化所需空间。
    osThreadDef(motortask, StartMOTORTASK, osPriorityNormal, 0, 512);
    motorTaskHandle = osThreadCreate(osThread(motortask), NULL);

    // Daemon 内有状态检查与日志，需同时容纳日志行数组和各层调用。
    osThreadDef(daemontask, StartDAEMONTASK, osPriorityNormal, 0, 512);
    daemonTaskHandle = osThreadCreate(osThread(daemontask), NULL);

    osThreadDef(robottask, StartROBOTTASK, osPriorityNormal, 0, 1024);
    robotTaskHandle = osThreadCreate(osThread(robottask), NULL);

#if MC02_ENABLE_REFEREE_UI
    osThreadDef(uitask, StartUITASK, osPriorityNormal, 0, 512);
    uiTaskHandle = osThreadCreate(osThread(uitask), NULL);
#endif

    osThreadDef(lcdtask, StartLCDTASK, osPriorityLow, 0, 1024);
    lcdTaskHandle = osThreadCreate(osThread(lcdtask), NULL);
    uint8_t all_created = 1u;
    if (insTaskHandle == NULL)
    {
        InitDiagnosticReportFailure(INIT_STAGE_TASKS, INIT_MODULE_TASKS,
                                    INIT_FAILURE_TASK_CREATE, INIT_RESOURCE_TASK_INS);
        all_created = 0u;
    }
    if (motorTaskHandle == NULL)
    {
        InitDiagnosticReportFailure(INIT_STAGE_TASKS, INIT_MODULE_TASKS,
                                    INIT_FAILURE_TASK_CREATE, INIT_RESOURCE_TASK_MOTOR);
        all_created = 0u;
    }
    if (daemonTaskHandle == NULL)
    {
        InitDiagnosticReportFailure(INIT_STAGE_TASKS, INIT_MODULE_TASKS,
                                    INIT_FAILURE_TASK_CREATE, INIT_RESOURCE_TASK_DAEMON);
        all_created = 0u;
    }
    if (robotTaskHandle == NULL)
    {
        InitDiagnosticReportFailure(INIT_STAGE_TASKS, INIT_MODULE_TASKS,
                                    INIT_FAILURE_TASK_CREATE, INIT_RESOURCE_TASK_ROBOT);
        all_created = 0u;
    }
#if MC02_ENABLE_REFEREE_UI
    if (uiTaskHandle == NULL)
    {
        InitDiagnosticReportFailure(INIT_STAGE_TASKS, INIT_MODULE_TASKS,
                                    INIT_FAILURE_TASK_CREATE, INIT_RESOURCE_TASK_UI);
        all_created = 0u;
    }
#endif
    if (lcdTaskHandle == NULL)
    {
        InitDiagnosticReportFailure(INIT_STAGE_TASKS, INIT_MODULE_TASKS,
                                    INIT_FAILURE_TASK_CREATE, INIT_RESOURCE_TASK_LCD);
        all_created = 0u;
    }

    if (all_created)
    {
#if !defined(MC02_PROFILE_BENCH_SAFE)
        HTMotorControlInit(); // 没有注册HT电机则不会执行
#endif
    }
    return all_created;
}

__attribute__((noreturn)) void StartINSTASK(void const *argument)
{
    static volatile float ins_start;
    static float ins_dt;
    TaskMonitorFeed(TASK_MONITOR_INS);
    if (INS_Init() == NULL)
    {
        InitDiagnosticReportFailure(INIT_STAGE_APPLICATION, INIT_MODULE_INS,
                                    INIT_FAILURE_ALGORITHM_MEMORY, 0u);
        RobotFaultReport(ROBOT_FAULT_INIT_FAILURE);
        IMUHeaterForceOff();
        BSP_SafetyStartupLatchOutputsOff();
        // 算法初始化失败后保持输出关闭，但继续更新诊断和本任务记录，供人工读取。
        for (;;)
        {
            InitDiagnosticHeartbeat();
            TaskMonitorFeed(TASK_MONITOR_INS);
            osDelay(100u);
        }
    }
    InitDiagnosticMarkReady();
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
            // 用整数微秒记录超时，每秒最多报告一次，保留总次数和省略次数。
            static LogRateLimit_t rl_ins_overrun = {0};
            if (LogRateLimitAllow(&rl_ins_overrun, 1000u))
            {
                ins_start = DWT_GetTimeline_ms(); /* 便于 Live Watch 看最近一次超时刻 */
                (void)ins_start;
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
    uint8_t buzzer_ready = BuzzerInit();
    if (!buzzer_ready)
    {
        InitDiagnosticReportFailure(INIT_STAGE_TASKS, INIT_MODULE_BUZZER,
                                    INIT_FAILURE_RESOURCE, 0u);
        RobotFaultReport(ROBOT_FAULT_INIT_FAILURE);
        IMUHeaterForceOff();
        BSP_SafetyStartupLatchOutputsOff();
    }
    LOGINFO("[freeRTOS] Daemon Task Start");
#if LOG_TEST
    /* 日志格式串安全自测(默认关闭): 见 bsp_log.h 的 LOG_TEST / bsp_log.c 的 LogSelfTest()
       启动先打一次, 之后每 2s 重打一次(100Hz x 200), 方便临时接上 RTT 抓取。 */
    LogSelfTest();
#endif
#if CAN_SELFTEST_LOOPBACK
    /* FDCAN 回环自测(默认关闭): 必须在所有设备注册完成之后跑(否则 CANRegister 会重新
       配置总线把自测配置覆盖掉), 放在任务上下文里也便于用 HAL_GetTick 做超时 */
    CANRunLoopbackSelfTest();
#endif
#if CRASH_TEST
    /* 崩溃自测(默认关闭)的等待计数: 见下面循环里的用法 */
    static uint32_t s_crash_test_wait = 0u;
#endif
#if CAN_DJI_INJECT_TEST >= 1
    /* DJI 假反馈注入自测(默认关闭): 验证"中断入队 → CANProcessRx 派发 → 解码 → 多圈累加" */
    (void)CANRunDjiFeedbackInjectTest();
#endif
    for (;;)
    {
        // 100Hz
        TaskMonitorFeed(TASK_MONITOR_DAEMON);
        uint32_t daemon_probe = DWT_ProbeStart();
#if LOG_TEST
        static uint32_t s_log_test_tick = 0u;
        if (++s_log_test_tick >= 200u)
        {
            s_log_test_tick = 0u;
            LogSelfTest();
        }
#endif
        // 约每 10 ms 停止出错或到期的异步 SPI，停止失败的总线保持隔离。
        (void)SPIProcessTimeouts();
        // 电压诊断只记录读取结果，内部每 250ms 更新一次，不参与控制许可。
        BSP_ADCUpdateDiagnostics();
        DaemonTask();
        if (buzzer_ready)
            BuzzerTask();
        CANHealthMonitor(); /* 10Hz: 检测 BusOff 并恢复(空总线/异常时防止 CAN 永久死掉) */
        TaskMonitorTick();
#if CRASH_TEST
        // 自测开启时按轮数等待其他任务启动，每轮仍完成监控和喂狗；到数后触发一种异常。
        if (++s_crash_test_wait >= CRASH_TEST_DELAY_LOOPS)
            CrashLogSelfTest(); /* 不返回: 记录现场后主动复位 */
#endif
#if CAN_DJI_INJECT_TEST >= 2
        DJIInjectStressTick(); /* 压测模式: 持续往接收队列里灌帧 */
#endif
        daemon_dt = (float)DWT_ProbeElapsedUs(daemon_probe);
        if (daemon_dt > 10000.0f)
        {
            static LogRateLimit_t rl_daemon_overrun = {0};
            if (LogRateLimitAllow(&rl_daemon_overrun, 1000u))
            {
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
                LOGERROR("[freeRTOS] ROBOT core Task 超时: %luus (累计 %lu 次, 期间限速 %lu 条)",
                         (unsigned long)(uint32_t)robot_dt,
                         (unsigned long)rl_robot_overrun.total, (unsigned long)rl_robot_overrun.dropped);
            }
        }
        osDelay(5);
    }
}

#if MC02_ENABLE_REFEREE_UI
__attribute__((noreturn)) void StartUITASK(void const *argument)
{
    (void)argument;
    LOGINFO("[freeRTOS] UI Task Start");
    RefereeSnapshot_t snapshot;
    // 只等0x0201自己的期限内数据，其他裁判命令不能替它提供机器人身份。
    while (RefereeGetSnapshot(&snapshot) == REFEREE_OK &&
           snapshot.command[REF_CMD_ROBOT_STATE].state != REFEREE_DATA_FRESH)
        osDelay(100);
    UIStatus_t ui_status = MyUIInit();
    if (ui_status == UI_SUBMITTED)
        LOGINFO("[freeRTOS] UI initial frames accepted locally");
    else
        LOGERROR("[freeRTOS] UI disabled after init failure: %d", (int)ui_status);
    for (;;)
    {
        // 每给裁判系统发送一包数据会挂起一次,详见UITask函数的refereeSend()
        (void)UITask(); // 失败保留待刷新项；初始化失败时此入口不会提交数据。
        osDelay(1); // 即使没有任何UI需要刷新,也挂起一次,防止卡在UITask中无法切换
    }
}
#endif
