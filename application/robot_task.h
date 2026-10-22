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
#include "bsp_usart.h"

#if defined(MC02_PROFILE_BENCH_SAFE) || defined(MC02_PROFILE_ROBOT)
/* 保留完整任务轮次最大耗时，供固定只读快照与任务栈数据一起核对。 */
volatile DWT_Probe_t r25_task_ins = {0};
volatile DWT_Probe_t r25_task_motor = {0};
volatile DWT_Probe_t r25_task_daemon = {0};
volatile DWT_Probe_t r25_task_robot = {0};
#endif

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

/** @brief 创建INS、电机、UART发送、Daemon、整车、LCD及当前配置所需UI任务。
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

    uint8_t uart_tx_created = USARTTxTaskInit();

    /* 日志任务是辅助诊断能力，创建失败由日志只读状态报告，不阻断核心任务启动。 */
    (void)BSPLogTaskInit();

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
    if (!uart_tx_created) {
        InitDiagnosticReportFailure(INIT_STAGE_TASKS, INIT_MODULE_TASKS,
            INIT_FAILURE_TASK_CREATE, INIT_RESOURCE_TASK_UART_TX);
        all_created = 0u;
    }
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
    BSP_LOG_EVENT_0(EV_APPLICATION_ROBOT_TASK_H_01);
    for (;;)
    {
        // 1kHz
        TaskMonitorFeed(TASK_MONITOR_INS);
        /* 用轻量周期计数代替 DWT_GetTimeline_ms(): 后者内部有 64bit 除法, 1kHz 下也不便宜 */
        uint32_t ins_probe = DWT_ProbeStart();
        INS_Task();
        DWT_ProbeDone(&r25_task_ins, ins_probe);
        if (r25_task_ins.last_us > 1000u)
        {
            ins_start = DWT_GetTimeline_ms(); /* 便于 Live Watch 查看最近一次超时刻 */
            (void)ins_start;
            BSP_LOG_EVENT_1(EV_APPLICATION_ROBOT_TASK_H_02,
                            BSP_LOG_FIELD_U32(r25_task_ins.last_us));
        }
        // 视觉上行数据由 INS_Task 内的 VisionUpdateTx() 填充,
        // VisionSend() 统一在 RobotCMDTask(200Hz) 中调用, 避免多任务并发 DMA 发送
        osDelay(1);
    }
}

__attribute__((noreturn)) void StartMOTORTASK(void const *argument)
{
    BSP_LOG_EVENT_0(EV_APPLICATION_ROBOT_TASK_H_03);
    for (;;)
    {
        TaskMonitorFeed(TASK_MONITOR_MOTOR);
        uint32_t motor_probe = DWT_ProbeStart();
        MotorControlTask();
        DWT_ProbeDone(&r25_task_motor, motor_probe);
        if (r25_task_motor.last_us > 1000u)
        {
            BSP_LOG_EVENT_1(EV_APPLICATION_ROBOT_TASK_H_04,
                            BSP_LOG_FIELD_U32(r25_task_motor.last_us));
        }
        osDelay(1);
    }
}

__attribute__((noreturn)) void StartDAEMONTASK(void const *argument)
{
    uint8_t buzzer_ready = BuzzerInit();
    if (!buzzer_ready)
    {
        InitDiagnosticReportFailure(INIT_STAGE_TASKS, INIT_MODULE_BUZZER,
                                    INIT_FAILURE_RESOURCE, 0u);
        RobotFaultReport(ROBOT_FAULT_INIT_FAILURE);
        IMUHeaterForceOff();
        BSP_SafetyStartupLatchOutputsOff();
    }
    BSP_LOG_EVENT_0(EV_APPLICATION_ROBOT_TASK_H_05);
#if LOG_TEST
    /* 类型化日志自测默认关闭；普通事件也需在bench_safe显式开启诊断输出。 */
    LogSelfTest();
#endif
#if CAN_SELFTEST_LOOPBACK
    /* FDCAN 回环自测(默认关闭): 必须在所有设备注册完成之后跑(否则 CANRegister 会重新
       配置总线把自测配置覆盖掉), 放在任务上下文里也便于用 HAL_GetTick 做超时 */
    CANRunLoopbackSelfTest();
#endif
#if MC02_BENCH_CRASH_DIAGNOSTICS
    /* 专用固件启动稳定后才检查许可；许可本身须由匹配Build ID的工具写入。 */
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
#if MC02_BENCH_CRASH_DIAGNOSTICS
        // 给其他任务留出启动时间；无许可时检查无副作用，有许可时只消费一次后复位。
        if (s_crash_test_wait < 100u)
            ++s_crash_test_wait;
        else
            CrashLogSelfTest();
#endif
#if CAN_DJI_INJECT_TEST >= 2
        DJIInjectStressTick(); /* 压测模式: 持续往接收队列里灌帧 */
#endif
        DWT_ProbeDone(&r25_task_daemon, daemon_probe);
        if (r25_task_daemon.last_us > 10000u)
        {
            BSP_LOG_EVENT_1(EV_APPLICATION_ROBOT_TASK_H_06,
                            BSP_LOG_FIELD_U32(r25_task_daemon.last_us));
        }
        osDelay(10);
    }
}

__attribute__((noreturn)) void StartROBOTTASK(void const *argument)
{
    BSP_LOG_EVENT_0(EV_APPLICATION_ROBOT_TASK_H_07);
    // 200Hz-500Hz,若有额外的控制任务如平衡步兵可能需要提升至1kHz
    for (;;)
    {
        TaskMonitorFeed(TASK_MONITOR_ROBOT);
        uint32_t robot_probe = DWT_ProbeStart();
        RobotTask();
        DWT_ProbeDone(&r25_task_robot, robot_probe);
        if (r25_task_robot.last_us > 5000u)
        {
            BSP_LOG_EVENT_1(EV_APPLICATION_ROBOT_TASK_H_08,
                            BSP_LOG_FIELD_U32(r25_task_robot.last_us));
        }
        osDelay(5);
    }
}

#if MC02_ENABLE_REFEREE_UI
__attribute__((noreturn)) void StartUITASK(void const *argument)
{
    (void)argument;
    // 只登记一次请求；每轮最多提交一帧，身份失效后由显式MyUIInit恢复。
    (void)MyUIInit();
    for (;;)
    {
        (void)UITask();
        osDelay(1u);
    }
}
#endif
