#include "bsp_init.h"
#include "robot.h"
#include "robot_def.h"
#include "robot_task.h"
#include "robot_safety.h"
#include "task_monitor.h"
#include "bsp_watchdog.h"
#include "master_process.h"
#include "bsp_param.h"
#include "imu_heater.h"

static uint8_t RobotParamFlashOperationBegin(void)
{
    if (!IMUHeaterBeginLongOperation(IMU_HEATER_LONG_OPERATION_FLASH))
        return 0u;
    if (!TaskMonitorBeginLongOperation(TASK_MONITOR_INS, TASK_MONITOR_LONG_OPERATION_FLASH))
    {
        IMUHeaterEndLongOperation(IMU_HEATER_LONG_OPERATION_FLASH, 0u);
        return 0u;
    }
    return 1u;
}

static uint8_t RobotParamFlashOperationEnd(uint8_t success)
{
    uint8_t within_window = TaskMonitorEndLongOperation(TASK_MONITOR_INS,
                                                        TASK_MONITOR_LONG_OPERATION_FLASH);
    uint8_t operation_success = (uint8_t)(success && within_window);
    IMUHeaterEndLongOperation(IMU_HEATER_LONG_OPERATION_FLASH, operation_success);
    return operation_success;
}

// 编译warning,提醒开发者修改机器人参数
#ifndef ROBOT_DEF_PARAM_WARNING
#define ROBOT_DEF_PARAM_WARNING
#pragma message "check if you have configured the parameters in robot_def.h, IF NOT, please refer to the comments AND DO IT, otherwise the robot will have FATAL ERRORS!!!"
#endif // !ROBOT_DEF_PARAM_WARNING

#if defined(ONE_BOARD) || defined(CHASSIS_BOARD)
#include "chassis.h"
#endif

#if defined(ONE_BOARD) || defined(GIMBAL_BOARD)
#include "gimbal.h"
#include "shoot.h"
#include "robot_cmd.h"
#endif


void RobotInit()
{  
    // 关闭中断,防止在初始化过程中发生中断
    // 请不要在初始化过程中使用中断和延时函数！
    // 若必须,则只允许使用DWT_Delay()
    __disable_irq();
    
    BSPInit();
    RobotSafetyInit();
    TaskMonitorInit();
    const ParamLongOperationGuard_t param_guard = {
        .begin = RobotParamFlashOperationBegin,
        .end = RobotParamFlashOperationEnd,
    };
    ParamSetLongOperationGuard(&param_guard);

#if defined(ONE_BOARD) || defined(GIMBAL_BOARD)
    RobotCMDInit();
    GimbalInit();
    ShootInit();
#endif

#if defined(MC02_PROFILE_BENCH_SAFE)
    VisionInit(&huart9);
#endif

#if defined(ONE_BOARD) || defined(CHASSIS_BOARD)
    ChassisInit();
#endif

    OSTaskInit(); // 创建基础任务

    // 所有任务创建完成后再启动IWDG, 由Daemon任务中的TaskMonitorTick统一喂狗
    BSP_WatchdogInit(200);

    // 初始化完成,开启中断
    __enable_irq();
}

void RobotTask()
{
#if defined(MC02_PROFILE_BENCH_SAFE)
    RobotSafetyUpdate();
    VisionSend();
#else
#if defined(ONE_BOARD) || defined(GIMBAL_BOARD)
    RobotCMDTask();
    GimbalTask();
    ShootTask();
#endif

#if defined(ONE_BOARD) || defined(CHASSIS_BOARD)
    ChassisTask();
#endif
#endif

}
