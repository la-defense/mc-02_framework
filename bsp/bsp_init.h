#ifndef BSP_INIT_h
#define BSP_INIT_h

#include "bsp_init.h"
#include "bsp_log.h"
#include "bsp_dwt.h"
#include "bsp_watchdog.h"
#include "bsp_adc.h"
#include "bsp_param.h"
#include "bsp_crash.h"
//#include "bsp_usb.h"

/**
 * @brief bsp层初始化统一入口,这里仅初始化必须的bsp组件,其他组件的初始化在各自的模块中进行
 *        需在实时系统启动前调用,目前由RobotoInit()调用
 * 
 * @note 其他实例型的外设如CAN和串口会在注册实例的时候自动初始化,不注册不初始化
  */
// 
void BSPInit()
{
    DWT_Init(480);
    /* IWDG 在普通系统复位后继续运行。启动早期使用 4s 窗口，RTOS 建好任务后由
       RobotInit 切到 200ms。标定在 INS 任务中运行：TaskMonitor 只在其他任务健康时
       喂狗，并把 INS 标定豁免限制在 15s；标定循环不直接喂狗。 */
    BSP_WatchdogInit(4000);
    BSPLogInit();
    BSP_ADCInit();
    BSP_WatchdogLogResetReason();
    /* 上次运行如果是崩溃(异常/断言/栈溢出)导致的, 会把现场打印出来 ——
       必须在日志初始化之后调用。 */
    CrashLogInit();
    /* 通用参数区读取(内部 Flash 扇区 6/7, A/B 双份): BMI088 标定值等
       需要掉电保持的参数都放这里。幂等, 后面各模块再调用也只是直接返回上次结果。 */
    ParamInit();
}


#endif // !BSP_INIT_h

