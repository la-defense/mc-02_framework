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
    /* 先用一个长超时启动看门狗: IWDG 一旦启动, 只有上电复位才会停止 —— 系统复位后它
       仍按原来的 200ms 计时, 而启动流程(BMI088 在线标定 ~4s)远长于 200ms, 于是
       "一次看门狗复位"会变成"永远起不来"的死循环。启动阶段用长超时兜住冒烟测试,
       等 RTOS 起来后再切回 200ms(BSP_WatchdogInit(200), 见 robot.c)。 */
    /* 启动阶段先用一个够长的超时启用看门狗: IWDG 一旦启动, 只有上电复位才会停,
       系统复位后它仍按上次配置计时; 而启动流程(BMI088 在线标定 3~15s)远长于运行期
       的 200ms, 于是"一次看门狗复位"会演变成"永远起不来"的死循环。
       长耗时步骤(如标定循环)里会主动 BSP_WatchdogFeed() 报进度。 */
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

