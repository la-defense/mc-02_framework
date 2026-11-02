/** @file FreeRTOS.h
 * @brief 温控主机测试不执行并发任务；供平台时钟适配器编译。
 */
#define taskENTER_CRITICAL() ((void)0)
#define taskEXIT_CRITICAL() ((void)0)
