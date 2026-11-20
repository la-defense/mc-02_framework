#ifndef MC02_HOST_FREERTOS_H
#define MC02_HOST_FREERTOS_H
#if defined(MC02_MANUAL_GENERATED_TEST)
void RobotCmdTestCriticalEnter(void);
void RobotCmdTestCriticalExit(void);
#define taskENTER_CRITICAL() RobotCmdTestCriticalEnter() // 生成配置测试模拟任务的嵌套临界区。
#define taskEXIT_CRITICAL() RobotCmdTestCriticalExit() // 只在最外层退出时安排另一个任务。
#else
#define taskENTER_CRITICAL() ((void)0)
#define taskEXIT_CRITICAL() ((void)0)
#endif
#endif
