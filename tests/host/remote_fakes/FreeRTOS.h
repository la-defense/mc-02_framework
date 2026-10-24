#ifndef MC02_HOST_FREERTOS_H
#define MC02_HOST_FREERTOS_H
#ifdef MC02_THREAD_CRITICAL
void HostTaskCriticalEnter(void);
void HostTaskCriticalExit(void);
#define taskENTER_CRITICAL() HostTaskCriticalEnter()
#define taskEXIT_CRITICAL() HostTaskCriticalExit()
#else
#define taskENTER_CRITICAL() ((void)0)
#define taskEXIT_CRITICAL() ((void)0)
#endif
#endif
