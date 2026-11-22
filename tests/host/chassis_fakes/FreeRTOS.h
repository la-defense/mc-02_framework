#ifndef MC02_HOST_FREERTOS_H
#define MC02_HOST_FREERTOS_H
#if defined(CHASSIS_TEST_TRACK_CRITICAL)
extern unsigned chassis_test_critical_depth; // 生产平台入口会经过此处，记录模拟派发能否抢占。
#define taskENTER_CRITICAL() ((void)++chassis_test_critical_depth)
#define taskEXIT_CRITICAL() ((void)--chassis_test_critical_depth)
#else
#define taskENTER_CRITICAL() ((void)0)
#define taskEXIT_CRITICAL() ((void)0)
#endif
#endif
