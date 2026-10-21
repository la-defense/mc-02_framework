/* 测试桩沿用生产事件目录；此类调用方测试关闭日志副作用。 */
#undef DISABLE_LOG_SYSTEM
#define DISABLE_LOG_SYSTEM 1
#include "../../../bsp/log/bsp_log.h"
/* 这些模块测试只验证业务结果；logger 核心在独立生产接口测试中验证。 */
#undef BSP_LOG_EVENT_0
#undef BSP_LOG_EVENT_1
#undef BSP_LOG_EVENT_2
#undef BSP_LOG_EVENT_3
#undef BSP_LOG_EVENT_4
#undef BSP_LOG_EVENT_5
#undef BSP_LOG_EVENT_6
#undef BSP_LOG_EVENT_7
#undef BSP_LOG_EVENT_8
#define BSP_LOG_EVENT_0(...) ((void)0)
#define BSP_LOG_EVENT_1(...) ((void)0)
#define BSP_LOG_EVENT_2(...) ((void)0)
#define BSP_LOG_EVENT_3(...) ((void)0)
#define BSP_LOG_EVENT_4(...) ((void)0)
#define BSP_LOG_EVENT_5(...) ((void)0)
#define BSP_LOG_EVENT_6(...) ((void)0)
#define BSP_LOG_EVENT_7(...) ((void)0)
#define BSP_LOG_EVENT_8(...) ((void)0)
#ifndef MC02_CAN_COMM_TEST_LOG_H
#define MC02_CAN_COMM_TEST_LOG_H
#define LOGWARNING(...) ((void)0)
#endif
