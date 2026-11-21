#ifndef MC02_HOST_CHASSIS_REFEREE_TASK_H
#define MC02_HOST_CHASSIS_REFEREE_TASK_H
#include "robot_def.h"
#if defined(MC02_VISION_PROTOCOL_V2)
#include "rm_referee.h" // V2 底盘使用实际裁判来源读口的声明。
#else
typedef struct { int unused; } Referee_Interactive_info_t;
typedef struct { unsigned timeout_10hz_ms; } RefereeConfig_t;
typedef enum { REFEREE_OK=0, REFEREE_ERROR_RESOURCE } RefereeStatus_t;
RefereeStatus_t RefereeInit(PlatformUartPort_t port, const RefereeConfig_t *config);
#endif
#endif
