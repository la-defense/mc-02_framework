#ifndef MC02_HOST_CHASSIS_REFEREE_TASK_H
#define MC02_HOST_CHASSIS_REFEREE_TASK_H
#include "robot_def.h"
typedef struct { int unused; } referee_info_t;
typedef struct { int unused; } Referee_Interactive_info_t;
referee_info_t *UITaskInit(UART_HandleTypeDef *handle, Referee_Interactive_info_t *ui);
#endif
