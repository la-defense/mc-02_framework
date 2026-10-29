#ifndef MC02_HOST_CHASSIS_SUPER_CAP_H
#define MC02_HOST_CHASSIS_SUPER_CAP_H
#include "main.h"
typedef struct { int unused; } SuperCapInstance;
typedef struct { struct { CAN_HandleTypeDef *can_handle; uint32_t tx_id, rx_id; } can_config; } SuperCap_Init_Config_s;
SuperCapInstance *SuperCapInit(const SuperCap_Init_Config_s *config);
#endif
