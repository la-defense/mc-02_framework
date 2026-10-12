#ifndef MC02_HOST_CHASSIS_INS_TASK_H
#define MC02_HOST_CHASSIS_INS_TASK_H
#include <stdint.h>
typedef struct { float Gyro[3], Accel[3], Roll, Pitch, Yaw, YawTotalAngle; } attitude_t;
#endif
