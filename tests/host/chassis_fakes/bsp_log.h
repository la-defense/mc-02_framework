#ifndef MC02_HOST_CHASSIS_BSP_LOG_H
#define MC02_HOST_CHASSIS_BSP_LOG_H
void MC02ChassisTestLogError(void);
#define LOGERROR(...) MC02ChassisTestLogError()
#define LOGINFO(...) ((void)0)
#endif
