#ifndef MC02_CAN_COMM_TEST_DAEMON_H
#define MC02_CAN_COMM_TEST_DAEMON_H
#include <stdint.h>
typedef struct { int unused; } DaemonInstance;
typedef struct { void (*callback)(void *); void *owner_id; uint16_t reload_count; } Daemon_Init_Config_s;
DaemonInstance *DaemonRegister(Daemon_Init_Config_s *config);
uint8_t DaemonUnregister(DaemonInstance *instance);
void DaemonReload(DaemonInstance *instance);
uint8_t DaemonIsOnline(DaemonInstance *instance);
#endif
