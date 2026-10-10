#ifndef REMOTE_SAFETY_H
#define REMOTE_SAFETY_H
#include "remote_control.h"
typedef struct { uint32_t start; uint8_t active; } RemoteHold_s;
typedef struct { RemoteHold_s enable, disable, reset; } RemoteSafetyGestures_s;
void RemoteSafetyGesturesInit(RemoteSafetyGestures_s *state);
/* allow_enable is false in the bench diagnostic. Offline resets all holds. */
void RemoteSafetyGesturesUpdate(RemoteSafetyGestures_s *state, const RC_ctrl_t *input,
                               uint8_t online, uint32_t now, uint8_t allow_enable);
#endif
