#ifndef MC02_HOST_FAKE_BSP_LOG_H
#define MC02_HOST_FAKE_BSP_LOG_H

void MC02TestLogError(void);
#define LOGERROR(...) MC02TestLogError()

#endif
