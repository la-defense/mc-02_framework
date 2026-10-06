#ifndef MC02_PARAM_OPERATION_FAKE_LOG_H
#define MC02_PARAM_OPERATION_FAKE_LOG_H

void MC02ParamOperationLog(void);
#define LOGERROR(...) MC02ParamOperationLog()
#define LOGWARNING(...) MC02ParamOperationLog()
#define LOGINFO(...) MC02ParamOperationLog()

#endif
