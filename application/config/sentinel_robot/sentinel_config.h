/** @file sentinel_config.h
* @brief 当前哨兵的静态配置入口；配置的含义和单位见 config.h。
*/
#include "config.h"

/** @brief 返回哨兵机械和电机参数，不执行硬件初始化。
* @return 内部静态对象地址，长期有效、不可释放；重复调用返回同一对象，不重置字段。
*/
Robot_Config_s *SentinelConfigInit(void);
