/** @file commissioning_chassis_cmd.h
 * @brief 受限调试用的单板人工底盘命令入口。
 */
#ifndef MC02_COMMISSIONING_CHASSIS_CMD_H
#define MC02_COMMISSIONING_CHASSIS_CMD_H

#include "config.h"

/** @brief 登记底盘命令发布者，保存已确认的人工作用配置。
 * @param[in] input 启动结束后仍可只读访问的车型输入配置。
 * @return 1 为成功；无效映射、非底盘参照或登记失败时返回 0。
 */
uint8_t CommissioningChassisCmdInit(const RobotManualInputConfig_s *input);

/** @brief 处理一次人工使能手势、反馈时效和受限底盘目标。
 * @note 仅在模块和任务已放行后，由整车任务单独调用。
 */
void CommissioningChassisCmdTask(void);

#endif
