/** @file robot_param_guard.h
 * @brief 整车参数 Flash 操作的任务监控与可选温控保护。
 */
#ifndef MC02_ROBOT_PARAM_GUARD_H
#define MC02_ROBOT_PARAM_GUARD_H
#include <stdint.h>

/** @brief 在 Flash 维护开始前申请必要保护。
 * @return 所有必要保护成功返回 1，否则返回 0；不允许重叠操作。
 */
uint8_t RobotParamFlashOperationBegin(void);
/** @brief 结束本次保护并检查任务监控期限。
 * @param[in] success Flash 和看门狗操作均成功时为非零。
 * @return 操作及期限均成功返回 1；没有对应开始或任一失败返回 0。
 */
uint8_t RobotParamFlashOperationEnd(uint8_t success);
#endif
