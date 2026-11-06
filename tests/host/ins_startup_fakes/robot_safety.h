/** @file robot_safety.h
 * @brief INS 启动测试的整车安全接口替身。
 */
#ifndef MC02_INS_STARTUP_TEST_SAFETY_H
#define MC02_INS_STARTUP_TEST_SAFETY_H
#define ROBOT_FAULT_INIT_FAILURE 1u
void RobotFaultReport(unsigned fault);
void RobotSafetyRequestDisable(void);
#endif
