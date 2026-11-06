/** @file imu_heater.h
 * @brief INS 启动测试的温控状态和关断替身。
 */
#ifndef MC02_INS_STARTUP_TEST_HEATER_H
#define MC02_INS_STARTUP_TEST_HEATER_H
#include <stdint.h>
uint8_t IMUHeaterInitSucceeded(void);
void IMUHeaterForceOff(void);
#endif
