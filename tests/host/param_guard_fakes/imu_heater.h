/** @file imu_heater.h
 * @brief 参数维护测试使用的最小温控保护接口。
 */
#include <stdint.h>
typedef enum { IMU_HEATER_LONG_OPERATION_FLASH = 1 } IMUHeater_LongOperation_e;
uint8_t IMUHeaterBeginLongOperation(IMUHeater_LongOperation_e operation);
void IMUHeaterEndLongOperation(IMUHeater_LongOperation_e operation, uint8_t success);
