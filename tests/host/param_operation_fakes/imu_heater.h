#ifndef MC02_PARAM_OPERATION_FAKE_IMU_HEATER_H
#define MC02_PARAM_OPERATION_FAKE_IMU_HEATER_H

#include <stdint.h>

typedef enum
{
    IMU_HEATER_LONG_OPERATION_CALIBRATION = 0,
    IMU_HEATER_LONG_OPERATION_FLASH,
    IMU_HEATER_LONG_OPERATION_COUNT
} IMUHeater_LongOperation_e;

uint8_t IMUHeaterBeginLongOperation(IMUHeater_LongOperation_e operation);
void IMUHeaterEndLongOperation(IMUHeater_LongOperation_e operation, uint8_t success);

#endif
