#ifndef IMU_HEATER_H
#define IMU_HEATER_H

#include <stdint.h>
#include "bmi088_data.h"

#ifndef MC02_HEATER_ENABLED
#define MC02_HEATER_ENABLED 0
#endif

typedef struct
{
    float target_temp;
    float temperature;
    uint16_t duty;          // 当前 PWM 比较值(0~ARR)
    uint8_t sensor_valid;   // 本次温度是否可信
    uint8_t fault;          // 加热器已锁存故障
    uint8_t overtemp;       // 过温锁存
    uint8_t timeout_fault;  // 长时间加热未达目标
    uint8_t heating;        // 当前是否在加热
} IMUHeaterStatus_t;

typedef enum
{
    IMU_HEATER_LONG_OPERATION_CALIBRATION = 0,
    IMU_HEATER_LONG_OPERATION_FLASH,
    IMU_HEATER_LONG_OPERATION_COUNT
} IMUHeater_LongOperation_e;

void IMUHeaterInit(void);
void IMUHeaterUpdate(const BMI088_Data_t *sample, uint8_t force_off);
void IMUHeaterForceOff(void);
uint8_t IMUHeaterBeginLongOperation(IMUHeater_LongOperation_e operation);
void IMUHeaterEndLongOperation(IMUHeater_LongOperation_e operation, uint8_t success);
void IMUHeaterClearFault(void);
void IMUHeaterGetStatus(IMUHeaterStatus_t *status);
void IMUHeaterSetTarget(float target_temp);

#endif // !IMU_HEATER_H
