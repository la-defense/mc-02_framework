#ifndef IMU_HEATER_H
#define IMU_HEATER_H

#include <stdint.h>

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

void IMUHeaterInit(void);
void IMUHeaterUpdate(float temperature, uint8_t sensor_valid, uint8_t force_off);
void IMUHeaterForceOff(void);
void IMUHeaterClearFault(void);
void IMUHeaterGetStatus(IMUHeaterStatus_t *status);
void IMUHeaterSetTarget(float target_temp);

#endif // !IMU_HEATER_H
