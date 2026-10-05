#ifndef MC02_HOST_BMI088_FAKE_BSP_PARAM_H
#define MC02_HOST_BMI088_FAKE_BSP_PARAM_H

#include <stdint.h>

#define PARAM_KEY_IMU_GYRO_OFFSET 0x0100u
#define PARAM_KEY_IMU_G_NORM 0x0101u
#define PARAM_KEY_IMU_CALIB_META 0x0102u
#define PARAM_IMU_CALIB_OK 1u
typedef struct
{
    uint32_t result;
    float temperature;
    uint32_t time_s;
} ParamImuCalibMeta_t;

uint8_t ParamInit(void);
uint8_t ParamGet(uint16_t key, void *buf, uint16_t len);
uint8_t ParamGetFloat(uint16_t key, float *out);
uint8_t ParamGetFloats(uint16_t key, float *out, uint8_t count);
uint8_t ParamSet(uint16_t key, const void *buf, uint16_t len);
uint8_t ParamSetFloat(uint16_t key, float value);
uint8_t ParamSetFloats(uint16_t key, const float *in, uint8_t count);
uint8_t ParamCommit(void);

#endif
