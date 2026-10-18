/** @file ins_hardware.h
 * @brief 仅供INS主机回归的采样、时钟和传输声明；生产KF与DSP保持真实。
 */
#ifndef MC02_INS_HARDWARE_FAKE_H
#define MC02_INS_HARDWARE_FAKE_H
#include <stdint.h>
#include "bmi088_data.h"
typedef struct { unsigned cycles; } DWT_Probe_t;
float DWT_GetDeltaT(uint32_t *counter);
uint32_t DWT_ProbeStart(void);
void DWT_ProbeDone(volatile DWT_Probe_t *p,uint32_t start);
uint32_t HAL_GetTick(void);
#define LOGERROR(...) ((void)0)
#define RAD_2_DEGREE 57.295779513f
#define CS2_GYRO_GPIO_Port 0
#define CS2_GYRO_Pin 0
#define CS2_ACCEL_GPIO_Port 0
#define CS2_ACCEL_Pin 0
#define SPI_BLOCK_MODE 0
#define BMI088_BLOCK_PERIODIC_MODE 0
#define BMI088_CALIBRATE_ONLINE_MODE 0
#define BMI088_ACQUIRE_OK 1
typedef struct { int unused; } BMI088Instance;
typedef struct { void *spi_handle; int GPIOx,cs_pin,spi_work_mode; } FakeSPIConfig;
typedef struct { int work_mode,cali_mode; FakeSPIConfig spi_gyro_config,spi_acc_config; } BMI088_Init_Config_s;
extern int hspi2;
extern volatile uint32_t bmi088_init_error,bmi088_init_retry_count;
BMI088Instance *BMI088Register(const BMI088_Init_Config_s *c);
uint8_t BMI088Acquire(BMI088Instance *i,BMI088_Data_t *data);
uint8_t BMI088CalibService(void);
typedef enum { ROBOT_READY,ROBOT_ESTOP,ROBOT_FAULT } Robot_Status_e;
void RobotSafetySetImuValid(uint8_t valid);
void RobotSafetySetCalibValid(uint8_t valid);
Robot_Status_e RobotSafetyGetState(void);
uint8_t RobotSafetyIsReady(void);
#define VISION_BULLET_SPEED_DEFAULT 15.0f
void VisionUpdateTx(uint8_t mode,float w,float x,float y,float z,float yaw,float vyaw,float pitch,float vpitch,float speed,uint8_t flag);
#define TASK_MONITOR_INS 0
void TaskMonitorDelayMs(int task,uint32_t ms);
float NormOf3d(float *v);
void Norm3d(float *v);
float Dot3d(float *a,float *b);
void Cross3d(float *a,float *b,float *out);
#endif
