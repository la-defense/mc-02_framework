/**
 * @file lcd_ui_test_types.h
 * @brief 提供编译生产 LCD UI 所需的窄型主机依赖声明。
 * @note 静态绘制测试不调用这些业务服务；固件仍使用各模块的正式头文件。
 */
#ifndef MC02_LCD_UI_TEST_TYPES_H
#define MC02_LCD_UI_TEST_TYPES_H

#include <stddef.h>
#include <stdint.h>
#include "platform_can.h"
#include "bmi088_calibration.h"

typedef enum { ROBOT_INIT, ROBOT_SAFE, ROBOT_CALIB, ROBOT_READY, ROBOT_FAULT, ROBOT_ESTOP } Robot_Status_e;
#define ROBOT_FAULT_RC_OFFLINE (1u << 0)
#define ROBOT_FAULT_IMU_INVALID (1u << 1)
#define ROBOT_FAULT_MOTOR_OFFLINE (1u << 2)
#define ROBOT_FAULT_CAN_BUSOFF (1u << 3)
#define ROBOT_FAULT_TILT (1u << 4)
#define ROBOT_FAULT_CALIB_INVALID (1u << 5)
#define ROBOT_FAULT_TASK_TIMEOUT (1u << 6)
typedef enum { ESTOP_REASON_RC_OFFLINE, ESTOP_REASON_MANUAL } EstopReason_e;
Robot_Status_e RobotSafetyGetState(void);
uint32_t RobotSafetyGetFaults(void);
uint8_t EstopIsLatched(void);
EstopReason_e EstopGetReason(void);

typedef enum { BSP_ADC_STATUS_VALID, BSP_ADC_STATUS_INVALID } BSP_ADC_Status_e;
typedef struct { uint16_t raw_key; float vcc_in_volts; uint8_t voltage_config_valid; } BSP_ADC_Sample_t;
BSP_ADC_Status_e BSP_ADCGetSample(BSP_ADC_Sample_t *output);

uint8_t BSP_WatchdogIsRunning(void);
uint8_t BSP_WatchdogWasIwdgReset(void);

typedef enum { TASK_MONITOR_SYSTEM, TASK_MONITOR_INS, TASK_MONITOR_REMOTE, TASK_MONITOR_MOTOR,
               TASK_MONITOR_UART_TX, TASK_MONITOR_COUNT } TaskMonitor_Id_e;
typedef struct { uint8_t alive; } TaskMonitorStatus_t;
void TaskMonitorGetStatus(TaskMonitor_Id_e id, TaskMonitorStatus_t *output);

uint8_t RemoteControlIsOnline(void);
typedef struct { uint8_t online; uint8_t mode; uint16_t bullet_count; uint32_t last_rx_ms; } Vision_Status_t;
void VisionGetStatus(Vision_Status_t *output);

typedef struct
{
    float temperature, target_temp;
    uint16_t duty;
    uint8_t fault, overtemp, timeout_fault, sensor_valid, heating;
} IMUHeaterStatus_t;
void IMUHeaterGetStatus(IMUHeaterStatus_t *output);
typedef struct { float Yaw, Pitch, Roll; } attitude_t;
attitude_t *INS_GetAttitude(void);

typedef enum { CRASH_TYPE_ASSERT, CRASH_TYPE_STACK_OVF } CrashType_e;
typedef struct { CrashType_e type; uint32_t pc, count; char task_name[12]; } CrashLog_t;
uint8_t CrashLogGetLast(CrashLog_t *output);
const char *CrashLogTypeShort(CrashType_e type);

#endif /* MC02_LCD_UI_TEST_TYPES_H */
