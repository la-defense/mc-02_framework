/** @file ins_task.h
 * @brief 为生产云台模式测试提供姿态数据和只读 IMU 输入接口。
 * @note 测试填入固定数据，不启动 IMU 采样任务。
 */
#ifndef MC02_GIMBAL_MODE_TEST_INS_TASK_H
#define MC02_GIMBAL_MODE_TEST_INS_TASK_H

#include <stdint.h>

enum
{
    X = 0,
    Y = 1,
    Z = 2
};

typedef struct
{
    float Gyro[3];
    float Accel[3];
    float Roll;
    float Pitch;
    float Yaw;
    float YawTotalAngle;
} attitude_t;

typedef struct
{
    float yaw_rad;
    float yaw_total_rad;
    float pitch_rad;
    float gyro_rad_s[3];
    uint32_t timestamp_ms;
    uint32_t sequence;
    uint8_t valid;
} INS_Control_Snapshot_s;

attitude_t *INS_GetStorage(void);
uint8_t INSGetControlSnapshot(INS_Control_Snapshot_s *snapshot);

#endif
