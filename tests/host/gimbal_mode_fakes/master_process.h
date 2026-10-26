/** @file master_process.h
 * @brief 提供 robot_def.h 定义控制结构所需的裁判枚举。
 * @note 不包含视觉传输实现，云台模式测试不读取或发送视觉数据。
 */
#ifndef MC02_GIMBAL_MODE_TEST_MASTER_PROCESS_H
#define MC02_GIMBAL_MODE_TEST_MASTER_PROCESS_H

typedef enum
{
    COLOR_NONE = 0,
    COLOR_BLUE,
    COLOR_RED
} Enemy_Color_e;

typedef enum
{
    BULLET_SPEED_NONE = 0,
    BIG_AMU_10 = 10,
    SMALL_AMU_15 = 15,
    BIG_AMU_16 = 16,
    SMALL_AMU_18 = 18,
    SMALL_AMU_30 = 30
} Bullet_Speed_e;

typedef enum
{
    VISION_YAW_UNCONFIGURED = 0
} GimbalModeTestVisionYawConstraint_e;

#endif
