/** @file motor_output_role.h
 * @brief 声明电机所属的机构，供执行前分功能许可检查使用。
 */
#ifndef MC02_MOTOR_OUTPUT_ROLE_H
#define MC02_MOTOR_OUTPUT_ROLE_H
typedef enum {
    MOTOR_OUTPUT_UNASSIGNED = 0, /**< 未声明；新许可策略启用时禁止输出。 */
    MOTOR_OUTPUT_CHASSIS, /**< 底盘轮组。 */
    MOTOR_OUTPUT_GIMBAL, /**< 云台轴。 */
    MOTOR_OUTPUT_FRICTION, /**< 发射机构摩擦轮。 */
    MOTOR_OUTPUT_LOADER /**< 拨弹机构；击发级许可仍待接入。 */
} MotorOutputRole_t;
#endif
