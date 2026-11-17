/** @file robot_can_motor_recovery.h
 * @brief 按车型核心依赖核对 DJI 电机，并交付逐台恢复反馈。
 */
#ifndef MC02_ROBOT_CAN_MOTOR_RECOVERY_H
#define MC02_ROBOT_CAN_MOTOR_RECOVERY_H

#include "config/config.h"
#include "module_plan.h"
#include "dji_motor.h"
#include "can_recovery.h"

/** @brief 一台核心电机在车型配置中的身份与反馈要求。 */
typedef struct {
    PlatformCanBus_t bus; /**< 配置中的逻辑总线。 */
    MotorOutputRole_t role; /**< 配置中的机构用途。 */
    Motor_Type_e type; /**< 配置中的 DJI 电机型号。 */
    uint32_t id; /**< 配置中的设备编号，范围 1～8。 */
    uint8_t require_position; /**< 使用内部多圈位置闭环时为 1。 */
} RobotCanMotorRequirement_t;

/** @brief 启动阶段构造的固定顺序电机需求。 */
typedef struct {
    uint8_t count; /**< 必需电机数量，范围 1～DJI_MOTOR_CNT。 */
    RobotCanMotorRequirement_t motors[DJI_MOTOR_CNT]; /**< 恢复计算的位号与此顺序相同。 */
} RobotCanMotorPlan_t;

/** @brief 从车型配置和核心依赖构造 DJI 电机需求。
 * @param[in] config 非空，只读车型配置。
 * @param[in] modules 已验证的模块计划。
 * @param[out] output 成功时写入完整计划；失败保持旧值。
 * @return 1 为成功；无电机、型号或配置不支持、设备冲突及输入错误返回 0。
 * @note 不登记设备或改变实际车型；双云台构型尚无已确认的登记规则时拒绝。
 */
uint8_t RobotCanMotorPlanBuild(const Robot_Config_s *config, const RobotModulePlan_t *modules,
                               RobotCanMotorPlan_t *output);

/** @brief 将已登记电机按车型身份映射到恢复计算输入。
 * @param[in] plan 启动时建立的电机计划。
 * @param[in] feedback 实际登记电机的反馈及身份。
 * @param[in] now_ms 读取结束时的本机时间，单位 ms。
 * @param[in,out] input 只更新反馈有效位和序号；身份不匹配时保持旧值。
 * @return 1 为身份完整且完成映射；0 为缺失、重复、超容量或参数无效。
 * @note 每台必须分别提供新鲜速度和电流，使用内部位置闭环的电机还须有可信位置。
 * 不验证总线、运行许可或命令，也不解除任何故障。
 */
uint8_t RobotCanMotorRecoveryMap(const RobotCanMotorPlan_t *plan, const DJI_RecoveryFeedback_s *feedback,
                                  uint32_t now_ms, CanRecoveryInput_t *input);

/** @brief 同时取得对应的实际登记下标位图，供最终原子复核使用。
 * @param[in] plan 车型必需电机。
 * @param[in] feedback 当前登记数据。
 * @param[in] now_ms 读取结束时的本机毫秒。
 * @param[in,out] input 成功时更新恢复反馈；失败不改变。
 * @param[out] registered_mask 成功时写入登记下标位图；不能为空，失败不改变。
 * @return 1 为全部身份匹配，0 为参数、容量或身份错误。
 */
uint8_t RobotCanMotorRecoveryMapWithMask(const RobotCanMotorPlan_t *plan,
    const DJI_RecoveryFeedback_s *feedback, uint32_t now_ms, CanRecoveryInput_t *input,
    uint32_t *registered_mask);
#endif
