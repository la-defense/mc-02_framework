/** @file robot_can_motor_recovery.c
 * @brief 将车型核心机构的电机身份对应到实际 DJI 反馈。
 */
#include "robot_can_motor_recovery.h"
#include <stddef.h>
#include <string.h>

_Static_assert(DJI_MOTOR_CNT <= CAN_RECOVERY_MAX_FEEDBACK, "DJI recovery capacity exceeds feedback mask");

// 与 DJI 驱动相同，按型号和设备编号计算标准反馈 ID。
static uint32_t RecoveryRxId(Motor_Type_e type, uint32_t id)
{
    if (id == 0u || id > 8u) return 0u; // 非法设备编号不能计算反馈 ID。
    if (type == M2006 || type == M3508) return 0x200u + id; // 这两种电调从 0x201 开始。
    if (type == GM6020) return 0x204u + id; // GM6020 的反馈从 0x205 开始。
    return 0u; // 其他驱动尚未提供此恢复身份。
}

// 添加一台必需电机；同总线反馈 ID 冲突会使身份无法区分。
static uint8_t AddMotor(RobotCanMotorPlan_t *plan, const Motor_Init_Config_s *config,
                        MotorOutputRole_t role, uint32_t id)
{
    if (plan->count >= DJI_MOTOR_CNT || config->can_init_config.can_bus < PLATFORM_CAN_BUS_1 ||
        config->can_init_config.can_bus > PLATFORM_CAN_BUS_3 ||
        config->can_init_config.identifier_type != CAN_TX_ID_STANDARD) return 0u; // 不接受超量或无效总线。
    const uint32_t rx_id = RecoveryRxId(config->motor_type, id); // 按实际驱动使用的规则计算反馈 ID。
    if (rx_id == 0u) return 0u; // 未实现的型号不能当作已具备反馈。
    for (uint8_t i = 0u; i < plan->count; ++i)
        if (plan->motors[i].bus == config->can_init_config.can_bus &&
            RecoveryRxId(plan->motors[i].type, plan->motors[i].id) == rx_id) return 0u; // 不能在同一总线上复用反馈 ID。
    RobotCanMotorRequirement_t *item = &plan->motors[plan->count]; // 只在检查通过后追加。
    item->bus = config->can_init_config.can_bus; // 保留车型声明的逻辑总线。
    item->role = role; // 保留实际初始化时的机构用途。
    item->type = config->motor_type; // 后续与登记型号逐台比较。
    item->id = id; // 后续与登记设备编号比较。
    item->require_position = (uint8_t)(config->controller_setting_init_config.outer_loop_type == ANGLE_LOOP &&
        (config->controller_setting_init_config.close_loop_type & ANGLE_LOOP) != 0u &&
        config->controller_setting_init_config.angle_feedback_source == MOTOR_FEED); // 只有内部位置环需要电机多圈位置。
    ++plan->count; // 完整写入后才增加数量。
    return 1u;
}

uint8_t RobotCanMotorPlanBuild(const Robot_Config_s *config, const RobotModulePlan_t *modules,
                               RobotCanMotorPlan_t *output)
{
    if (config == NULL || modules == NULL || output == NULL ||
        RobotModulePlanValidate(modules, NULL) != ROBOT_MODULE_OK) return 0u; // 失败不改调用方结果。
    const uint32_t core = RobotModuleCoreClosure(modules); // 包含核心机构依赖的模块。
    RobotCanMotorPlan_t candidate = {0}; // 先在局部构建，不能交付部分计划。
    if ((core & ROBOT_MODULE_BIT(ROBOT_MODULE_CHASSIS)) != 0u)
    {
        if (config->chassis_param.wheel_type != OMNI_WHEEL &&
            config->chassis_param.wheel_type != MECANUM_WHEEL) return 0u; // 其他底盘尚无对应轮组登记规则。
        for (uint8_t i = 0u; i < 4u; ++i)
            if (!AddMotor(&candidate, &config->chassis_param.chassis_motor_config, MOTOR_OUTPUT_CHASSIS,
                          config->chassis_param.chassis_motor_id[i])) return 0u; // 四轮各自核对编号。
    }
    if ((core & ROBOT_MODULE_BIT(ROBOT_MODULE_GIMBAL)) != 0u)
    {
        if (config->gimbal_param.gimbal_type != SINGLE_GIMBAL &&
            config->gimbal_param.gimbal_type != MINI_GIMBAL) return 0u; // 不猜测其他轴系的登记关系。
        const Motor_Init_Config_s *yaw = &config->gimbal_param.yaw_motor_config; // 大 yaw 配置。
        if (!AddMotor(&candidate, yaw, MOTOR_OUTPUT_GIMBAL, yaw->can_init_config.tx_id)) return 0u; // 首先检查大 yaw。
        if (config->gimbal_param.gimbal_type == MINI_GIMBAL)
        {
            const Motor_Init_Config_s *mini = &config->gimbal_param.mini_yaw_motor_config; // 小 yaw 独立配置。
            if (!AddMotor(&candidate, mini, MOTOR_OUTPUT_GIMBAL, mini->can_init_config.tx_id)) return 0u; // 不能漏掉第三轴。
        }
        const Motor_Init_Config_s *pitch = &config->gimbal_param.pitch_motor_config; // pitch 配置。
        if (!AddMotor(&candidate, pitch, MOTOR_OUTPUT_GIMBAL, pitch->can_init_config.tx_id)) return 0u; // 最后检查 pitch。
    }
    if ((core & ROBOT_MODULE_BIT(ROBOT_MODULE_SHOOTER)) != 0u)
    {
        const Shoot_Config_s *shoot = &config->shoot_param; // 读取该车型发射机构。
        const uint8_t count = ShootFrictionWheelCount(shoot->shoot_type); // 按配置决定摩擦轮数量。
        if (count == 0u || count > 4u) return 0u; // 不接受未知结构。
        for (uint8_t i = 0u; i < count; ++i)
            if (!AddMotor(&candidate, &shoot->friction_motor_config, MOTOR_OUTPUT_FRICTION,
                          shoot->friction_motor_id[i])) return 0u; // 每台摩擦轮独立检查。
        if (!AddMotor(&candidate, &shoot->loader_motor_config, MOTOR_OUTPUT_LOADER,
                      shoot->loader_motor_config.can_init_config.tx_id)) return 0u; // 拨弹盘也单独检查。
    }
    if (candidate.count == 0u) return 0u; // 没有已知必需电机时不能声称电机反馈已验证。
    *output = candidate; // 全部设备均合法时才交付计划。
    return 1u;
}

uint8_t RobotCanMotorRecoveryMap(const RobotCanMotorPlan_t *plan, const DJI_RecoveryFeedback_s *feedback,
                                  uint32_t now_ms, CanRecoveryInput_t *input)
{
    if (plan == NULL || feedback == NULL || input == NULL || plan->count == 0u ||
        plan->count > DJI_MOTOR_CNT || feedback->count > DJI_MOTOR_CNT) return 0u; // 非法容量不能访问数组。
    uint32_t sequences[CAN_RECOVERY_MAX_FEEDBACK] = {0}; // 按车型固定顺序存储原始序号。
    uint32_t valid = 0u; // 开始时没有电机满足反馈条件。
    uint32_t used = 0u; // 不能把一台已登记电机匹配给多个需求。
    uint32_t core_roles = 0u; // 记录车型计划中的核心机构用途。
    for (uint8_t i = 0u; i < plan->count; ++i)
    {
        const RobotCanMotorRequirement_t *expected = &plan->motors[i]; // 当前车型要求。
        if (expected->role < MOTOR_OUTPUT_CHASSIS || expected->role > MOTOR_OUTPUT_LOADER)
            return 0u; // 非法用途不能作为位图下标。
        core_roles |= UINT32_C(1) << (unsigned)expected->role; // 后续检查该机构是否还有未声明的电机。
        uint8_t matches = 0u; // 设备身份必须恰好匹配一次。
        for (uint8_t j = 0u; j < feedback->count; ++j)
        {
            const DJI_RecoveryFeedbackItem_s *actual = &feedback->motors[j]; // 本轮检查一台已登记电机。
            if (actual->bus != expected->bus || actual->role != expected->role ||
                actual->motor_type != expected->type || actual->motor_id != expected->id) continue; // 不把其他电机当作当前设备。
            if ((used & (UINT32_C(1) << j)) != 0u) return 0u; // 同一设备不能满足两个需求。
            used |= UINT32_C(1) << j; // 记录已占用的登记项。
            ++matches; // 重复身份会在循环后拒绝。
            sequences[i] = actual->sequence; // 保留原始序号而非读取次数。
            if (actual->sequence != 0u && actual->speed_valid && actual->current_valid &&
                (!expected->require_position || actual->position_valid) &&
                (uint32_t)(now_ms - actual->source_time_ms) <= DJI_FEEDBACK_MAX_AGE_MS)
                valid |= UINT32_C(1) << i; // 到读取结束时仍有效才参与稳定判断。
        }
        if (matches != 1u) return 0u; // 缺少或重复的登记不能建立可信对应。
    }
    for (uint8_t j = 0u; j < feedback->count; ++j)
    {
        const unsigned role = (unsigned)feedback->motors[j].role; // 读取每个实际登记设备的用途。
        if (role < (unsigned)MOTOR_OUTPUT_CHASSIS || role > (unsigned)MOTOR_OUTPUT_LOADER)
            return 0u; // 未声明用途时不能证明它不属于核心机构。
        if ((core_roles & (UINT32_C(1) << role)) != 0u &&
            (used & (UINT32_C(1) << j)) == 0u) return 0u; // 同一核心机构存在未声明电机时不能声称核对完整。
    }
    memcpy(input->feedback_sequence, sequences, sizeof(sequences)); // 所有身份匹配后才交付序号。
    input->valid_feedback_mask = valid; // 保留每台设备独立有效位。
    return 1u;
}
