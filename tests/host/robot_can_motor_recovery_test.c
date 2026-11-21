/** @file robot_can_motor_recovery_test.c
 * @brief 验证车型核心电机身份与恢复反馈的逐台对应，不连接硬件。
 */
#include "robot_can_motor_recovery.h"
#include "test_check.h"
#include <string.h>

static Motor_Init_Config_s motor(PlatformCanBus_t bus, Motor_Type_e type, uint32_t id)
{
    Motor_Init_Config_s result = {0}; // 使用独立的测试配置。
    result.can_init_config.can_bus = bus; // 设备所属测试总线。
    result.can_init_config.tx_id = id; // 设备编号按测试输入给出。
    result.motor_type = type; // 型号必须能被 DJI 驱动识别。
    return result;
}

int main(void)
{
    Robot_Config_s config = {0}; // 测试参数不进入生产车型。
    config.chassis_param.wheel_type = OMNI_WHEEL; // 使用四轮全向底盘。
    config.chassis_param.chassis_motor_config = motor(PLATFORM_CAN_BUS_1, M3508, 0u);
    config.chassis_param.wheel_motors[MOTOR_LF] = (ChassisWheelMotorConfig_t){3u, MOTOR_AXIS_DIRECTION_NORMAL}; // 左前自定义编号 3。
    config.chassis_param.wheel_motors[MOTOR_RF] = (ChassisWheelMotorConfig_t){1u, MOTOR_AXIS_DIRECTION_REVERSE}; // 右前自定义编号 1。
    config.chassis_param.wheel_motors[MOTOR_LB] = (ChassisWheelMotorConfig_t){4u, MOTOR_AXIS_DIRECTION_NORMAL}; // 左后自定义编号 4。
    config.chassis_param.wheel_motors[MOTOR_RB] = (ChassisWheelMotorConfig_t){2u, MOTOR_AXIS_DIRECTION_REVERSE}; // 右后自定义编号 2。
    config.gimbal_param.gimbal_type = MINI_GIMBAL; // 测试大 yaw、小 yaw 和 pitch 均不可省略。
    config.gimbal_param.yaw_motor_config = motor(PLATFORM_CAN_BUS_1, GM6020, 5u);
    config.gimbal_param.mini_yaw_motor_config = motor(PLATFORM_CAN_BUS_2, GM6020, 1u);
    config.gimbal_param.pitch_motor_config = motor(PLATFORM_CAN_BUS_2, GM6020, 2u);
    config.gimbal_param.pitch_motor_config.controller_setting_init_config.outer_loop_type = ANGLE_LOOP;
    config.gimbal_param.pitch_motor_config.controller_setting_init_config.close_loop_type = ANGLE_LOOP;
    config.gimbal_param.pitch_motor_config.controller_setting_init_config.angle_feedback_source = MOTOR_FEED;
    RobotModulePlan_t modules = {0}; // 两个机构均属于核心。
    modules.required = modules.enabled = ROBOT_MODULE_BIT(ROBOT_MODULE_CHASSIS) |
                                         ROBOT_MODULE_BIT(ROBOT_MODULE_GIMBAL);
    RobotCanMotorPlan_t plan = {0}; // 保存按车型声明建立的设备顺序。
    TEST_CHECK(RobotCanMotorPlanBuild(&config, &modules, &plan) && plan.count == 7u);
    TEST_CHECK(plan.motors[0].id == 3u && plan.motors[1].id == 1u &&
        plan.motors[2].id == 4u && plan.motors[3].id == 2u); // 恢复计划跟随轮序，编号没有被重新排列。
    TEST_CHECK(plan.motors[5].id == 1u && plan.motors[6].require_position); // 小 yaw 和内部位置需求都存在。
    RobotCanMotorPlan_t saved = plan; // 错误配置不能改写旧结果。
    config.chassis_param.wheel_motors[MOTOR_LF].device_id = 5u; // 左前轮的反馈 ID 会与 GM6020 的编号 1 重合。
    config.gimbal_param.mini_yaw_motor_config.can_init_config.can_bus = PLATFORM_CAN_BUS_1; // 同总线上的两台电机不能共享反馈 ID。
    TEST_CHECK(!RobotCanMotorPlanBuild(&config, &modules, &plan));
    TEST_CHECK(memcmp(&plan, &saved, sizeof(plan)) == 0); // 拒绝跨机构冲突时保留完整旧清单。
    config.gimbal_param.mini_yaw_motor_config.can_init_config.can_bus = PLATFORM_CAN_BUS_2; // 分到不同总线后允许使用相同反馈 ID。
    TEST_CHECK(RobotCanMotorPlanBuild(&config, &modules, &plan) && plan.motors[0].id == 5u);
    config.chassis_param.wheel_motors[MOTOR_LF].device_id = 3u; // 恢复本用例余下检查使用的轮组编号。
    plan = saved; // 后续仍以通过的原计划检查错误保留行为。
    config.gimbal_param.mini_yaw_motor_config.can_init_config.tx_id = 2u; // 小 yaw 与 pitch 冲突。
    TEST_CHECK(!RobotCanMotorPlanBuild(&config, &modules, &plan));
    TEST_CHECK(memcmp(&plan, &saved, sizeof(plan)) == 0);
    config.gimbal_param.mini_yaw_motor_config.can_init_config.tx_id = 1u;
    config.gimbal_param.mini_yaw_motor_config.motor_type = MOTOR_TYPE_NONE; // 未支持型号不能假定反馈可用。
    TEST_CHECK(!RobotCanMotorPlanBuild(&config, &modules, &plan));
    config.gimbal_param.mini_yaw_motor_config.motor_type = GM6020;
    RobotModulePlan_t shooter_modules = {0}; // 另一车型可以只把发射机构列为核心。
    shooter_modules.enabled = shooter_modules.required = ROBOT_MODULE_BIT(ROBOT_MODULE_SHOOTER);
    config.shoot_param.shoot_type = THREE_SHOOT; // 三个摩擦轮都必须独立登记。
    config.shoot_param.friction_motor_config = motor(PLATFORM_CAN_BUS_3, M3508, 0u);
    config.shoot_param.friction_motor_id[0] = 1u;
    config.shoot_param.friction_motor_id[1] = 2u;
    config.shoot_param.friction_motor_id[2] = 3u;
    config.shoot_param.loader_motor_config = motor(PLATFORM_CAN_BUS_3, M2006, 4u);
    RobotCanMotorPlan_t shooter_plan = {0};
    TEST_CHECK(RobotCanMotorPlanBuild(&config, &shooter_modules, &shooter_plan) &&
               shooter_plan.count == 4u && shooter_plan.motors[3].role == MOTOR_OUTPUT_LOADER);
    shooter_modules.required = ROBOT_MODULE_BIT(ROBOT_MODULE_CHASSIS); // 未启用的核心模块不能构成有效计划。
    TEST_CHECK(!RobotCanMotorPlanBuild(&config, &shooter_modules, &shooter_plan));

    DJI_RecoveryFeedback_s feedback = {0}; // 模拟登记顺序与车型顺序相反。
    feedback.count = plan.count;
    for (uint8_t i = 0u; i < plan.count; ++i)
    {
        const RobotCanMotorRequirement_t *expected = &plan.motors[i];
        DJI_RecoveryFeedbackItem_s *item = &feedback.motors[plan.count - 1u - i];
        item->bus = expected->bus; item->role = expected->role; // 登记总线和用途。
        item->motor_type = expected->type; item->motor_id = expected->id; // 登记型号和编号。
        item->sequence = (uint32_t)i + 10u; item->source_time_ms = 100u; // 各自的原始样本。
        item->speed_valid = item->current_valid = item->position_valid = 1u;
    }
    CanRecoveryInput_t input = {.now_ms = 100u}; // 保留其他恢复条件供调用方填写。
    TEST_CHECK(RobotCanMotorRecoveryMap(&plan, &feedback, 100u, &input));
    TEST_CHECK(input.valid_feedback_mask == 0x7fu && input.feedback_sequence[0] == 10u &&
               input.feedback_sequence[6] == 16u); // 交付顺序由车型而非注册顺序决定。
    feedback.motors[7] = feedback.motors[1]; // 同一核心机构多出未声明的电机。
    feedback.motors[7].motor_id = 7u;
    feedback.count = 8u;
    TEST_CHECK(!RobotCanMotorRecoveryMap(&plan, &feedback, 100u, &input));
    feedback.motors[7].role = MOTOR_OUTPUT_UNASSIGNED; // 未声明用途时不能判断其影响范围。
    TEST_CHECK(!RobotCanMotorRecoveryMap(&plan, &feedback, 100u, &input));
    feedback.count = plan.count; // 后续恢复测试继续使用完整原清单。
    for (uint8_t i = 0u; i < feedback.count; ++i)
        feedback.motors[i].source_time_ms = UINT32_MAX - 3u; // 模拟毫秒计数回绕前采样。
    TEST_CHECK(RobotCanMotorRecoveryMap(&plan, &feedback, 1u, &input) &&
               input.valid_feedback_mask == 0x7fu); // 回绕后 5 ms 的样本仍然有效。
    for (uint8_t i = 0u; i < feedback.count; ++i)
        feedback.motors[i].source_time_ms = 100u; // 后续恢复普通测试时间。
    feedback.motors[0].position_valid = 0u; // pitch 使用内部多圈位置，失效不能恢复。
    TEST_CHECK(RobotCanMotorRecoveryMap(&plan, &feedback, 100u, &input));
    TEST_CHECK((input.valid_feedback_mask & (1u << 6u)) == 0u);
    feedback.motors[0].position_valid = 1u;
    TEST_CHECK(RobotCanMotorRecoveryMap(&plan, &feedback, 121u, &input) &&
               input.valid_feedback_mask == 0u); // 过期样本不能被重复查询续期。
    CanRecoveryInput_t previous = input;
    feedback.motors[0].motor_id = 7u; // 错误设备编号不能替代必需 pitch。
    TEST_CHECK(!RobotCanMotorRecoveryMap(&plan, &feedback, 100u, &input));
    TEST_CHECK(memcmp(&input, &previous, sizeof(input)) == 0); // 映射失败保持旧输出。
    feedback.motors[0] = feedback.motors[1]; // 重复身份也不能满足缺失轴。
    TEST_CHECK(!RobotCanMotorRecoveryMap(&plan, &feedback, 100u, &input));
    return 0;
}
