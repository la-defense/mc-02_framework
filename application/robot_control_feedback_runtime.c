/** @file robot_control_feedback_runtime.c
 * @brief 读取整车实际反馈，将分功能期限交给统一安全管理。
 */
#include "robot_control_feedback_runtime.h"
#include "robot_safety.h"
#include "platform_irq.h"
#include "platform_sync.h"
#include <stddef.h>

static RobotControlFeedback_t collector; // 启动时固定，只由命令任务更新间断记录。

uint8_t RobotControlFeedbackStart(const RobotCanMotorPlan_t *plan)
{
    if (collector.initialized) return 0u; // 重复绑定不改变已运行的采集者。
    if (!RobotControlFeedbackInit(&collector, plan)) return 0u; // 缺少完整的电机身份时不能绑定。
    if (RobotSafetyBindVehicleFeedback()) return 1u; // 成功后命令不能再提供反馈事实。
    collector.initialized = 0u; // 绑定被拒绝，周期入口不能使用此次初始化结果。
    return 0u;
}

uint8_t RobotControlFeedbackPoll(const MessageFreshness_t *pose)
{
    if (!collector.initialized) return 0u; // 只允许启动已绑定的采集者执行。
    DJI_RecoveryFeedback_s samples = {0}; // 每个登记项携带自己的原始时间与有效标志。
    RobotControlFeedbackResult_t result = {0}; // 汇总失败时以全部无效提交。
    const uint32_t now_ms = PlatformTimeNowMs(); // 使用电机和消息通用的本机毫秒时钟。
    if (DJIMotorGetRecoveryFeedback(now_ms, &samples))
        (void)RobotControlFeedbackCollect(&collector, &samples, pose,
            PlatformTimeNowMs(), &result); // 汇总时重新取时，避免把晚到的样本判成将来数据。

    const PlatformIrqState_t previous = PlatformIrqSaveAndDisable(); // 最终检查和提交之间不让出执行权。
    const uint32_t publish_ms = PlatformTimeNowMs(); // 汇总结束后反馈仍必须在原期限内。
    for (unsigned group = 0u; group < 3u; ++group)
    {
        const uint32_t bit = UINT32_C(1) << group; // 分别复查三个已知功能。
        if (!(result.valid_functions & bit)) continue; // 已判无效的组无需重复查询电机。
        const RobotControlLease_t *lease = &result.feedback[group]; // 保留原始输入的截止时间。
        if ((uint32_t)(publish_ms - lease->source_time_ms) > lease->max_age_ms ||
            !DJIMotorRecoveryStillValid(&samples, result.registered_motor_mask[group], publish_ms))
            result.valid_functions &= ~bit; // 复查失败只撤销受影响的功能。
    }
    const uint8_t accepted = RobotSafetyPublishVehicleFeedback(result.valid_functions, result.feedback); // 只更新反馈，不改写来源选择。
    PlatformIrqRestore(previous); // 恢复调用前的中断屏蔽。
    return accepted;
}
