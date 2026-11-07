/** @file robot_param_guard.c
 * @brief 保留 Flash 维护期限，按温控模块实际启动状态申请加热保护。
 */
#include "robot_param_guard.h"
#include "config/build_selection.h"
#include "task_monitor.h"
#include "platform_irq.h"
#if MC02_FEATURE_HEATER
#include "imu_heater.h"
#if defined(MC02_CONFIG_GENERATED)
#include "config/vehicle_selection.h"
#endif
static uint8_t heater_held; // 只由本次已接受的维护操作持有。
#endif
static uint8_t operation_active; // 0 空闲、1 准备、2 已开始、3 收尾；中断保护状态变化。

uint8_t RobotParamFlashOperationBegin(void)
{
    const PlatformIrqState_t saved = PlatformIrqSaveAndDisable(); // 保存调用前的中断状态。
    if (operation_active)
    {
        PlatformIrqRestore(saved); // 重叠操作不能覆盖已经取得的保护。
        return 0u;
    }
    operation_active = 1u; // 在申请其他资源前占用本次维护位置。
    PlatformIrqRestore(saved); // 申请保护不能在长时间关中断下执行。
#if MC02_FEATURE_HEATER
#if defined(MC02_CONFIG_GENERATED)
    const uint8_t heater_available = RobotBuildModuleStarted(ROBOT_MODULE_HEATER); // 降级模块不接收新请求。
#else
    const uint8_t heater_available = 1u; // 历史独立组合沿用温控保护。
#endif
    if (heater_available)
    {
        if (!IMUHeaterBeginLongOperation(IMU_HEATER_LONG_OPERATION_FLASH)) goto failed; // 可用温控拒绝时不能写入。
        heater_held = 1u; // 无论后续模块状态如何，本次保护都必须结束。
    }
#endif
    if (!TaskMonitorBeginLongOperation(TASK_MONITOR_INS, TASK_MONITOR_LONG_OPERATION_FLASH))
    {
#if MC02_FEATURE_HEATER
        if (heater_held)
        {
            IMUHeaterEndLongOperation(IMU_HEATER_LONG_OPERATION_FLASH, 0u); // 监控拒绝时撤销已取得的温控保护。
            heater_held = 0u;
        }
#endif
        goto failed; // 不允许没有任务监控期限的 Flash 维护。
    }
    {
        const PlatformIrqState_t previous = PlatformIrqSaveAndDisable(); // 所有保护成功后才能接受结束请求。
        operation_active = 2u; // 发布已开始状态。
        PlatformIrqRestore(previous);
    }
    return 1u; // 参数层还需单独通过 Flash 授权与看门狗检查。
failed:
    {
        const PlatformIrqState_t previous = PlatformIrqSaveAndDisable(); // 防止释放时与新请求竞争。
        operation_active = 0u; // 此次准备失败，可以接受下一次请求。
        PlatformIrqRestore(previous);
    }
    return 0u;
}

uint8_t RobotParamFlashOperationEnd(uint8_t success)
{
    const PlatformIrqState_t saved = PlatformIrqSaveAndDisable(); // 保持配对检查与状态变化一致。
    if (operation_active != 2u)
    {
        PlatformIrqRestore(saved); // 没有开始或已有结束者时不得重复收尾。
        return 0u;
    }
    operation_active = 3u; // 收尾完成前阻止新的维护请求。
    PlatformIrqRestore(saved);
    const uint8_t within_window = TaskMonitorEndLongOperation(TASK_MONITOR_INS,
        TASK_MONITOR_LONG_OPERATION_FLASH); // 结束原始固定期限，不续期。
    const uint8_t result = (uint8_t)(success && within_window); // 两项都成功才允许报告成功。
#if MC02_FEATURE_HEATER
    if (heater_held)
    {
        IMUHeaterEndLongOperation(IMU_HEATER_LONG_OPERATION_FLASH, result); // 已取得的保护即使模块降级也需结束。
        heater_held = 0u;
    }
#endif
    const PlatformIrqState_t previous = PlatformIrqSaveAndDisable(); // 全部收尾完成后才允许下一次请求。
    operation_active = 0u;
    PlatformIrqRestore(previous);
    return result;
}
