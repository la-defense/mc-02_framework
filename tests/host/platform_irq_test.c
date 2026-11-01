/** @file platform_irq_test.c
 * @brief 验证中断状态恢复和车型故障标记使用同一生产适配器。
 */
#include "platform_irq.h"
#include "vehicle_selection.h"
#include "test_check.h"
#include <stddef.h>
#include <stdint.h>

static uint32_t irq_mask; // 模拟 PRIMASK 中的当前屏蔽位。
static unsigned disable_count; // 检查无效模块不会进入保护区。
uint32_t TestIrqGetMask(void) { return irq_mask; }
void TestIrqDisable(void) { irq_mask = 1u; ++disable_count; }
void TestIrqSetMask(uint32_t mask) { irq_mask = mask; }

static uint8_t StartModule(void *context, unsigned module)
{
    (void)context; // 测试不需要启动上下文。
    (void)module; // 启用模块均模拟启动成功。
    return 1u;
}

int main(void)
{
    const PlatformIrqState_t outer = PlatformIrqSaveAndDisable(); // 保存初始未屏蔽状态。
    TEST_CHECK(outer == 0u && irq_mask == 1u);
    const PlatformIrqState_t inner = PlatformIrqSaveAndDisable(); // 嵌套时保存已屏蔽状态。
    TEST_CHECK(inner == 1u && irq_mask == 1u);
    PlatformIrqRestore(inner); // 恢复内层前的状态后仍保持屏蔽。
    TEST_CHECK(irq_mask == 1u);
    PlatformIrqRestore(outer); // 恢复外层前的状态后才重新允许中断。
    TEST_CHECK(irq_mask == 0u);
    TEST_CHECK(RobotBuildStart(StartModule, NULL) == ROBOT_MODULE_OK); // 建立车型启动结果。
    const unsigned before = disable_count; // 记录有效故障处理前的进入次数。
    TEST_CHECK(RobotBuildMarkFailure(ROBOT_MODULE_COUNT) == 0u && disable_count == before);
    irq_mask = 1u; // 模拟调用前已经屏蔽中断，验证辅助故障处理不擅自开启。
    TEST_CHECK(RobotBuildMarkFailure(ROBOT_MODULE_LCD) == 0u && irq_mask == 1u);
    TEST_CHECK((robot_module_startup.failed & ROBOT_MODULE_BIT(ROBOT_MODULE_LCD)) != 0u);
    irq_mask = 0u; // 模拟正常任务状态，核心故障必须按原状态恢复。
    TEST_CHECK(RobotBuildMarkFailure(ROBOT_MODULE_SAFETY) == 1u && irq_mask == 0u);
    TEST_CHECK(robot_module_startup.core_failed == 1u); // 核心失败继续向上报告。
    return 0;
}
