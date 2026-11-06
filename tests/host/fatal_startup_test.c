/** @file fatal_startup_test.c
 * @brief 验证生产致命错误入口在恢复额度内复位，耗尽后保持关断。
 */
#include "main.h"
#include "bsp_reset_recovery.h"
#include "test_check.h"
#include <setjmp.h>
#include <stdint.h>
#include <string.h>

FatalTestWatchdog_t fatal_test_watchdog;
uint32_t fatal_test_irq;
static uint8_t locked, outputs_off, fault_recorded, flags_cleared, reset_requested;
static jmp_buf exit_loop;

void BSP_SafetyLatchOutputsOff(void)
{ TEST_CHECK(fatal_test_irq == 1u); outputs_off = 1u; } // 故障路径先屏蔽中断并关断输出。
void BSP_ResetRecoveryMarkFault(ResetRecoveryCause_e cause)
{ TEST_CHECK(outputs_off && cause == RESET_RECOVERY_CAUSE_FATAL_HANDLER); fault_recorded = 1u; }
uint8_t BSP_ResetRecoveryIsLockedOut(void)
{ TEST_CHECK(fault_recorded); return locked; } // 查询额度之前必须保存故障。
void FatalTestClearResetFlags(void)
{ TEST_CHECK(fault_recorded && !locked); flags_cleared = 1u; } // 只在准备复位时清旧标志。
void NVIC_SystemReset(void)
{ TEST_CHECK(flags_cleared); reset_requested = 1u; longjmp(exit_loop, 1); }
void FatalTestNop(void) { longjmp(exit_loop, 1); } // 让主机测试退出实际的失能循环。

int main(int argc, char **argv)
{
    TEST_CHECK(argc == 2); // 每个场景在独立进程运行。
    locked = (uint8_t)(strcmp(argv[1], "locked") == 0); // 选择额度耗尽或允许恢复。
    TEST_CHECK(locked || strcmp(argv[1], "reset") == 0); // 拒绝未知测试名。
    if (setjmp(exit_loop) == 0) Error_Handler(); // 使用生产致命错误入口。
    TEST_CHECK(outputs_off && fault_recorded && fatal_test_irq); // 离开前必须保持关断。
    TEST_CHECK(reset_requested == !locked && flags_cleared == !locked); // 只在有额度时复位。
    TEST_CHECK(!locked || fatal_test_watchdog.KR == 0xAAAAu); // 锁定时重载可能仍运行的看门狗。
    return 0;
}
