/** @file lcd_diag_state_test.c
 * @brief 验证 LCD 诊断结束后按键和常规刷新能够继续工作。
 */
#include "lcd_task_diag_state.h"
#include "test_check.h"

int main(void)
{
    TEST_CHECK(LCDTaskDiagnosticOwnsUi(LCD_TASK_DIAG_DISABLED) == 0u);
    TEST_CHECK(LCDTaskDiagnosticOwnsUi(LCD_TASK_DIAG_IDLE) == 0u);
    TEST_CHECK(LCDTaskDiagnosticOwnsUi(LCD_TASK_DIAG_RUNNING) == 1u);
    TEST_CHECK(LCDTaskDiagnosticOwnsUi(LCD_TASK_DIAG_COMPLETE) == 0u);

    /* 诊断失败后要允许长按恢复，不能让黑屏状态一直挡住按键。 */
    TEST_CHECK(LCDTaskDiagnosticOwnsUi(LCD_TASK_DIAG_FAILED) == 0u);
    return 0;
}
