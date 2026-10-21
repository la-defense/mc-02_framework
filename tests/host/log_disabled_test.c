/** @file log_disabled_test.c
 * @brief 关闭事件提交时验证参数不求值，且目标无需链接日志处理端。
 * @note 生产事件目录仍参与编译检查；运行路径不会引用日志核心。
 */
#include "bsp_log.h"
#include "test_check.h"

#include <stdlib.h>

unsigned log_sample_never_linked(void);

int main(void)
{
    unsigned count = 0u;
    BSP_LOG_EVENT_0(EV_BSP_USB_BSP_USB_C_02);
    BSP_LOG_EVENT_1(EV_BSP_USB_BSP_USB_C_01, BSP_LOG_FIELD_U32(++count));
    BSP_LOG_EVENT_2(EV_MODULES_SAFETY_ROBOT_SAFETY_C_01,
                    BSP_LOG_FIELD_TEXT_LITERAL("SAFE"),
                    BSP_LOG_FIELD_U32(log_sample_never_linked()));
    TEST_CHECK(count == 0u);
    return EXIT_SUCCESS;
}
