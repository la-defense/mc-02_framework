/** @file log_disabled_test.c
 * @brief 关闭日志后检查参数不求值；不提供RTT和取样函数，若留下调用引用会直接链接失败。
 * @note 通过生产日志头编译；所有检查在Release仍执行，不访问硬件。
 */
#include "bsp_log.h"
#include "test_check.h"
#include <stdlib.h>

// 故意只声明而不定义：关闭宏不能保留这个实参调用。
unsigned log_sample_never_linked(void);

/** @brief 三类关闭宏均不执行参数，自增与函数调用都不发生。 */
int main(void)
{
    unsigned count = 0u;
    LOGINFO("%u", ++count);
    LOGWARNING("%u", ++count);
    LOGERROR("%u", log_sample_never_linked());
    TEST_CHECK(count == 0u);
    return EXIT_SUCCESS;
}
