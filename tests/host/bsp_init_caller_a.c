/** @file bsp_init_caller_a.c
 * @brief 第一个独立调用源文件；重复包含公开头也应正常编译。
 */
#include "bsp_init.h"
#include "bsp_init.h"

/** @brief 在主机替身环境调用一次生产启动入口。 */
void BSPInitCallerA(void)
{
    BSPInit();
}
