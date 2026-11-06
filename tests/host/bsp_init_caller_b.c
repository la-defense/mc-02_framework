/** @file bsp_init_caller_b.c
 * @brief 第二个独立调用源文件；与第一个调用方共同链接同一份生产定义。
 */
#include "bsp_init.h"

/** @brief 在主机替身环境调用一次生产启动入口。 */
uint8_t BSPInitCallerB(void)
{
    return BSPInit(); // 第二个调用者也读取同一启动结果。
}
