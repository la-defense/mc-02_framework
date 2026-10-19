/** @file bsp_dwt.h
 * @brief 主机启动测试的计时替身声明；频率单位为 MHz，不访问寄存器。
 */
#ifndef BSP_INIT_TEST_DWT_H
#define BSP_INIT_TEST_DWT_H
#include <stdint.h>
void DWT_Init(uint32_t cpu_mhz);
#endif
