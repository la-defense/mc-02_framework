/** @file bsp_param.h
 * @brief 主机启动测试的参数替身声明；模拟载入失败，不读写 Flash。
 */
#ifndef BSP_INIT_TEST_PARAM_H
#define BSP_INIT_TEST_PARAM_H
#include <stdint.h>
uint8_t ParamInit(void);
#endif
