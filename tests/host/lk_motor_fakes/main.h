/** @file main.h
 * @brief 为 LK 主机测试提供 HAL 毫秒时钟声明。
 * @note 仅在生产模块的主机替身目录中使用，不参与固件构建。
 * @author MC-02 project contributors
 * @copyright Copyright (c) HNU YueLu EC 2022 all rights reserved
 */
#ifndef MC02_LK_TEST_MAIN_H
#define MC02_LK_TEST_MAIN_H

#include <stdint.h>

uint32_t HAL_GetTick(void);

#endif
