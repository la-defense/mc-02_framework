/** @file main.h
 * @brief 为 Daemon 来源测试提供 HAL 毫秒时钟声明。
 * @note 时钟由测试按场景推进；不参与固件构建。
 * @author MC-02 project contributors
 * @copyright Copyright (c) HNU YueLu EC 2022 all rights reserved
 */
#ifndef MC02_DAEMON_TEST_MAIN_H
#define MC02_DAEMON_TEST_MAIN_H
#include <stdint.h>
uint32_t HAL_GetTick(void);
#endif
