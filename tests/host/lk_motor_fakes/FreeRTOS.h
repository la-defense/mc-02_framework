/** @file FreeRTOS.h
 * @brief LK 主机测试中的临界区替身。
 * @note 确定性交错测试为单线程；线程同步测试使用生产临界区模型。
 * @author MC-02 project contributors
 * @copyright Copyright (c) HNU YueLu EC 2022 all rights reserved
 */
#ifndef MC02_LK_TEST_FREERTOS_H
#define MC02_LK_TEST_FREERTOS_H
#define taskENTER_CRITICAL() ((void)0)
#define taskEXIT_CRITICAL() ((void)0)
#endif
