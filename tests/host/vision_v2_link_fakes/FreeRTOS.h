/** @file FreeRTOS.h
 * @brief 单线程V2链接测试使用的临界区替身。
 * @note 并发锁行为由消息中心独立线程测试覆盖。
 */
#ifndef MC02_VISION_V2_LINK_FREERTOS_H
#define MC02_VISION_V2_LINK_FREERTOS_H

#define taskENTER_CRITICAL() ((void)0)
#define taskEXIT_CRITICAL() ((void)0)

#endif
