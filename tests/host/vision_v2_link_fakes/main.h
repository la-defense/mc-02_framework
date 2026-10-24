/** @file main.h
 * @brief V2生产接收链路主机测试所需的本机毫秒时钟声明。
 * @note 不包含板级外设；测试通过USB回调模型注入接收块。
 */
#ifndef MC02_VISION_V2_LINK_MAIN_H
#define MC02_VISION_V2_LINK_MAIN_H

#include <stdint.h>

uint32_t HAL_GetTick(void);

#endif
