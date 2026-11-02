/** @file vision_transport_select.h
 * @brief 确定上位机通信使用的传输方式，不依赖车型或芯片定义。
 */
#ifndef MC02_VISION_TRANSPORT_SELECT_H
#define MC02_VISION_TRANSPORT_SELECT_H

#if defined(VISION_USE_VCP) && defined(VISION_USE_UART)
#error "Select only one vision transport"
#endif
#if !defined(VISION_USE_VCP) && !defined(VISION_USE_UART)
#define VISION_USE_VCP // 未显式选择时沿用板载 USB CDC 默认配置。
#endif
#endif
