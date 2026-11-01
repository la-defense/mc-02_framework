/** @file platform_resources.h
 * @brief 车型使用的逻辑资源编号；不暴露芯片或外设句柄。
 */
#ifndef MC02_PLATFORM_RESOURCES_H
#define MC02_PLATFORM_RESOURCES_H

/** @brief 车型使用的经典 CAN 总线编号，由板级适配决定实际控制器。 */
typedef enum {
    PLATFORM_CAN_INVALID = 0, /**< 未指定总线，不可注册设备。 */
    PLATFORM_CAN_BUS_1,       /**< 板级声明的第一条 CAN 总线。 */
    PLATFORM_CAN_BUS_2,       /**< 板级声明的第二条 CAN 总线。 */
    PLATFORM_CAN_BUS_3        /**< 板级声明的第三条 CAN 总线。 */
} PlatformCanBus_t;
#endif
