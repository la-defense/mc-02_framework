/** @file platform_power.h
 * @brief 板级受控电源的逻辑编号和启动操作。
 */
#ifndef MC02_PLATFORM_POWER_H
#define MC02_PLATFORM_POWER_H
#include <stdint.h>

typedef enum {
    PLATFORM_POWER_24V_1 = 0, /**< 第一条 24V 输出。 */
    PLATFORM_POWER_24V_2, /**< 第二条 24V 输出。 */
    PLATFORM_POWER_5V, /**< 5V 输出。 */
    PLATFORM_POWER_COUNT
} PlatformPowerRail_t;

/** @brief 发出一路电源的开启命令。
 * @param[in] rail 板级声明的电源编号。
 * @return 命令已写入返回 1；配置错误或裸板固件返回 0。
 * @note 调用方须在同一中断临界区检查安全状态；成功不代表实际负载已有电压。
 */
uint8_t PlatformPowerEnable(PlatformPowerRail_t rail);
/** @brief 查询电源引脚是否仍保持已命令的高电平。
 * @param[in] rail 板级声明的电源编号。
 * @return 输出模式且锁存为高电平返回 1；裸板或无效编号返回 0。
 * @note 此结果只反映 GPIO 命令，不能证明电源或负载电压正常。
 */
uint8_t PlatformPowerIsEnabled(PlatformPowerRail_t rail);
#endif
