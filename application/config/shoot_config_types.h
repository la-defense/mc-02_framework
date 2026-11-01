/** @file shoot_config_types.h
 * @brief 声明发射机构配置类型，并统一按机构类型查询摩擦轮数量。
 */
#ifndef MC02_SHOOT_CONFIG_TYPES_H
#define MC02_SHOOT_CONFIG_TYPES_H

#include <stdint.h>

/** @brief 发射机构配置类型。 */
typedef enum
{
    DUAL_SHOOT = 0,
    THREE_SHOOT,
    FOUR_SHOOT,
} Shoot_Type_e;

/** @brief 取得指定发射机构启用的摩擦轮数量。
 * @param[in] type 发射机构类型。
 * @return 有效类型返回2、3或4；未知类型返回0。
 */
static inline uint8_t ShootFrictionWheelCount(Shoot_Type_e type)
{
    switch (type)
    {
    case DUAL_SHOOT: return 2u;
    case THREE_SHOOT: return 3u;
    case FOUR_SHOOT: return 4u;
    default: return 0u;
    }
}

#endif
