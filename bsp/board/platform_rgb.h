/** @file platform_rgb.h
 * @brief 单颗 RGB 指示灯的平台接口；发送由后端异步完成。
 */
#ifndef MC02_PLATFORM_RGB_H
#define MC02_PLATFORM_RGB_H
#include <stdint.h>
#include "platform_resources.h"

/** @brief 本机发送的完成、进行中和不可用状态。 */
typedef enum {
    PLATFORM_RGB_OK = 0, /**< 初始化或本机发送完成。 */
    PLATFORM_RGB_BUSY, /**< 资源正忙或发送尚未完成。 */
    PLATFORM_RGB_ERROR /**< 初始化或发送失败。 */
} PlatformRgbStatus_t;

/** @brief 初始化板载 RGB 输出。
 * @param[in] port 车型声明的逻辑 RGB 指示灯。
 * @return 初始化成功、资源忙或失败的状态。
 * @note 在任务启动前串行调用；后端定义实际资源与初始化条件。
 */
PlatformRgbStatus_t PlatformRgbInit(PlatformRgbPort_t port);

/** @brief 复制一组颜色并推进本机发送。
 * @param[in] r 红色分量，范围 0～255。
 * @param[in] g 绿色分量，范围 0～255。
 * @param[in] b 蓝色分量，范围 0～255。
 * @return 本机发送完成、待处理或失败；完成不代表灯珠已显示。
 * @note 由唯一普通任务串行调用，并定期调用 PlatformRgbProcess。
 */
PlatformRgbStatus_t PlatformRgbSet(uint8_t r, uint8_t g, uint8_t b);

/** @brief 在普通任务推进在途事务的完成和超时处理。
 * @note 仅初始化成功后调用；本接口不改变整车许可。
 */
void PlatformRgbProcess(void);
#endif
