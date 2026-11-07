/** @file platform_feature_startup.h
 * @brief 按固件启用的功能准备 MC-02 板载 SPI 和 PWM 外设。
 */
#ifndef MC02_PLATFORM_FEATURE_STARTUP_H
#define MC02_PLATFORM_FEATURE_STARTUP_H
#include <stdint.h>
#include "platform_resources.h"

/** @brief 在 GPIO 和 DMA 初始化之后、业务启动之前准备所需外设。
 * @note 当前生成的初始化函数遇到硬件错误时仍进入 Error_Handler；辅助降级尚未接入。
 */
void PlatformFeaturePeripheralsInit(void);
/** @brief 查询板载 LCD 所需 SPI1 是否在启动时配置成功。
 * @return 成功返回 1；未启用或初始化失败返回 0。
 */
uint8_t PlatformFeatureLcdReady(void);
/** @brief 查询板载 IMU 所需 SPI2 是否在启动时配置成功。
 * @return 成功返回 1；未启用或初始化失败返回 0。
 */
uint8_t PlatformFeatureImuReady(void);
/** @brief 查询 IMU 加热定时器是否准备成功。
 * @return 成功返回 1；未启用或失败返回 0。
 */
uint8_t PlatformFeatureHeaterReady(void);

/** @brief 按需初始化业务串口，同时保留本板 USART10 诊断串口。
 * @param[in] ports 待启用的逻辑串口数组；count 为零时允许 NULL。
 * @param[in] count 数组元素数，最多为 3。
 * @return 全部编号有效返回 1；无效时返回 0，且不初始化任何串口。
 * @note 仅在启动时调用一次；同次数组内相同端口只初始化一次。生成函数的硬件失败仍由 Error_Handler 处理。
 */
uint8_t PlatformFeatureUartsInit(const PlatformUartPort_t *ports, uint8_t count);

/** @brief 初始化车型或诊断请求的 CAN 控制器。
 * @param[in] buses 逻辑总线数组；count 为零时允许 NULL。
 * @param[in] count 数组元素数，最多为 8。
 * @return 编号有效返回 1；无效时返回 0 且不初始化任何控制器。各控制器结果另行查询。
 * @note 仅在启动时调用一次，数组中重复编号只初始化一次；硬件失败仍由 Error_Handler 处理。
 */
uint8_t PlatformFeatureCanInit(const PlatformCanBus_t *buses, uint8_t count);
/** @brief 查询一条 CAN 控制器是否完成板级准备。
 * @param[in] bus 逻辑总线编号。
 * @return 已准备返回 1；未选择、失败或编号无效返回 0。
 */
uint8_t PlatformFeatureCanReady(PlatformCanBus_t bus);
#endif
