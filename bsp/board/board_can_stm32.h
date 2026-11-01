/** @file board_can_stm32.h
 * @brief MC-02 板级 CAN 绑定的私有 STM32 适配入口。
 */
#ifndef MC02_BOARD_CAN_STM32_H
#define MC02_BOARD_CAN_STM32_H
#include "platform_resources.h"
#include "bsp_can.h"

/** @brief 根据车型逻辑总线查找 MC-02 板的 CAN 控制器。
 * @param[in] bus 框架声明的 CAN 总线编号。
 * @return 对应板级句柄；未声明的编号返回 NULL。
 * @note 仅供当前 STM32 适配路径使用；存在句柄不代表通信已启动或设备在线。
 */
FDCAN_HandleTypeDef *BoardCanResolve(PlatformCanBus_t bus);

/** @brief 将只读逻辑端点转换为 STM32 CAN 注册配置。
 * @param[in] endpoint 车型端点配置，不可为空。
 * @param[out] output 成功时接收句柄和 ID，失败时保持原值。
 * @return 1 为转换成功；0 为空参数或总线未绑定。
 * @note 此函数不注册设备；回调和所有者由对应驱动在注册前填写。
 */
uint8_t BoardCanPrepareEndpoint(const PlatformCanEndpoint_t *endpoint, CAN_Init_Config_s *output);
#endif
