/** @file led.h
 * @brief 为告警策略提供单颗板载 RGB 灯的结果接口。
 * @note RobotAlarmInit() 在任务启动前初始化；之后由 RobotAlarmTask() 所在普通任务串行提交。
 *       本模块不保存颜色或闪烁状态。
 */
#ifndef MC02_ALARM_LED_H
#define MC02_ALARM_LED_H

#include <stdint.h>

typedef enum
{
    LED_STATUS_OK = 0,
    LED_STATUS_ARGUMENT,
    LED_STATUS_UNINITIALIZED,
    LED_STATUS_BUSY,
    LED_STATUS_TRANSFER,
} LEDStatus_t;

/** @brief 初始化 SPI6 RGB 输出。
 * @return 成功为 LED_STATUS_OK；资源忙为 BUSY，其余初始化失败为 TRANSFER。
 * @note 可重复调用；不调用 Error_Handler，也不修改 ADC 按键 PA5。
 */
LEDStatus_t LEDInit(void);

/** @brief 在普通任务推进 LED 发送和超时恢复，不改变 safety 状态。
 * @note 由唯一告警任务约每10ms调用；初始化失败时不执行操作。
 */
void LEDProcess(void);
/** @brief 提交一组 RGB 颜色。
 * @param[in] r,g,b 红、绿、蓝分量，范围 0～255。
 * @return 对应颜色本机 SPI 发送完成为 OK；待启动／在途为 BUSY，其余失败返回对应状态。
 * @note 普通任务串行调用；本机发送完成不代表灯珠已显示。
 */
LEDStatus_t LEDSetRGB(uint8_t r, uint8_t g, uint8_t b);

#endif /* MC02_ALARM_LED_H */
