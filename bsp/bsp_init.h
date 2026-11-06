/**
 * @file bsp_init.h
 * @brief 声明基础 BSP 启动入口；由 RobotInit 在调度器启动前调用一次。
 */
#ifndef BSP_INIT_H
#define BSP_INIT_H
#include <stdint.h>

/**
 * @brief 按既定顺序准备计时、CAN固定工作区、看门狗、日志、ADC、崩溃记录和参数。
 * @return 1 表示看门狗已完成配置并执行了其余步骤；0 表示看门狗失败，后续步骤未运行。
 * @note 先完成 HAL、480 MHz 系统时钟及所需外设的 MX 初始化，再由 RobotInit
 *       在调度器启动前调用一次。中断状态由 RobotInit 保存和恢复。
 *       本接口不提供重复初始化保护；CAN、UART 等实例资源仍在各自注册时初始化。
 */
uint8_t BSPInit(void);

#endif // BSP_INIT_H

