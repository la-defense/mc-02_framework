/**
 * @file bsp_init.h
 * @brief 声明基础 BSP 启动入口；由 RobotInit 在调度器启动前调用一次。
 */
#ifndef BSP_INIT_H
#define BSP_INIT_H
#include <stdint.h>

/** @brief 系统时钟就绪后启动周期计数和启动看门狗。
 * @return 1 表示看门狗配置成功；0 表示不允许继续外设初始化。
 * @note 在 GPIO、DMA 和其他外设初始化之前调用一次。
 */
uint8_t BSPStartupProtectionInit(void);

/**
 * @brief 准备 CAN 固定工作区、日志、ADC、崩溃记录和参数。
 * @return 1 表示启动保护有效且已执行其余步骤；0 表示看门狗未运行。
 * @note 先完成 HAL、480 MHz 系统时钟及所需外设的 MX 初始化，再由 RobotInit
 *       在调度器启动前调用一次。共享状态由各接口短暂保护，不要求全程关闭中断。
 *       本接口不提供重复初始化保护；CAN、UART 等实例资源仍在各自注册时初始化。
 */
uint8_t BSPInit(void);

#endif // BSP_INIT_H

