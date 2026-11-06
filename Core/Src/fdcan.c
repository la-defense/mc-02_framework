/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    fdcan.c
  * @brief   This file provides code for the configuration
  *          of the FDCAN instances.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "fdcan.h"

/* USER CODE BEGIN 0 */
#include <stddef.h> // NULL 用于检查 HAL 传入的控制器句柄。
// SRAMCAN 以 32 位字计址；接收元素预留64字节，避免 HAL 按异常 DLC 读取相邻元素。
#define MC02_FDCAN_RAM_WORDS (0x2800u / 4u) // H723 的共享消息 RAM 总字数。
#define MC02_FDCAN_STD_FILTERS 14u // 每条总线保留14个标准过滤器。
#define MC02_FDCAN_EXT_FILTERS 16u // 每条总线保留16个扩展过滤器，每项占2字。
#define MC02_FDCAN_RX_FIFO_ITEMS 4u // 每个接收 FIFO 保持4个元素。
#define MC02_FDCAN_TX_FIFO_ITEMS 16u // 每个发送 FIFO 保持16个经典 CAN 元素。
#define MC02_FDCAN_RAM_OFFSET_1 0u // 第一条总线的消息 RAM 起始字偏移。
#define MC02_FDCAN_RAM_OFFSET_2 0x406u // 第二条总线的消息 RAM 起始字偏移。
#define MC02_FDCAN_RAM_OFFSET_3 0x812u // 第三条总线的消息 RAM 起始字偏移。
#define MC02_FDCAN_WORDS_PER_BUS (MC02_FDCAN_STD_FILTERS + 2u * MC02_FDCAN_EXT_FILTERS + \
  2u * MC02_FDCAN_RX_FIFO_ITEMS * FDCAN_DATA_BYTES_64 + MC02_FDCAN_TX_FIFO_ITEMS * FDCAN_DATA_BYTES_8)
_Static_assert(MC02_FDCAN_RAM_OFFSET_1 + MC02_FDCAN_WORDS_PER_BUS <= MC02_FDCAN_RAM_OFFSET_2,
               "FDCAN1 and FDCAN2 message RAM overlaps");
_Static_assert(MC02_FDCAN_RAM_OFFSET_2 + MC02_FDCAN_WORDS_PER_BUS <= MC02_FDCAN_RAM_OFFSET_3,
               "FDCAN2 and FDCAN3 message RAM overlaps");
_Static_assert(MC02_FDCAN_RAM_OFFSET_3 + MC02_FDCAN_WORDS_PER_BUS <= MC02_FDCAN_RAM_WORDS,
               "FDCAN3 message RAM exceeds SRAMCAN");
// 三个控制器共用一个时钟；每位记录一个已完成 MSP 初始化的控制器。
static uint8_t fdcan_clock_users;

// 只接受本板声明的三个控制器，未知地址不能改变共享时钟。
static uint8_t FdcanClockUserBit(const FDCAN_GlobalTypeDef *instance)
{
  if (instance == FDCAN1) return 1u << 0; // FDCAN1 独占第 0 位。
  if (instance == FDCAN2) return 1u << 1; // FDCAN2 独占第 1 位。
  if (instance == FDCAN3) return 1u << 2; // FDCAN3 独占第 2 位。
  return 0u; // 未知控制器不能取得资源。
}

// 首个使用者配置并开启共享时钟；调用方须串行初始化和退出控制器。
static uint8_t FdcanClockAcquire(uint8_t bit)
{
  if (bit == 0u) return 0u; // 无效控制器不能配置时钟。
  if ((fdcan_clock_users & bit) != 0u) return 1u; // 重复调用不增加占用次数。
  if (fdcan_clock_users == 0u)
  {
    RCC_PeriphCLKInitTypeDef clock_config = {0}; // 首次使用前准备完整时钟选择。
    clock_config.PeriphClockSelection = RCC_PERIPHCLK_FDCAN; // 三个 FDCAN 共用选择项。
    clock_config.FdcanClockSelection = RCC_FDCANCLKSOURCE_PLL; // 沿用当前 PLL 时钟源。
    if (HAL_RCCEx_PeriphCLKConfig(&clock_config) != HAL_OK) return 0u; // 失败时不登记使用者。
    __HAL_RCC_FDCAN_CLK_ENABLE(); // 时钟源配置成功后才开启外设时钟。
  }
  fdcan_clock_users |= bit; // 记录本控制器，供后续独立退出。
  return 1u;
}

// 最后一个使用者退出后关闭时钟；重复退出不改变其他使用者。
static void FdcanClockRelease(uint8_t bit)
{
  if (bit == 0u || (fdcan_clock_users & bit) == 0u) return; // 未持有资源时不操作。
  fdcan_clock_users &= (uint8_t)~bit; // 清除本控制器的使用标记。
  if (fdcan_clock_users == 0u) __HAL_RCC_FDCAN_CLK_DISABLE(); // 无其他使用者时关闭时钟。
}

// 已配置过的控制器无需重复配置 GPIO；时钟失败仍沿用当前错误入口。
static uint8_t FdcanMspBegin(FDCAN_HandleTypeDef *handle)
{
  if (handle == NULL) return 0u; // 空句柄不能申请板级资源。
  const uint8_t bit = FdcanClockUserBit(handle->Instance); // 按控制器确定占用标记。
  if (bit == 0u || (fdcan_clock_users & bit) != 0u) return 0u; // 无效或重复初始化不改状态。
  if (FdcanClockAcquire(bit) == 0u)
  {
    Error_Handler(); // 当前生成入口仍使用统一错误处理，协调器接入后改为报告结果。
    return 0u; // 测试替身返回时不能继续配置 GPIO。
  }
  return 1u; // 已获得共享时钟，可以配置本控制器的引脚和中断。
}

// 只允许退出持有共享时钟的控制器。
static uint8_t FdcanMspCanEnd(FDCAN_HandleTypeDef *handle)
{
  if (handle == NULL) return 0u; // 空句柄无资源可退出。
  const uint8_t bit = FdcanClockUserBit(handle->Instance); // 找到本控制器的占用位。
  return (uint8_t)(bit != 0u && (fdcan_clock_users & bit) != 0u); // 重复退出不影响其他总线。
}

/* USER CODE END 0 */

FDCAN_HandleTypeDef hfdcan1;
FDCAN_HandleTypeDef hfdcan2;
FDCAN_HandleTypeDef hfdcan3;

/* FDCAN1 init function */
void MX_FDCAN1_Init(void)
{

  /* USER CODE BEGIN FDCAN1_Init 0 */

  /* USER CODE END FDCAN1_Init 0 */

  /* USER CODE BEGIN FDCAN1_Init 1 */

  /* USER CODE END FDCAN1_Init 1 */
  hfdcan1.Instance = FDCAN1;
  hfdcan1.Init.FrameFormat = FDCAN_FRAME_CLASSIC;
  hfdcan1.Init.Mode = FDCAN_MODE_NORMAL;
  hfdcan1.Init.AutoRetransmission = DISABLE;
  hfdcan1.Init.TransmitPause = DISABLE;
  hfdcan1.Init.ProtocolException = ENABLE;
  hfdcan1.Init.NominalPrescaler = 3;
  hfdcan1.Init.NominalSyncJumpWidth = 1;
  hfdcan1.Init.NominalTimeSeg1 = 29;
  hfdcan1.Init.NominalTimeSeg2 = 10;
  hfdcan1.Init.DataPrescaler = 3;
  hfdcan1.Init.DataSyncJumpWidth = 10;
  hfdcan1.Init.DataTimeSeg1 = 29;
  hfdcan1.Init.DataTimeSeg2 = 10;
  hfdcan1.Init.MessageRAMOffset = MC02_FDCAN_RAM_OFFSET_1; // 第一条总线的共享 RAM 起点。
  hfdcan1.Init.StdFiltersNbr = MC02_FDCAN_STD_FILTERS; // 标准 ID 过滤器数量。
  hfdcan1.Init.ExtFiltersNbr = MC02_FDCAN_EXT_FILTERS; // 扩展 ID 过滤器数量。
  hfdcan1.Init.RxFifo0ElmtsNbr = MC02_FDCAN_RX_FIFO_ITEMS; // FIFO0 元素数。
  hfdcan1.Init.RxFifo0ElmtSize = FDCAN_DATA_BYTES_64; // 容纳 HAL 对异常 DLC 的最大读取。
  hfdcan1.Init.RxFifo1ElmtsNbr = MC02_FDCAN_RX_FIFO_ITEMS; // FIFO1 元素数。
  hfdcan1.Init.RxFifo1ElmtSize = FDCAN_DATA_BYTES_64; // FIFO1 与 FIFO0 使用相同边界。
  hfdcan1.Init.RxBuffersNbr = 0;
  hfdcan1.Init.RxBufferSize = FDCAN_DATA_BYTES_8;
  hfdcan1.Init.TxEventsNbr = 0;
  hfdcan1.Init.TxBuffersNbr = 0;
  hfdcan1.Init.TxFifoQueueElmtsNbr = MC02_FDCAN_TX_FIFO_ITEMS; // 经典 CAN 发送队列容量。
  hfdcan1.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
  hfdcan1.Init.TxElmtSize = FDCAN_DATA_BYTES_8;
  if (HAL_FDCAN_Init(&hfdcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN FDCAN1_Init 2 */

  /* USER CODE END FDCAN1_Init 2 */

}
/* FDCAN2 init function */
void MX_FDCAN2_Init(void)
{

  /* USER CODE BEGIN FDCAN2_Init 0 */

  /* USER CODE END FDCAN2_Init 0 */

  /* USER CODE BEGIN FDCAN2_Init 1 */

  /* USER CODE END FDCAN2_Init 1 */
  hfdcan2.Instance = FDCAN2;
  hfdcan2.Init.FrameFormat = FDCAN_FRAME_CLASSIC;
  hfdcan2.Init.Mode = FDCAN_MODE_NORMAL;
  hfdcan2.Init.AutoRetransmission = DISABLE;
  hfdcan2.Init.TransmitPause = DISABLE;
  hfdcan2.Init.ProtocolException = DISABLE;
  hfdcan2.Init.NominalPrescaler = 3;
  hfdcan2.Init.NominalSyncJumpWidth = 1;
  hfdcan2.Init.NominalTimeSeg1 = 29;
  hfdcan2.Init.NominalTimeSeg2 = 10;
  hfdcan2.Init.DataPrescaler = 3;
  hfdcan2.Init.DataSyncJumpWidth = 10;
  hfdcan2.Init.DataTimeSeg1 = 29;
  hfdcan2.Init.DataTimeSeg2 = 10;
  hfdcan2.Init.MessageRAMOffset = MC02_FDCAN_RAM_OFFSET_2; // 第二条总线的共享 RAM 起点。
  hfdcan2.Init.StdFiltersNbr = MC02_FDCAN_STD_FILTERS; // 标准 ID 过滤器数量。
  hfdcan2.Init.ExtFiltersNbr = MC02_FDCAN_EXT_FILTERS; // 扩展 ID 过滤器数量。
  hfdcan2.Init.RxFifo0ElmtsNbr = MC02_FDCAN_RX_FIFO_ITEMS; // FIFO0 元素数。
  hfdcan2.Init.RxFifo0ElmtSize = FDCAN_DATA_BYTES_64; // 保证 HAL 最大读取不跨元素。
  hfdcan2.Init.RxFifo1ElmtsNbr = MC02_FDCAN_RX_FIFO_ITEMS; // FIFO1 元素数。
  hfdcan2.Init.RxFifo1ElmtSize = FDCAN_DATA_BYTES_64; // FIFO1 使用相同边界。
  hfdcan2.Init.RxBuffersNbr = 0;
  hfdcan2.Init.RxBufferSize = FDCAN_DATA_BYTES_8;
  hfdcan2.Init.TxEventsNbr = 0;
  hfdcan2.Init.TxBuffersNbr = 0;
  hfdcan2.Init.TxFifoQueueElmtsNbr = MC02_FDCAN_TX_FIFO_ITEMS; // 经典 CAN 发送队列容量。
  hfdcan2.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
  hfdcan2.Init.TxElmtSize = FDCAN_DATA_BYTES_8;
  if (HAL_FDCAN_Init(&hfdcan2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN FDCAN2_Init 2 */

  /* USER CODE END FDCAN2_Init 2 */

}
/* FDCAN3 init function */
void MX_FDCAN3_Init(void)
{

  /* USER CODE BEGIN FDCAN3_Init 0 */

  /* USER CODE END FDCAN3_Init 0 */

  /* USER CODE BEGIN FDCAN3_Init 1 */

  /* USER CODE END FDCAN3_Init 1 */
  hfdcan3.Instance = FDCAN3;
  hfdcan3.Init.FrameFormat = FDCAN_FRAME_CLASSIC;
  hfdcan3.Init.Mode = FDCAN_MODE_NORMAL;
  hfdcan3.Init.AutoRetransmission = DISABLE;
  hfdcan3.Init.TransmitPause = DISABLE;
  hfdcan3.Init.ProtocolException = DISABLE;
  hfdcan3.Init.NominalPrescaler = 3;
  hfdcan3.Init.NominalSyncJumpWidth = 1;
  hfdcan3.Init.NominalTimeSeg1 = 29;
  hfdcan3.Init.NominalTimeSeg2 = 10;
  hfdcan3.Init.DataPrescaler = 3;
  hfdcan3.Init.DataSyncJumpWidth = 10;
  hfdcan3.Init.DataTimeSeg1 = 29;
  hfdcan3.Init.DataTimeSeg2 = 10;
  hfdcan3.Init.MessageRAMOffset = MC02_FDCAN_RAM_OFFSET_3; // 第三条总线的共享 RAM 起点。
  hfdcan3.Init.StdFiltersNbr = MC02_FDCAN_STD_FILTERS; // 标准 ID 过滤器数量。
  hfdcan3.Init.ExtFiltersNbr = MC02_FDCAN_EXT_FILTERS; // 扩展 ID 过滤器数量。
  hfdcan3.Init.RxFifo0ElmtsNbr = MC02_FDCAN_RX_FIFO_ITEMS; // FIFO0 元素数。
  hfdcan3.Init.RxFifo0ElmtSize = FDCAN_DATA_BYTES_64; // 保证 HAL 最大读取不跨元素。
  hfdcan3.Init.RxFifo1ElmtsNbr = MC02_FDCAN_RX_FIFO_ITEMS; // FIFO1 元素数。
  hfdcan3.Init.RxFifo1ElmtSize = FDCAN_DATA_BYTES_64; // FIFO1 使用相同边界。
  hfdcan3.Init.RxBuffersNbr = 0;
  hfdcan3.Init.RxBufferSize = FDCAN_DATA_BYTES_8;
  hfdcan3.Init.TxEventsNbr = 0;
  hfdcan3.Init.TxBuffersNbr = 0;
  hfdcan3.Init.TxFifoQueueElmtsNbr = MC02_FDCAN_TX_FIFO_ITEMS; // 经典 CAN 发送队列容量。
  hfdcan3.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
  hfdcan3.Init.TxElmtSize = FDCAN_DATA_BYTES_8;
  if (HAL_FDCAN_Init(&hfdcan3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN FDCAN3_Init 2 */

  /* USER CODE END FDCAN3_Init 2 */

}

void HAL_FDCAN_MspInit(FDCAN_HandleTypeDef* fdcanHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if (fdcanHandle == NULL) return; // 空句柄不能操作板级资源。
  if(fdcanHandle->Instance==FDCAN1)
  {
  /* USER CODE BEGIN FDCAN1_MspInit 0 */
    if (!FdcanMspBegin(fdcanHandle)) return; // 首次取得共享时钟；重复初始化直接退出。
  /* USER CODE END FDCAN1_MspInit 0 */

    __HAL_RCC_GPIOD_CLK_ENABLE();
    /**FDCAN1 GPIO Configuration
    PD0     ------> FDCAN1_RX
    PD1     ------> FDCAN1_TX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF9_FDCAN1;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

    /* FDCAN1 interrupt Init */
    HAL_NVIC_SetPriority(FDCAN1_IT0_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(FDCAN1_IT0_IRQn);
    HAL_NVIC_SetPriority(FDCAN1_IT1_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(FDCAN1_IT1_IRQn);
  /* USER CODE BEGIN FDCAN1_MspInit 1 */

  /* USER CODE END FDCAN1_MspInit 1 */
  }
  else if(fdcanHandle->Instance==FDCAN2)
  {
  /* USER CODE BEGIN FDCAN2_MspInit 0 */
    if (!FdcanMspBegin(fdcanHandle)) return; // 只在此控制器首次初始化时配置引脚。
  /* USER CODE END FDCAN2_MspInit 0 */

    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**FDCAN2 GPIO Configuration
    PB5     ------> FDCAN2_RX
    PB6     ------> FDCAN2_TX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_5|GPIO_PIN_6;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF9_FDCAN2;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* FDCAN2 interrupt Init */
    HAL_NVIC_SetPriority(FDCAN2_IT0_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(FDCAN2_IT0_IRQn);
    HAL_NVIC_SetPriority(FDCAN2_IT1_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(FDCAN2_IT1_IRQn);
  /* USER CODE BEGIN FDCAN2_MspInit 1 */

  /* USER CODE END FDCAN2_MspInit 1 */
  }
  else if(fdcanHandle->Instance==FDCAN3)
  {
  /* USER CODE BEGIN FDCAN3_MspInit 0 */
    if (!FdcanMspBegin(fdcanHandle)) return; // 与其他控制器共享已打开的时钟。
  /* USER CODE END FDCAN3_MspInit 0 */

    __HAL_RCC_GPIOD_CLK_ENABLE();
    /**FDCAN3 GPIO Configuration
    PD12     ------> FDCAN3_RX
    PD13     ------> FDCAN3_TX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF5_FDCAN3;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

    /* FDCAN3 interrupt Init */
    HAL_NVIC_SetPriority(FDCAN3_IT0_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(FDCAN3_IT0_IRQn);
    HAL_NVIC_SetPriority(FDCAN3_IT1_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(FDCAN3_IT1_IRQn);
  /* USER CODE BEGIN FDCAN3_MspInit 1 */

  /* USER CODE END FDCAN3_MspInit 1 */
  }
}

void HAL_FDCAN_MspDeInit(FDCAN_HandleTypeDef* fdcanHandle)
{
  if (fdcanHandle == NULL) return; // 空句柄不能释放板级资源。
  if(fdcanHandle->Instance==FDCAN1)
  {
  /* USER CODE BEGIN FDCAN1_MspDeInit 0 */
    if (!FdcanMspCanEnd(fdcanHandle)) return; // 未初始化或重复退出不释放其他总线资源。
  /* USER CODE END FDCAN1_MspDeInit 0 */
    HAL_NVIC_DisableIRQ(FDCAN1_IT0_IRQn); // 先阻止新中断访问正在释放的控制器。
    HAL_NVIC_DisableIRQ(FDCAN1_IT1_IRQn); // 关闭该控制器的第二条中断线。
    HAL_NVIC_ClearPendingIRQ(FDCAN1_IT0_IRQn); // 清除已挂起的第一条中断。
    HAL_NVIC_ClearPendingIRQ(FDCAN1_IT1_IRQn); // 清除已挂起的第二条中断。

    /**FDCAN1 GPIO Configuration
    PD0     ------> FDCAN1_RX
    PD1     ------> FDCAN1_TX
    */
    HAL_GPIO_DeInit(GPIOD, GPIO_PIN_0|GPIO_PIN_1);

  /* USER CODE BEGIN FDCAN1_MspDeInit 1 */
    FdcanClockRelease(FdcanClockUserBit(fdcanHandle->Instance)); // 本控制器退出后释放共享时钟。
  /* USER CODE END FDCAN1_MspDeInit 1 */
  }
  else if(fdcanHandle->Instance==FDCAN2)
  {
  /* USER CODE BEGIN FDCAN2_MspDeInit 0 */
    if (!FdcanMspCanEnd(fdcanHandle)) return; // 只退出当前持有资源的控制器。
  /* USER CODE END FDCAN2_MspDeInit 0 */
    HAL_NVIC_DisableIRQ(FDCAN2_IT0_IRQn); // 停止本控制器的第一条中断。
    HAL_NVIC_DisableIRQ(FDCAN2_IT1_IRQn); // 停止本控制器的第二条中断。
    HAL_NVIC_ClearPendingIRQ(FDCAN2_IT0_IRQn); // 清除本控制器的待处理中断。
    HAL_NVIC_ClearPendingIRQ(FDCAN2_IT1_IRQn); // 避免重新启用后处理旧事件。

    /**FDCAN2 GPIO Configuration
    PB5     ------> FDCAN2_RX
    PB6     ------> FDCAN2_TX
    */
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_5|GPIO_PIN_6);

  /* USER CODE BEGIN FDCAN2_MspDeInit 1 */
    FdcanClockRelease(FdcanClockUserBit(fdcanHandle->Instance)); // 其他总线使用时保留共享时钟。
  /* USER CODE END FDCAN2_MspDeInit 1 */
  }
  else if(fdcanHandle->Instance==FDCAN3)
  {
  /* USER CODE BEGIN FDCAN3_MspDeInit 0 */
    if (!FdcanMspCanEnd(fdcanHandle)) return; // 未持有时钟的控制器不修改 GPIO 或中断。
  /* USER CODE END FDCAN3_MspDeInit 0 */
    HAL_NVIC_DisableIRQ(FDCAN3_IT0_IRQn); // 停止本控制器的第一条中断。
    HAL_NVIC_DisableIRQ(FDCAN3_IT1_IRQn); // 停止本控制器的第二条中断。
    HAL_NVIC_ClearPendingIRQ(FDCAN3_IT0_IRQn); // 清除本控制器的待处理中断。
    HAL_NVIC_ClearPendingIRQ(FDCAN3_IT1_IRQn); // 避免重新启用后处理旧事件。

    /**FDCAN3 GPIO Configuration
    PD12     ------> FDCAN3_RX
    PD13     ------> FDCAN3_TX
    */
    HAL_GPIO_DeInit(GPIOD, GPIO_PIN_12|GPIO_PIN_13);

  /* USER CODE BEGIN FDCAN3_MspDeInit 1 */
    FdcanClockRelease(FdcanClockUserBit(fdcanHandle->Instance)); // 最后一个使用者才关闭时钟。
  /* USER CODE END FDCAN3_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
