/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    spi.c
  * @brief   This file provides code for the configuration
  *          of the SPI instances.
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
#include "spi.h"
#include <stddef.h>

/* USER CODE BEGIN 0 */
static uint8_t spi1_clock_prepared; // 标记本次受检查启动已准备 SPI1 时钟。
static uint8_t spi1_pins_prepared; // 记录 MSP 已取得的 SPI1 引脚和时钟。
static uint8_t spi2_clock_prepared; // 标记本次受检查启动已准备 SPI2 时钟。
static uint8_t spi2_pins_prepared; // 记录 MSP 已取得的 SPI2 引脚和时钟。

/* USER CODE END 0 */

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;

/* SPI1 init function */
void MX_SPI1_Init(void)
{
  if (!MX_SPI1_InitChecked()) Error_Handler(); // 保留 CubeMX 兼容入口的致命失败行为。
}

uint8_t MX_SPI1_InitChecked(void)
{
  RCC_PeriphCLKInitTypeDef spi1_clock = {0}; // 在 HAL 配置寄存器前检查时钟设置。
  spi1_clock.PeriphClockSelection = RCC_PERIPHCLK_SPI1; // 指定 LCD 使用的 SPI1。
  spi1_clock.Spi123ClockSelection = RCC_SPI123CLKSOURCE_PLL; // 沿用 CubeMX 的 PLL 来源。
  if (HAL_RCCEx_PeriphCLKConfig(&spi1_clock) != HAL_OK)
  {
    if (spi1_pins_prepared != 0u) HAL_SPI_MspDeInit(&hspi1); // 若此前已取得资源则释放。
    hspi1.Instance = NULL; // 不让上层把失败句柄当作可用总线。
    return 0u; // 时钟失败时不调用 HAL_SPI_Init。
  }
  spi1_clock_prepared = 1u; // MSP 回调复用刚刚确认的时钟配置。

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES_TXONLY;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  /* MC02-0046: ST7789V2 samples MOSI on rising SCL; with idle-high SCL,
     use Mode 3 (second edge), launching MOSI on falling edges.
     CLKPhase is also recorded in Basic_Framework_MC02.ioc.
     !! 手工改动(2026-09-26, LCD-08): CubeMX 重新生成会覆盖这里 !!
     SPI1 的时钟源是 PLL1Q = 120MHz(main.c 没配 PeriphClkInit, 用的是默认),
     所以分频 8 → 15MHz, 分频 4 → **30MHz**。LCD(ST7789V2) 在 30MHz 下的写周期
     余量足够, 把刷新一帧的 SPI 传输时间砍掉一半。
     若实测出现花屏/错位, 把它改回 SPI_BAUDRATEPRESCALER_8 即可
     (LCD 那边的"按内容变化重绘"优化不受影响)。 */
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 0x0;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  hspi1.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
  hspi1.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
  hspi1.Init.TxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi1.Init.RxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi1.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
  hspi1.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
  hspi1.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
  /* Keep idle SCK driven while HAL closes/reopens SPI after GPIO CS is selected. */
  hspi1.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_ENABLE;
  hspi1.Init.IOSwap = SPI_IO_SWAP_DISABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    if (spi1_pins_prepared != 0u) HAL_SPI_MspDeInit(&hspi1); // 只释放本次已取得的资源。
    spi1_clock_prepared = 0u; // 不让下次初始化误用旧状态。
    hspi1.Instance = NULL; // 失败后不能再通过 LCD 就绪检查。
    return 0u;
  }
  spi1_clock_prepared = 0u; // HAL 初始化结束后不保留本次标记。
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

  return 1u; // 将实际初始化结果交给启动协调流程。
}
/* SPI2 init function */
void MX_SPI2_Init(void)
{
  if (!MX_SPI2_InitChecked()) Error_Handler(); // 保留原入口的致命失败行为。
}

uint8_t MX_SPI2_InitChecked(void)
{
  RCC_PeriphCLKInitTypeDef spi2_clock = {0}; // 在访问 SPI2 寄存器前检查时钟。
  spi2_clock.PeriphClockSelection = RCC_PERIPHCLK_SPI2; // 指定板载 IMU 使用的 SPI2。
  spi2_clock.Spi123ClockSelection = RCC_SPI123CLKSOURCE_PLL; // 保持 CubeMX 的 PLL 来源。
  if (HAL_RCCEx_PeriphCLKConfig(&spi2_clock) != HAL_OK)
  {
    if (spi2_pins_prepared != 0u) HAL_SPI_MspDeInit(&hspi2); // 释放本控制器之前取得的资源。
    hspi2.Instance = NULL; // 无效句柄不能用于 IMU 注册。
    return 0u; // 时钟失败时不访问 SPI2 控制器。
  }
  spi2_clock_prepared = 1u; // MSP 回调使用本次已确认的时钟配置。

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi2.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 0x0;
  hspi2.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  hspi2.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
  hspi2.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
  hspi2.Init.TxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi2.Init.RxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi2.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
  hspi2.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
  hspi2.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
  hspi2.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_DISABLE;
  hspi2.Init.IOSwap = SPI_IO_SWAP_DISABLE;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    if (spi2_pins_prepared != 0u) HAL_SPI_MspDeInit(&hspi2); // 只释放 SPI2 的资源。
    spi2_clock_prepared = 0u; // 不让后续操作误用本次时钟状态。
    hspi2.Instance = NULL; // 初始化失败不能作为已就绪设备。
    return 0u;
  }
  spi2_clock_prepared = 0u; // 结束本次检查，后续独立调用仍检查时钟。
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

  return 1u; // 向启动计划报告 SPI2 已准备。
}

void HAL_SPI_MspInit(SPI_HandleTypeDef* spiHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};
  if(spiHandle->Instance==SPI1)
  {
  /* USER CODE BEGIN SPI1_MspInit 0 */

  /* USER CODE END SPI1_MspInit 0 */

  /** Initializes the peripherals clock
  */
    PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_SPI1;
    PeriphClkInitStruct.Spi123ClockSelection = RCC_SPI123CLKSOURCE_PLL;
    if (spi1_clock_prepared == 0u && HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
    {
      Error_Handler();
    }

    /* SPI1 clock enable */
    __HAL_RCC_SPI1_CLK_ENABLE();

    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**SPI1 GPIO Configuration
    PD7     ------> SPI1_MOSI
    PB3(JTDO/TRACESWO)     ------> SPI1_SCK
    */
    GPIO_InitStruct.Pin = GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_3;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    spi1_pins_prepared = 1u; // 之后失败时允许只释放 SPI1 取得的资源。

  /* USER CODE BEGIN SPI1_MspInit 1 */

  /* USER CODE END SPI1_MspInit 1 */
  }
  else if(spiHandle->Instance==SPI2)
  {
  /* USER CODE BEGIN SPI2_MspInit 0 */

  /* USER CODE END SPI2_MspInit 0 */

  /** Initializes the peripherals clock
  */
    PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_SPI2;
    PeriphClkInitStruct.Spi123ClockSelection = RCC_SPI123CLKSOURCE_PLL;
    if (spi2_clock_prepared == 0u && HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
    {
      Error_Handler();
    }

    /* SPI2 clock enable */
    __HAL_RCC_SPI2_CLK_ENABLE();

    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**SPI2 GPIO Configuration
    PC1     ------> SPI2_MOSI
    PC2_C     ------> SPI2_MISO
    PB13     ------> SPI2_SCK
    */
    GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_2;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_13;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    spi2_pins_prepared = 1u; // 标记 SPI2 引脚已经完成配置。

  /* USER CODE BEGIN SPI2_MspInit 1 */

  /* USER CODE END SPI2_MspInit 1 */
  }
}

void HAL_SPI_MspDeInit(SPI_HandleTypeDef* spiHandle)
{

  if(spiHandle->Instance==SPI1)
  {
  /* USER CODE BEGIN SPI1_MspDeInit 0 */

  /* USER CODE END SPI1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_SPI1_CLK_DISABLE();

    /**SPI1 GPIO Configuration
    PD7     ------> SPI1_MOSI
    PB3(JTDO/TRACESWO)     ------> SPI1_SCK
    */
    HAL_GPIO_DeInit(GPIOD, GPIO_PIN_7);

    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_3);
    spi1_pins_prepared = 0u; // 资源释放后避免重复退出。

  /* USER CODE BEGIN SPI1_MspDeInit 1 */

  /* USER CODE END SPI1_MspDeInit 1 */
  }
  else if(spiHandle->Instance==SPI2)
  {
  /* USER CODE BEGIN SPI2_MspDeInit 0 */

  /* USER CODE END SPI2_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_SPI2_CLK_DISABLE();

    /**SPI2 GPIO Configuration
    PC1     ------> SPI2_MOSI
    PC2_C     ------> SPI2_MISO
    PB13     ------> SPI2_SCK
    */
    HAL_GPIO_DeInit(GPIOC, GPIO_PIN_1|GPIO_PIN_2);

    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_13);
    spi2_pins_prepared = 0u; // 清除已释放的 SPI2 资源标记。

  /* USER CODE BEGIN SPI2_MspDeInit 1 */

  /* USER CODE END SPI2_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
