/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    adc.c
  * @brief   This file provides code for the configuration
  *          of the ADC instances.
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
#include "adc.h"

/* USER CODE BEGIN 0 */
static uint8_t adc_checked_attempt; // 受检查启动时将 MSP 错误返回给调用者。
static uint8_t adc_msp_failed; // 记录时钟或 DMA 是否失败。
static uint8_t adc_msp_acquired; // 时钟准备后失败时需要释放本次资源。
static ADC_HandleTypeDef *adc_prepared; // HAL 再次调用 MSP 时跳过已经准备的资源。

/* USER CODE END 0 */

ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

/* ADC1 init function */
void MX_ADC1_Init(void)
{
  if (!MX_ADC1_InitChecked()) Error_Handler(); // 旧入口保留原有致命处理。
}

uint8_t MX_ADC1_InitChecked(void)
{
  if (hadc1.State != HAL_ADC_STATE_RESET) return 0u; // 不覆盖正在使用的 ADC。

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV64;
  hadc1.Init.Resolution = ADC_RESOLUTION_16B;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.NbrOfConversion = 2;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ConversionDataManagement = ADC_CONVERSIONDATA_DMA_CIRCULAR;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.LeftBitShift = ADC_LEFTBITSHIFT_NONE;
  hadc1.Init.OversamplingMode = DISABLE;
  adc_msp_failed = 0u; // 每次启动使用独立结果。
  adc_msp_acquired = 0u; // 当前尚未取得板级资源。
  adc_checked_attempt = 1u; // MSP 内的错误应返回而非进入致命处理。
  HAL_ADC_MspInit(&hadc1); // 在 HAL 访问 ADC 寄存器前准备硬件。
  adc_checked_attempt = 0u; // 此次板级准备已经结束。
  if (adc_msp_failed)
  {
    if (adc_msp_acquired) HAL_ADC_MspDeInit(&hadc1); // 清理本次已取得的 DMA 和引脚。
    hadc1.Instance = NULL; // 禁止后续调用使用失败句柄。
    return 0u;
  }
  adc_prepared = &hadc1; // HAL 内部回调不能再次分配相同资源。
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    goto adc_failure; // 保持外设时钟有效，交给 HAL 退出。
  }

  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    goto adc_failure; // 多模式失败时释放本 ADC。
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_32CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  sConfig.OffsetSignedSaturation = DISABLE;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    goto adc_failure; // 第一采样通道配置失败。
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_19;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    goto adc_failure; // 第二采样通道配置失败。
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

  adc_prepared = NULL; // 后续独立初始化不得复用本次标记。
  return 1u; // 时钟、DMA 和两个通道均准备完成。
adc_failure:
  adc_prepared = NULL; // 退出时恢复正常 MSP 行为。
  (void)HAL_ADC_DeInit(&hadc1); // 关闭 ADC 并释放已取得的板级资源。
  hadc1.Instance = NULL; // 上层不能注册失败的 ADC。
  return 0u;
}

void HAL_ADC_MspInit(ADC_HandleTypeDef* adcHandle)
{
  if (adc_prepared == adcHandle) return; // HAL 再次进入时不重复初始化。

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};
  if(adcHandle->Instance==ADC1)
  {
  /* USER CODE BEGIN ADC1_MspInit 0 */

  /* USER CODE END ADC1_MspInit 0 */

  /** Initializes the peripherals clock
  */
    PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_ADC;
    PeriphClkInitStruct.PLL2.PLL2M = 2;
    PeriphClkInitStruct.PLL2.PLL2N = 16;
    PeriphClkInitStruct.PLL2.PLL2P = 2;
    PeriphClkInitStruct.PLL2.PLL2Q = 2;
    PeriphClkInitStruct.PLL2.PLL2R = 2;
    PeriphClkInitStruct.PLL2.PLL2RGE = RCC_PLL2VCIRANGE_3;
    PeriphClkInitStruct.PLL2.PLL2VCOSEL = RCC_PLL2VCOWIDE;
    PeriphClkInitStruct.PLL2.PLL2FRACN = 0;
    PeriphClkInitStruct.AdcClockSelection = RCC_ADCCLKSOURCE_PLL2;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
    {
      if (adc_checked_attempt) { adc_msp_failed = 1u; return; } // 时钟失败立即交还启动入口。
      Error_Handler();
    }
    if (adc_checked_attempt) adc_msp_acquired = 1u; // 后续失败须释放该 ADC 的引脚和 DMA。

    /* ADC1 clock enable */
    __HAL_RCC_ADC12_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    /**ADC1 GPIO Configuration
    PA5     ------> ADC1_INP19
    PC4     ------> ADC1_INP4
    */
    GPIO_InitStruct.Pin = GPIO_PIN_5;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_4;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    /* ADC1 DMA Init */
    /* ADC1 Init */
    hdma_adc1.Instance = DMA1_Stream0;
    hdma_adc1.Init.Request = DMA_REQUEST_ADC1;
    hdma_adc1.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_adc1.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_adc1.Init.MemInc = DMA_MINC_ENABLE;
    hdma_adc1.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    hdma_adc1.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
    hdma_adc1.Init.Mode = DMA_CIRCULAR;
    hdma_adc1.Init.Priority = DMA_PRIORITY_LOW;
    hdma_adc1.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    __HAL_LINKDMA(adcHandle,DMA_Handle,hdma_adc1); // 先绑定，初始化失败时也可清理。
    if (HAL_DMA_Init(&hdma_adc1) != HAL_OK)
    {
      if (adc_checked_attempt) { adc_msp_failed = 1u; return; } // 报告 DMA 失败。
      Error_Handler();
    }

  /* USER CODE BEGIN ADC1_MspInit 1 */
    HAL_NVIC_DisableIRQ(DMA1_Stream0_IRQn); // 清理旧事件期间禁止进入处理函数。
    HAL_NVIC_ClearPendingIRQ(DMA1_Stream0_IRQn); // 不把上次采集的事件交给新实例。
    HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 5, 0); // 沿用现有 RTOS 中断优先级。
    HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn); // DMA 配置成功后才允许中断。
  /* USER CODE END ADC1_MspInit 1 */
  }
}

void HAL_ADC_MspDeInit(ADC_HandleTypeDef* adcHandle)
{

  if(adcHandle->Instance==ADC1)
  {
  /* USER CODE BEGIN ADC1_MspDeInit 0 */
    HAL_NVIC_DisableIRQ(DMA1_Stream0_IRQn); // 先停止 ADC 的 DMA 中断。
    HAL_NVIC_ClearPendingIRQ(DMA1_Stream0_IRQn); // 清除退出前残留的事件。

  /* USER CODE END ADC1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_ADC12_CLK_DISABLE();

    /**ADC1 GPIO Configuration
    PA5     ------> ADC1_INP19
    PC4     ------> ADC1_INP4
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_5);

    HAL_GPIO_DeInit(GPIOC, GPIO_PIN_4);

    /* ADC1 DMA DeInit */
    HAL_DMA_DeInit(adcHandle->DMA_Handle);
  /* USER CODE BEGIN ADC1_MspDeInit 1 */

  /* USER CODE END ADC1_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
