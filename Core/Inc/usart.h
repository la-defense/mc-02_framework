/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    usart.h
  * @brief   This file contains all the function prototypes for
  *          the usart.c file
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
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __USART_H__
#define __USART_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

extern UART_HandleTypeDef huart5;

extern UART_HandleTypeDef huart7;

extern UART_HandleTypeDef huart8;

extern UART_HandleTypeDef huart9;

extern UART_HandleTypeDef huart1;

extern UART_HandleTypeDef huart2;

extern UART_HandleTypeDef huart3;

extern UART_HandleTypeDef huart10;

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

void MX_UART5_Init(void);
void MX_UART7_Init(void);
void MX_UART8_Init(void);
void MX_UART9_Init(void);
void MX_USART1_UART_Init(void);
void MX_USART2_UART_Init(void);
void MX_USART3_UART_Init(void);
void MX_USART10_UART_Init(void);

/* USER CODE BEGIN Prototypes */
/** @brief 受检查的串口启动入口，仅供启动阶段串行调用。
 * @return 成功返回 1；资源或外设初始化失败、或设备已在使用时返回 0。
 * @note 失败会释放本次取得的资源；重新生成 CubeMX 代码后须保留这些声明和实现。
 */
uint8_t MX_UART5_InitChecked(void); // UART5 初始化失败时返回 0。
uint8_t MX_UART7_InitChecked(void); // UART7 初始化失败时返回 0。
uint8_t MX_UART8_InitChecked(void); // UART8 初始化失败时返回 0。
uint8_t MX_UART9_InitChecked(void); // UART9 初始化失败时返回 0。
uint8_t MX_USART1_UART_InitChecked(void); // USART1 初始化失败时返回 0。
uint8_t MX_USART2_UART_InitChecked(void); // USART2 初始化失败时返回 0。
uint8_t MX_USART3_UART_InitChecked(void); // USART3 初始化失败时返回 0。
uint8_t MX_USART10_UART_InitChecked(void); // USART10 初始化失败时返回 0。

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __USART_H__ */

