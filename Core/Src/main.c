/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "main.h"
#include "bsp_safety.h"
#include "cmsis_os.h"
#include "adc.h"
#include "dma.h"
#include "fdcan.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "bsp_observation.h"
#include "robot.h"
#include "bsp_log.h"
#include "bsp_watchdog.h"
#include "init_diagnostic.h"
#include "robot_safety.h"
#include "bsp_reset_recovery.h"
#include "platform_clock_startup.h"
#include "platform_feature_startup.h"
#include "config.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
uint8_t MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static __attribute__((noreturn)) void StartupDiagnosticLoop(void)
{
  for (;;)
  {
    // ������δ����ʱÿ��ֻ�ƽ�һ����ϣ�����ʧ��ѭ���ȴ� RTT �ռ䡣
    (void)BSPLogProcessPending(1u);
    BSP_SafetyStartupLatchOutputsOff();
    BSP_WatchdogFeed();
    InitDiagnosticHeartbeat();
    for (volatile uint32_t spin = 0u; spin < 200000u; ++spin)
      __NOP();
  }
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  BSP_SafetyStartupLatchOutputsOff();
  // 在看门狗清除RCC标志前保存本次复位来源。
  (void)BSP_ResetRecoveryInit(RCC->RSR);
  if (!BSPObservationInitEarly())
  {
    Error_Handler();
  }

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  // 基础 HAL 时基失败时立即关断并记录。
  if (HAL_Init() != HAL_OK)
    PlatformClockStartupFail(PLATFORM_CLOCK_FAILURE_HAL);

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  PlatformFeaturePeripheralsInit();
  // 在设备注册前检查车型配置，避免使用错误的串口编号。
  const Robot_Config_s *startup_config = NULL;
  if (RobotConfigSelect(ROBOT_CONFIG_SUPPORTED_ID, &startup_config) != ROBOT_CONFIG_OK ||
      startup_config == NULL)
    Error_Handler(); // 配置无效时保持受控输出关闭。
  PlatformUartPort_t startup_uarts[3]; // 最多包含裁判、遥控与视觉各一项。
  uint8_t startup_uart_count = 0u;
#if MC02_FEATURE_REFEREE
  startup_uarts[startup_uart_count++] = startup_config->referee_uart; // 裁判接收端口。
#endif
#if MC02_FEATURE_REMOTE
  startup_uarts[startup_uart_count++] = startup_config->remote_uart; // 遥控输入端口。
#endif
#if MC02_FEATURE_VISION && defined(VISION_USE_UART)
  startup_uarts[startup_uart_count++] = startup_config->vision_uart; // UART 视觉端口。
#endif
  // 板级入口先检查全部编号，再初始化所需端口及诊断串口。
  if (!PlatformFeatureUartsInit(startup_uarts, startup_uart_count))
    Error_Handler(); // 端口编号不受支持时停止启动。
  MX_FDCAN1_Init();
  MX_FDCAN2_Init();
  MX_FDCAN3_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  /* OCTOSPI 未被使用；重新生成后检查不得恢复初始化。 */
  /* PB10/PB11 released from I2C2: used as LCD_BLK/LCD_RES */
  /* USER CODE BEGIN 2 */
  if (!RobotInit())
    StartupDiagnosticLoop(); // Ψһ�ĳ�ʼ������
  BSP_LOG_EVENT_0(EV_CORE_SRC_MAIN_C_01);
  /* USER CODE END 2 */

  /* Call init function for freertos objects (in freertos.c) */
  if (!MX_FREERTOS_Init())
  {
    InitDiagnosticReportFailure(INIT_STAGE_FREERTOS, INIT_MODULE_FREERTOS,
                                INIT_FAILURE_TASK_CREATE, INIT_RESOURCE_TASK_DEFAULT);
    RobotFaultReport(ROBOT_FAULT_INIT_FAILURE);
    RobotSafetyRequestDisable();
    StartupDiagnosticLoop();
  }

  /* Start scheduler */
  if (osKernelStart() != osOK)
  {
    InitDiagnosticReportFailure(INIT_STAGE_FREERTOS, INIT_MODULE_FREERTOS,
                                INIT_FAILURE_SCHEDULER_RETURNED, 0u);
    RobotFaultReport(ROBOT_FAULT_INIT_FAILURE);
    RobotSafetyRequestDisable();
    StartupDiagnosticLoop();
  }

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    StartupDiagnosticLoop();
//	  vofa_start();

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  // 为整次时钟配置设置固定总期限。
  PlatformClockStartupBegin();

  /** Supply configuration update enable
  */
  // 供电配置失败时不再切换电压。
  if (HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY) != HAL_OK)
    PlatformClockStartupFail(PLATFORM_CLOCK_FAILURE_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  // VOS 计时和查询次数均有上限。
  if (!PlatformClockStartupWaitVos())
    PlatformClockStartupFail(PLATFORM_CLOCK_FAILURE_VOS);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48|RCC_OSCILLATORTYPE_HSI
                              |RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = 64;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 2;
  RCC_OscInitStruct.PLL.PLLN = 40;
  RCC_OscInitStruct.PLL.PLLP = 1;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  // 振荡器或 PLL 配置失败时记录具体阶段。
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    PlatformClockStartupFail(PLATFORM_CLOCK_FAILURE_OSCILLATOR);

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  // 主时钟或分频切换失败时不进入外设启动。
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
    PlatformClockStartupFail(PLATFORM_CLOCK_FAILURE_SWITCH);
  // 验证 HAL 原始时基仍在前进，且总期限未超时。
  if (!PlatformClockStartupFinish())
    PlatformClockStartupFail(PLATFORM_CLOCK_FAILURE_DEADLINE);
  HAL_RCC_MCOConfig(RCC_MCO1, RCC_MCO1SOURCE_HSI, RCC_MCODIV_1);
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM23 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM23) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
